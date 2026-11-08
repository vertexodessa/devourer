#include "aic8800/Aic8800Radio.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include "SignalStop.h"

using namespace devourer::aic8800;

namespace {

constexpr int kMonitorIfType = 4;
constexpr size_t kRxUrb = 32 * 1024;
/* Received monitor frames carry no FCS. */
constexpr bool kRxFcsPresent = false;
/* Station MAC for the monitor interface. The SA on air comes from each frame. */
constexpr uint8_t kVifMac[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};
constexpr int kRssiBiasDb = 110;

constexpr uint16_t k5gChannels[] = {36,  40,  44,  48,  52,  56,  60,  64,  100,
                                    104, 108, 112, 116, 120, 124, 128, 132, 136,
                                    140, 144, 149, 153, 157, 161, 165};

int channel_freq(uint8_t ch) {
  if (ch >= 1 && ch <= 13)
    return 2407 + 5 * ch;
  for (uint16_t c : k5gChannels)
    if (c == ch)
      return 5000 + 5 * ch;
  return 0;
}

std::string hex(const uint8_t *p, size_t n) {
  static const char *d = "0123456789abcdef";
  std::string s;
  s.reserve(n * 2);
  for (size_t i = 0; i < n; ++i) {
    s += d[p[i] >> 4];
    s += d[p[i] & 0xf];
  }
  return s;
}

} /* namespace */

Aic8800Radio::Aic8800Radio(libusb_device_handle *handle, libusb_context *ctx,
                           std::shared_ptr<devourer::UsbDeviceLock> usb_lock, Logger_t logger,
                           devourer::DeviceConfig cfg)
    : _h(handle), _ctx(ctx), _usb_lock(std::move(usb_lock)), _logger(std::move(logger)),
      _cfg(std::move(cfg)) {}

Aic8800Radio::~Aic8800Radio() { Stop(); }

bool Aic8800Radio::cmd(uint16_t id, uint16_t dest, const uint8_t *params, uint16_t len,
                       uint8_t *out, size_t out_max, unsigned timeout_ms) {
  auto msg = build_msg(id, dest, params, len);
  int done = 0;
  int rc = libusb_bulk_transfer(_h, kEpMsgOut, msg.data(), static_cast<int>(msg.size()), &done,
                                2000);
  if (rc != 0) {
    _logger->error("aic8800: message 0x{:04x} send failed: {}", id, libusb_error_name(rc));
    return false;
  }
  const uint16_t want = cfm_id(id);
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  uint8_t rx[4096];
  while (std::chrono::steady_clock::now() < deadline) {
    int n = 0;
    rc = libusb_bulk_transfer(_h, kEpMsgIn, rx, sizeof rx, &n, 200);
    if (rc == LIBUSB_ERROR_TIMEOUT && n == 0)
      continue;
    if (rc != 0 && rc != LIBUSB_ERROR_TIMEOUT) {
      _logger->error("aic8800: confirm read failed: {}", libusb_error_name(rc));
      return false;
    }
    bool found = false;
    walk_rx(
        rx, static_cast<size_t>(n), [](const uint8_t *, size_t) {},
        [&](uint8_t type, const uint8_t *rec, size_t rec_len) {
          if (type != kTypeCmdRsp || rec_len < 16 || get_le16(rec + 4) != want)
            return;
          found = true;
          if (out != nullptr) {
            const size_t plen = std::min<size_t>(get_le16(rec + 10), rec_len - 16);
            std::memcpy(out, rec + 16, std::min(plen, out_max));
          }
        });
    if (found)
      return true;
  }
  _logger->error("aic8800: no confirm for message 0x{:04x}", id);
  return false;
}

bool Aic8800Radio::firmware_init() {
  uint8_t p[256] = {};
  uint8_t out[64] = {};

  p[0] = 1;
  p[2] = 0x20;
  if (!cmd(kMmSetStackStart, kTaskMm, p, 4, out, sizeof out))
    return false;
  _logger->info("aic8800: stack started, is_5g_support={} vendor_info=0x{:02x}", out[0], out[1]);

  std::memset(p, 0, 24);
  put_le32(p, 0x0f8f);
  put_le32(p + 4, 0x0f0f);
  put_le32(p + 8, 0x0c34c008);
  put_le32(p + 16, 0x264203);
  if (!cmd(kMmSetRfCalib, kTaskMm, p, 24, out, sizeof out, 6000))
    return false;

  std::memset(p, 0, 4);
  p[0] = 1;
  if (!cmd(kMmGetMacAddr, kTaskMm, p, 4, out, sizeof out))
    return false;
  std::memcpy(_mac, out, 6);

  if (!cmd(kMmReset, kTaskMm, nullptr, 0, nullptr, 0, 6000))
    return false;
  if (!cmd(kMmVersion, kTaskMm, nullptr, 0, out, sizeof out))
    return false;
  _logger->info("aic8800: lmac version 0x{:08x}", get_le32(out));

  std::memset(p, 0, 112);
  p[3] = 0xff;
  put_le16(p + 100, 100);
  p[103] = 1;
  if (!cmd(kMeConfig, kTaskMe, p, 112, nullptr, 0))
    return false;

  std::memset(p, 0, 254);
  for (unsigned i = 0; i < 13; ++i) {
    put_le16(p + i * 6, static_cast<uint16_t>(2412 + 5 * i));
    p[i * 6 + 4] = 20;
  }
  p[252] = 13;
  unsigned n5 = 0;
  for (uint16_t ch : k5gChannels) {
    put_le16(p + 84 + n5 * 6, static_cast<uint16_t>(5000 + 5 * ch));
    p[84 + n5 * 6 + 2] = 1;
    p[84 + n5 * 6 + 4] = 20;
    ++n5;
  }
  p[253] = static_cast<uint8_t>(n5);
  if (!cmd(kMeChanConfig, kTaskMe, p, 254, nullptr, 0))
    return false;

  std::memset(p, 0, 16);
  p[0] = 1;
  p[2] = 1;
  if (!cmd(kMmSetCoex, kTaskMm, p, 16, nullptr, 0))
    return false;

  std::memset(p, 0, 72);
  put_le32(p + 64, 300);
  put_le16(p + 68, 20);
  return cmd(kMmStart, kTaskMm, p, 72, nullptr, 0, 6000);
}

bool Aic8800Radio::add_monitor() {
  uint8_t p[10] = {};
  uint8_t out[8] = {};
  p[0] = kMonitorIfType;
  std::memcpy(p + 2, kVifMac, 6);
  if (!cmd(kMmAddIf, kTaskMm, p, sizeof p, out, sizeof out))
    return false;
  if (out[0] != 0) {
    _logger->error("aic8800: MM_ADD_IF refused, status {}", out[0]);
    return false;
  }
  _vif = out[1];
  return true;
}

bool Aic8800Radio::tune(const SelectedChannel &channel) {
  if (channel.ChannelWidth != CHANNEL_WIDTH_20) {
    _logger->error("aic8800: only 20 MHz is implemented");
    return false;
  }
  const int freq = channel_freq(channel.Channel);
  if (freq == 0) {
    _logger->error("aic8800: channel {} is not in the firmware's channel list", channel.Channel);
    return false;
  }
  uint8_t p[14] = {};
  uint8_t out[16] = {};
  p[0] = freq >= 5000 ? 1 : 0;
  put_le16(p + 2, static_cast<uint16_t>(freq));
  put_le16(p + 4, static_cast<uint16_t>(freq));
  p[8] = 20;
  p[10] = 1;
  if (!cmd(kMeConfigMonitor, kTaskMe, p, sizeof p, out, sizeof out))
    return false;
  _channel = channel;
  return true;
}

void Aic8800Radio::InitWrite(SelectedChannel channel) {
  std::lock_guard<std::mutex> lock(_ctl_mu);
  if (_up) {
    if (!tune(channel))
      throw std::runtime_error("aic8800: tune failed");
    return;
  }
  if (!firmware_init() || !add_monitor() || !tune(channel))
    throw std::runtime_error("aic8800: firmware init failed");
  _up = true;
  start_drain();
  _logger->info("aic8800: monitor up on channel {}, vif {}", channel.Channel, _vif);
}

void Aic8800Radio::Init(Action_ParsedRadioPacket packetProcessor, SelectedChannel channel) {
  InitWrite(channel);
  StartRxLoop(std::move(packetProcessor));
}

void Aic8800Radio::start_drain() {
  _rx_stop.store(false);
  _drain = std::thread([this] { rx_pump(nullptr); });
}

void Aic8800Radio::stop_drain() {
  if (_drain.joinable()) {
    _rx_stop.store(true);
    _drain.join();
  }
}

void Aic8800Radio::StartRxLoop(Action_ParsedRadioPacket packetProcessor) {
  stop_drain();
  _rx_stop.store(false);
  rx_pump(&packetProcessor);
}

void Aic8800Radio::StopRxLoop() { _rx_stop.store(true); }

void Aic8800Radio::rx_pump(const Action_ParsedRadioPacket *processor) {
  std::vector<uint8_t> buf(kRxUrb);
  while (!_rx_stop.load(std::memory_order_relaxed) && !g_devourer_should_stop) {
    int n = 0;
    const int rc = libusb_bulk_transfer(_h, kEpDataIn, buf.data(), static_cast<int>(buf.size()),
                                        &n, 100);
    if (rc != 0 && rc != LIBUSB_ERROR_TIMEOUT) {
      _logger->error("aic8800: RX stopped: {}", libusb_error_name(rc));
      return;
    }
    if (n == 0 || processor == nullptr)
      continue;
    if (_rx_dump_left > 0) {
      --_rx_dump_left;
      _logger->info("aic8800: rx urb n={} {}", n, hex(buf.data(), std::min(n, 96)));
    }
    walk_rx(
        buf.data(), static_cast<size_t>(n),
        [&](const uint8_t *rec, size_t total) { deliver(rec, total, *processor); },
        [&](uint8_t type, const uint8_t *rec, size_t len) {
          if (type == kTypePrint && len > 4)
            _logger->debug("aic8800 fw: {}",
                           std::string(reinterpret_cast<const char *>(rec) + 4, len - 4));
        });
  }
}

void Aic8800Radio::deliver(const uint8_t *record, size_t total,
                           const Action_ParsedRadioPacket &processor) {
  const RxHdr h = decode_rx_hdr(record);
  Packet packet{};
  rx_pkt_attrib &a = packet.RxAtrib;
  a.pkt_len = h.len;
  a.crc_err = h.fcs_err || h.phy_err;
  a.tsfl = h.tsf_lo;
  a.bw = h.bw;
  a.sgi = h.sgi;
  a.stbc = h.stbc;
  a.ldpc = h.ldpc;
  a.paggr = h.ampdu;
  a.physt = true;
  a.fcs_present = kRxFcsPresent;
  a.pkt_rpt_type = RX_PACKET_TYPE::NORMAL_RX;
  const int raw = static_cast<int>(h.rssi) + kRssiBiasDb;
  a.rssi[0] = static_cast<uint8_t>(raw < 0 ? 0 : raw);
  if (h.format == 2 || h.format == 3) {
    a.data_rate = static_cast<uint16_t>(DESC_RATEMCS0 + h.mcs);
  } else if (h.format <= 1) {
    a.data_rate = legacy_rate_from_rx_code(h.leg_rate).value_or(DESC_RATE1M);
  } else {
    a.data_rate = DESC_RATE1M;
  }
  packet.Data = std::span<uint8_t>(const_cast<uint8_t *>(record) + kRxHdrLen, total - kRxHdrLen);
  if (packet.Data.size() >= 24) {
    const uint16_t seq = get_le16(packet.Data.data() + 22);
    a.seq_num = seq >> 4;
    a.frag_num = seq & 0xf;
    a.qos = packet.Data[0] == 0x88;
  }
  processor(packet);
}

void Aic8800Radio::SetMonitorChannel(SelectedChannel channel) {
  std::lock_guard<std::mutex> lock(_ctl_mu);
  if (!tune(channel))
    _logger->error("aic8800: retune to channel {} failed", channel.Channel);
}

bool Aic8800Radio::send_packet(const uint8_t *packet, size_t length) {
  TxRate dflt;
  {
    std::lock_guard<std::mutex> lock(_rate_mu);
    dflt = _default_rate;
  }
  const TxParsed parsed = parse_tx_radiotap(packet, length, dflt);
  if (parsed.status != TxParse::Ok) {
    if (!_warned_rate.exchange(true))
      _logger->error("aic8800: refusing a frame: {}",
                     parsed.status == TxParse::Malformed ? "malformed radiotap"
                                                         : "rate not implemented");
    _tx_fail.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  uint8_t buf[kMaxTxTotal + 1];
  const size_t n = build_tx(buf, sizeof buf, _vif, parsed.rate, packet + parsed.radiotap_len,
                            length - parsed.radiotap_len);
  if (n == 0) {
    _tx_fail.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  int done = 0;
  const int rc = libusb_bulk_transfer(_h, kEpDataOut, buf, static_cast<int>(n), &done, 1000);
  if (rc != 0) {
    _tx_fail.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  _tx_ok.fetch_add(1, std::memory_order_relaxed);
  return true;
}

SelectedChannel Aic8800Radio::GetSelectedChannel() {
  std::lock_guard<std::mutex> lock(_ctl_mu);
  return _channel;
}

void Aic8800Radio::SetCcaMode(bool disabled) {
  /* No firmware message for it is known. Carrier sense stays on, which is the
   * state a false request asks for. */
  if (disabled)
    throw std::runtime_error("aic8800: disabling carrier sense is not implemented");
}

void Aic8800Radio::Stop() {
  _rx_stop.store(true);
  stop_drain();
}

devourer::AdapterCaps Aic8800Radio::GetAdapterCaps() {
  devourer::AdapterCaps c;
  c.supported = true;
  c.chip_name = "AIC8800D80";
  c.marketing_names = "AIC8800D80";
  c.generation = devourer::ChipGeneration::Aic8800;
  c.transport = "usb";
  c.tx_chains = 1;
  c.rx_chains = 1;
  c.bw_mask = devourer::kBw20;
  devourer::set_standard_freq_ranges(c);
  return c;
}

devourer::TxStats Aic8800Radio::GetTxStats() {
  devourer::TxStats s{};
  s.submitted = _tx_ok.load(std::memory_order_relaxed);
  s.failed = _tx_fail.load(std::memory_order_relaxed);
  return s;
}

bool Aic8800Radio::GetPermanentMacAddress(uint8_t out[6]) {
  std::lock_guard<std::mutex> lock(_ctl_mu);
  if (!_up)
    return false;
  std::memcpy(out, _mac, 6);
  return true;
}

void Aic8800Radio::SetTxMode(const devourer::TxMode &mode) {
  TxRate r;
  if (mode.mode == devourer::TxMode::Mode::HT && mode.ht_mcs <= 7) {
    r.fmt = TxFmt::HtMixed;
    r.rate = mode.ht_mcs;
  } else if (mode.mode == devourer::TxMode::Mode::Legacy) {
    r.fmt = TxFmt::NonHt;
    r.rate = legacy_index(mode.legacy_rate_500kbps).value_or(4);
  } else {
    _logger->error("aic8800: TX mode not implemented, keeping the previous default");
    return;
  }
  std::lock_guard<std::mutex> lock(_rate_mu);
  _default_rate = r;
}

void Aic8800Radio::ClearTxMode() {
  std::lock_guard<std::mutex> lock(_rate_mu);
  _default_rate = TxRate{};
}
