/* AIC8800D80 bring-up harness, through the production IRadio path.
 *   aic8800probe <fw-dir> <channel> rx <seconds> [hex-urbs]
 *   aic8800probe <fw-dir> <channel> tx <frames> <rate> [gap-us] [frame-len]
 * <rate> is a DEVOURER_TX_RATE spec such as MCS3 or 24M. Frames are QoS-Data
 * with the canonical test SA, which rxdemo counts as rx.txhit. */
#include <libusb.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "RadiotapBuilder.h"
#include "RxPacket.h"
#include "UsbOpen.h"
#include "WiFiDriver.h"
#include "aic8800/Aic8800Fw.h"
#include "aic8800/Aic8800Radio.h"
#include "logger.h"

namespace {

uint32_t crc32_ieee(const uint8_t *p, size_t n) {
  uint32_t c = 0xffffffffu;
  for (size_t i = 0; i < n; ++i) {
    c ^= p[i];
    for (int b = 0; b < 8; ++b)
      c = (c >> 1) ^ (0xedb88320u & (0u - (c & 1u)));
  }
  return ~c;
}

int run_rx(Aic8800Radio &radio, int secs, int dump) {
  radio.SetRxDump(dump);
  std::atomic<uint64_t> frames{0}, crc_bad{0}, fcs_tail{0};
  std::atomic<int> shown{0};
  std::atomic<uint32_t> own_by_rate[64] = {};
  std::atomic<uint64_t> own{0};
  static const uint8_t test_sa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};
  std::thread stopper([&] {
    std::this_thread::sleep_for(std::chrono::seconds(secs));
    radio.StopRxLoop();
  });
  radio.StartRxLoop([&](const Packet &pkt) {
    frames.fetch_add(1);
    if (pkt.Data.size() >= 16 && std::memcmp(pkt.Data.data() + 10, test_sa, 6) == 0) {
      own.fetch_add(1);
      own_by_rate[pkt.RxAtrib.data_rate & 63].fetch_add(1);
    }
    if (pkt.RxAtrib.crc_err)
      crc_bad.fetch_add(1);
    bool tail = false;
    if (!pkt.RxAtrib.crc_err && pkt.Data.size() > 4) {
      const size_t n = pkt.Data.size() - 4;
      const uint8_t *d = pkt.Data.data();
      const uint32_t want = d[n] | (d[n + 1] << 8) | (d[n + 2] << 16) |
                            (static_cast<uint32_t>(d[n + 3]) << 24);
      tail = crc32_ieee(d, n) == want;
      if (tail)
        fcs_tail.fetch_add(1);
    }
    if (shown.fetch_add(1) < 12 && pkt.Data.size() >= 16)
      std::printf("rx len=%zu rate=%u rssi_raw=%u crc_err=%d fc=%02x sa=%02x:%02x:%02x:%02x:%02x:%02x "
                  "fcs_in_tail=%d\n",
                  pkt.Data.size(), pkt.RxAtrib.data_rate, pkt.RxAtrib.rssi[0],
                  pkt.RxAtrib.crc_err, pkt.Data[0], pkt.Data[10], pkt.Data[11], pkt.Data[12],
                  pkt.Data[13], pkt.Data[14], pkt.Data[15], tail);
  });
  stopper.join();
  for (int r = 0; r < 64; ++r)
    if (own_by_rate[r].load() != 0)
      std::printf("own rate=%d count=%u\n", r, own_by_rate[r].load());
  std::printf("state=rx_done own=%llu frames=%llu crc_bad=%llu fcs_in_tail=%llu\n",
              static_cast<unsigned long long>(own.load()),
              static_cast<unsigned long long>(frames.load()),
              static_cast<unsigned long long>(crc_bad.load()),
              static_cast<unsigned long long>(fcs_tail.load()));
  return 0;
}

int run_tx(Aic8800Radio &radio, int count, const std::string &rate, int gap_us, int len) {
  const auto rt = devourer::build_stream_radiotap(devourer::parse_tx_mode_str(rate));
  std::vector<uint8_t> pkt(rt.size() + static_cast<size_t>(len), 0);
  std::memcpy(pkt.data(), rt.data(), rt.size());
  uint8_t *f = pkt.data() + rt.size();
  static const uint8_t sa[6] = {0x57, 0x42, 0x75, 0x05, 0xd6, 0x00};
  f[0] = 0x88;
  std::memset(f + 4, 0xff, 6);
  std::memcpy(f + 10, sa, 6);
  std::memcpy(f + 16, sa, 6);
  for (int i = 26; i < len; ++i)
    f[i] = static_cast<uint8_t>(i);
  int ok = 0, bad = 0;
  const auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < count; ++i) {
    f[26] = static_cast<uint8_t>(i);
    f[27] = static_cast<uint8_t>(i >> 8);
    (radio.send_packet(pkt.data(), pkt.size()) ? ok : bad)++;
    if (gap_us > 0)
      usleep(static_cast<useconds_t>(gap_us));
  }
  const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  std::printf("state=tx_done sent=%d failed=%d secs=%.2f pps=%.0f\n", ok, bad, s, ok / s);
  return bad == 0 ? 0 : 7;
}

} /* namespace */

int main(int argc, char **argv) {
  if (argc < 5) {
    std::fprintf(stderr, "usage: see the header of aic8800probe.cpp\n");
    return 64;
  }
  devourer::aic8800::FwConfig fw;
  fw.dir = argv[1];
  const auto channel = static_cast<uint8_t>(std::atoi(argv[2]));
  const std::string cmd = argv[3];

  auto logger = std::make_shared<Logger>();
  libusb_context *ctx = nullptr;
  if (libusb_init(&ctx) < 0)
    return 2;
  libusb_device_handle *h = nullptr;
  std::shared_ptr<devourer::UsbDeviceLock> lock;
  int rc = devourer::aic8800::open_radio(ctx, fw, logger, &h, lock);
  if (rc != 0) {
    std::printf("state=open_failed rc=%s\n", libusb_error_name(rc));
    libusb_exit(ctx);
    return 3;
  }
  int code = 1;
  {
    WiFiDriver driver(logger);
    auto radio = driver.CreateRadio(h, ctx, lock, {});
    auto *aic = dynamic_cast<Aic8800Radio *>(radio.get());
    if (aic == nullptr) {
      std::printf("state=create_failed\n");
    } else {
      try {
        const auto t0 = std::chrono::steady_clock::now();
        aic->InitWrite(SelectedChannel{channel, 0, CHANNEL_WIDTH_20});
        std::printf("state=init_ok ms=%lld\n",
                    static_cast<long long>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                               std::chrono::steady_clock::now() - t0)
                                               .count()));
        std::fflush(stdout);
        if (cmd == "rx") {
          code = run_rx(*aic, std::atoi(argv[4]), argc > 5 ? std::atoi(argv[5]) : 0);
        } else if (cmd == "tx" && argc >= 6) {
          code = run_tx(*aic, std::atoi(argv[4]), argv[5], argc > 6 ? std::atoi(argv[6]) : 2000,
                        argc > 7 ? std::atoi(argv[7]) : 200);
        } else {
          code = 64;
        }
      } catch (const std::exception &e) {
        std::printf("state=init_failed what=%s\n", e.what());
        code = 5;
      }
    }
  }
  libusb_release_interface(h, devourer::aic8800::kIfaceFmac);
  libusb_close(h);
  libusb_exit(ctx);
  return code;
}
