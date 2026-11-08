#ifndef DEVOURER_AIC8800_PROTO_H
#define DEVOURER_AIC8800_PROTO_H

/* The AIC8800D80 USB wire formats, free of libusb so a selftest can cover them.
 * docs/aic8800d80-userspace-injection.md is the reference for every offset. */

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <vector>

extern "C" {
#include "ieee80211_radiotap.h"
}

namespace devourer {
namespace aic8800 {

constexpr uint16_t kVid = 0xa69c;
constexpr uint16_t kPidBoot = 0x8d80;
constexpr uint16_t kPidFmac = 0x8d81;

inline bool is_usb_id(uint16_t vid, uint16_t pid) {
  return vid == kVid && (pid == kPidBoot || pid == kPidFmac);
}

/* Bootrom state: messages and replies share one bulk pair on interface 0. */
constexpr int kIfaceBoot = 0;
constexpr uint8_t kEpBootOut = 0x01;
constexpr uint8_t kEpBootIn = 0x82;
/* Firmware state: Wi-Fi is interface 2, the others belong to Bluetooth. */
constexpr int kIfaceFmac = 2;
constexpr uint8_t kEpDataOut = 0x01;
constexpr uint8_t kEpDataIn = 0x81;
constexpr uint8_t kEpMsgOut = 0x02;
constexpr uint8_t kEpMsgIn = 0x82;

constexpr uint16_t kTaskMm = 0;
constexpr uint16_t kTaskDbg = 1;
constexpr uint16_t kTaskMe = 5;
constexpr uint16_t kDrvTask = 100;

constexpr uint16_t kDbgMemRead = 0x400;
constexpr uint16_t kDbgMemWrite = 0x402;
constexpr uint16_t kDbgMemBlockWrite = 0x40b;
constexpr uint16_t kDbgStartApp = 0x40d;
constexpr uint32_t kStartAppAuto = 1;
constexpr uint32_t kStartAppReboot = 3;

constexpr uint16_t kMmReset = 0x00;
constexpr uint16_t kMmStart = 0x02;
constexpr uint16_t kMmVersion = 0x04;
constexpr uint16_t kMmAddIf = 0x06;
constexpr uint16_t kMmSetCoex = 0x65;
constexpr uint16_t kMmSetRfCalib = 0x69;
constexpr uint16_t kMmGetMacAddr = 0x73;
constexpr uint16_t kMmSetStackStart = 0x7b;
constexpr uint16_t kMeConfig = 0x1400;
constexpr uint16_t kMeChanConfig = 0x1402;
constexpr uint16_t kMeConfigMonitor = 0x1411;

/* Every request above is answered by the id one higher. */
constexpr uint16_t cfm_id(uint16_t req) { return static_cast<uint16_t>(req + 1); }

constexpr uint8_t kTypeTxData = 0x01;
constexpr uint8_t kTypeCfgFlag = 0x10;
constexpr uint8_t kTypeCmdRsp = 0x11;
constexpr uint8_t kTypeDataCfm = 0x12;
constexpr uint8_t kTypePrint = 0x13;

constexpr size_t kTxHdrLen = 4;
constexpr size_t kTxDescLen = 28;
constexpr size_t kRxHdrLen = 60;
/* The USB length field is 12 bits wide. */
constexpr size_t kMaxTxTotal = 0xfff;
constexpr size_t kUsbMaxPacket = 512;

inline void put_le16(uint8_t *p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v);
  p[1] = static_cast<uint8_t>(v >> 8);
}
inline void put_le32(uint8_t *p, uint32_t v) {
  put_le16(p, static_cast<uint16_t>(v));
  put_le16(p + 2, static_cast<uint16_t>(v >> 16));
}
inline uint16_t get_le16(const uint8_t *p) {
  return static_cast<uint16_t>(p[0] | (p[1] << 8));
}
inline uint32_t get_le32(const uint8_t *p) {
  return static_cast<uint32_t>(get_le16(p)) |
         (static_cast<uint32_t>(get_le16(p + 2)) << 16);
}

inline size_t round_up4(size_t n) { return (n + 3u) & ~static_cast<size_t>(3u); }

inline std::vector<uint8_t> build_msg(uint16_t id, uint16_t dest,
                                      const uint8_t *params, uint16_t param_len) {
  const size_t msg_len = 8u + param_len;
  std::vector<uint8_t> out(msg_len + 8u, 0);
  out[0] = static_cast<uint8_t>((msg_len + 4u) & 0xff);
  out[1] = static_cast<uint8_t>(((msg_len + 4u) >> 8) & 0x0f);
  out[2] = kTypeCmdRsp;
  put_le16(out.data() + 8, id);
  put_le16(out.data() + 10, dest);
  put_le16(out.data() + 12, kDrvTask);
  put_le16(out.data() + 14, param_len);
  if (param_len != 0)
    std::memcpy(out.data() + 16, params, param_len);
  return out;
}

enum class TxFmt : uint8_t { NonHt = 0, HtMixed = 2 };

struct TxRate {
  TxFmt fmt = TxFmt::NonHt;
  uint8_t rate = 4; /* 6 Mb/s: the rate a frame with no radiotap rate gets */
  bool sgi = false;

  uint32_t word() const {
    return (1u << 30) | (static_cast<uint32_t>(fmt) << 11) |
           (static_cast<uint32_t>(sgi) << 9) | rate;
  }
};

/* Radiotap legacy rate (500 kb/s units) -> the firmware's non-HT index. */
inline std::optional<uint8_t> legacy_index(uint8_t rate_500k) {
  static constexpr uint8_t kRates[12] = {2,  4,  11, 22, 12, 18,
                                         24, 36, 48, 72, 96, 108};
  for (uint8_t i = 0; i < 12; ++i)
    if (kRates[i] == rate_500k)
      return i;
  return std::nullopt;
}

enum class TxParse { Ok, Malformed, Unsupported };

struct TxParsed {
  TxParse status = TxParse::Malformed;
  uint16_t radiotap_len = 0;
  TxRate rate;
};

/* Reads the send_packet radiotap prefix. A rate the firmware word cannot carry
 * (40 MHz, VHT, MCS above 7) is Unsupported so the caller refuses the frame:
 * sending it at some other rate would break the link's airtime accounting
 * without any visible error. LDPC and STBC have no bit in the word; those
 * frames go out BCC, which every receiver decodes. */
inline TxParsed parse_tx_radiotap(const uint8_t *pkt, size_t len, const TxRate &dflt) {
  TxParsed out;
  out.rate = dflt;
  if (pkt == nullptr || len < sizeof(struct ieee80211_radiotap_header))
    return out;
  const uint16_t rlen = get_le16(pkt + 2);
  if (rlen == 0 || rlen >= len)
    return out;
  auto *hdr = reinterpret_cast<struct ieee80211_radiotap_header *>(
      const_cast<uint8_t *>(pkt));
  struct ieee80211_radiotap_iterator it;
  if (ieee80211_radiotap_iterator_init(&it, hdr, rlen, nullptr) != 0)
    return out;
  out.radiotap_len = rlen;
  bool have_mcs = false;
  std::optional<uint8_t> legacy;
  while (ieee80211_radiotap_iterator_next(&it) == 0) {
    switch (it.this_arg_index) {
    case IEEE80211_RADIOTAP_RATE:
      legacy = legacy_index(it.this_arg[0]);
      if (!legacy) {
        out.status = TxParse::Unsupported;
        return out;
      }
      break;
    case IEEE80211_RADIOTAP_MCS: {
      const uint8_t known = it.this_arg[0];
      const uint8_t flags = it.this_arg[1];
      const uint8_t mcs = it.this_arg[2];
      if (((known & IEEE80211_RADIOTAP_MCS_HAVE_BW) != 0 && (flags & 0x03) == 1) ||
          mcs > 7) {
        out.status = TxParse::Unsupported;
        return out;
      }
      have_mcs = true;
      out.rate.fmt = TxFmt::HtMixed;
      out.rate.rate = mcs;
      out.rate.sgi = (known & IEEE80211_RADIOTAP_MCS_HAVE_GI) != 0 && (flags & 0x04) != 0;
      break;
    }
    case IEEE80211_RADIOTAP_VHT:
      out.status = TxParse::Unsupported;
      return out;
    default:
      break;
    }
  }
  if (!have_mcs && legacy) {
    out.rate.fmt = TxFmt::NonHt;
    out.rate.rate = *legacy;
    out.rate.sgi = false;
  }
  out.status = TxParse::Ok;
  return out;
}

/* Fills `out` with one data-endpoint transfer and returns its USB length, or 0
 * when the frame does not fit the 12-bit length field. */
inline size_t build_tx(uint8_t *out, size_t cap, uint8_t vif, const TxRate &rate,
                       const uint8_t *frame, size_t frame_len) {
  const size_t total = kTxHdrLen + kTxDescLen + frame_len;
  if (total > kMaxTxTotal || total + 1 > cap)
    return 0;
  std::memset(out, 0, kTxHdrLen + kTxDescLen);
  out[0] = static_cast<uint8_t>(total & 0xff);
  out[1] = static_cast<uint8_t>((total >> 8) & 0x0f);
  out[2] = kTypeTxData;
  uint8_t *desc = out + kTxHdrLen;
  put_le16(desc, static_cast<uint16_t>(frame_len));
  put_le32(desc + 4, rate.word());
  desc[22] = 3;    /* AC_VO */
  desc[23] = 0xff; /* tid */
  desc[24] = vif;
  desc[25] = 0xff; /* staid */
  put_le16(desc + 26, 1u << 3); /* TXU_CNTRL_MGMT */
  std::memcpy(out + kTxHdrLen + kTxDescLen, frame, frame_len);
  /* A transfer ending exactly on a packet boundary needs a ZLP the firmware
   * does not expect. The pad byte is outside the header's length. */
  if (total % kUsbMaxPacket == 0) {
    out[total] = 0;
    return total + 1;
  }
  return total;
}

struct RxHdr {
  uint16_t len = 0;
  uint32_t tsf_lo = 0;
  uint8_t format = 0;
  uint8_t bw = 0;
  uint8_t leg_rate = 0;
  int8_t rssi = 0;
  uint8_t mcs = 0;
  bool sgi = false;
  bool stbc = false;
  bool ldpc = false;
  bool ampdu = false;
  bool phy_err = false;
  bool fcs_err = false;
};

/* `p` is a data record of at least kRxHdrLen bytes. */
inline RxHdr decode_rx_hdr(const uint8_t *p) {
  RxHdr h;
  h.len = get_le16(p);
  h.ampdu = (p[3] & 0xc0) != 0;
  h.tsf_lo = get_le32(p + 4);
  h.format = p[12] & 0x0f;
  h.bw = (p[12] >> 4) & 0x07;
  h.leg_rate = (p[16] >> 4) & 0x0f;
  h.rssi = static_cast<int8_t>(p[17]);
  h.sgi = (p[18] & 0x04) != 0;
  h.stbc = (p[18] & 0x10) != 0;
  h.mcs = p[19] & 0x7f;
  h.ldpc = (p[19] & 0x80) != 0;
  const uint32_t status = get_le32(p + 36);
  h.phy_err = (status & (1u << 7)) != 0;
  h.fcs_err = (status & (1u << 8)) != 0;
  return h;
}

/* The receive vector's legacy rate code -> the DESC_RATE index (and the
 * firmware's TX index), which number the rates the same way. Codes 4..7 are
 * not rates. */
inline std::optional<uint8_t> legacy_rate_from_rx_code(uint8_t code) {
  static constexpr int8_t kLut[16] = {0,  1,  2, 3, -1, -1, -1, -1,
                                      10, 8,  6, 4, 11, 9,  7,  5};
  const int8_t v = kLut[code & 0x0f];
  if (v < 0)
    return std::nullopt;
  return static_cast<uint8_t>(v);
}

/* One bulk-IN transfer can hold several records, each 4-byte aligned. Returns
 * the number of bytes consumed; less than `n` means a truncated tail. */
template <typename OnData, typename OnCfg>
size_t walk_rx(const uint8_t *b, size_t n, OnData &&on_data, OnCfg &&on_cfg) {
  size_t off = 0;
  while (off + 4 <= n) {
    const size_t len = get_le16(b + off);
    const uint8_t type = b[off + 2];
    if ((type & kTypeCfgFlag) != kTypeCfgFlag) {
      const size_t total = len + kRxHdrLen;
      if (len == 0 || off + total > n)
        break;
      on_data(b + off, total);
      off += round_up4(total);
    } else {
      if (off + 4 + len > n)
        break;
      on_cfg(static_cast<uint8_t>(type & 0x7f), b + off, 4 + len);
      off += round_up4(len) + 4;
    }
  }
  return off;
}

} /* namespace aic8800 */
} /* namespace devourer */

#endif /* DEVOURER_AIC8800_PROTO_H */
