/* Headless guard for the AIC8800D80 wire formats (src/aic8800/Aic8800Proto.h).
 * The expected bytes come from docs/aic8800d80-userspace-injection.md. */
#include "RadiotapBuilder.h"
#include "aic8800/Aic8800Proto.h"

#include <cstdio>
#include <vector>

using namespace devourer::aic8800;

static int g_fail = 0;
#define CHECK(c)                                                                  \
  do {                                                                            \
    if (!(c)) {                                                                   \
      std::fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #c);                   \
      ++g_fail;                                                                   \
    }                                                                             \
  } while (0)

static void test_msg() {
  const uint8_t params[4] = {1, 2, 3, 4};
  const auto m = build_msg(0x400, kTaskDbg, params, 4);
  CHECK(m.size() == 20);
  CHECK(m[0] == 16 && m[1] == 0 && m[2] == 0x11);
  CHECK(get_le16(&m[8]) == 0x400 && get_le16(&m[10]) == 1);
  CHECK(get_le16(&m[12]) == 100 && get_le16(&m[14]) == 4);
  CHECK(m[16] == 1 && m[19] == 4);
}

static void test_rate_word() {
  TxRate ht;
  ht.fmt = TxFmt::HtMixed;
  ht.rate = 5;
  CHECK(ht.word() == ((1u << 30) | (2u << 11) | 5u));
  ht.sgi = true;
  CHECK(ht.word() == ((1u << 30) | (2u << 11) | (1u << 9) | 5u));
  CHECK(TxRate{}.word() == ((1u << 30) | 4u));
}

static void test_tx() {
  std::vector<uint8_t> out(4096);
  std::vector<uint8_t> frame(200, 0xaa);
  TxRate r;
  size_t n = build_tx(out.data(), out.size(), 7, r, frame.data(), frame.size());
  CHECK(n == 232);
  CHECK(out[0] == 232 && out[1] == 0 && out[2] == 1 && out[3] == 0);
  CHECK(get_le16(&out[4]) == 200);
  CHECK(out[26] == 3 && out[27] == 0xff && out[28] == 7 && out[29] == 0xff);
  CHECK(get_le16(&out[30]) == 8);
  CHECK(out[32] == 0xaa);

  /* total on a USB packet boundary gets one pad byte outside the length field */
  frame.assign(512 - 32, 0x55);
  n = build_tx(out.data(), out.size(), 0, r, frame.data(), frame.size());
  CHECK(n == 513);
  CHECK(get_le16(out.data()) == 512);

  frame.assign(kMaxTxTotal, 0);
  CHECK(build_tx(out.data(), out.size(), 0, r, frame.data(), frame.size()) == 0);
}

static void test_walk() {
  std::vector<uint8_t> b(256, 0);
  /* a 6-byte data record: 66 bytes, rounds to 68 */
  put_le16(b.data(), 6);
  b[2] = 0x00;
  b[12] = 0x02 | (0 << 4);
  b[16] = 0x90;
  b[17] = static_cast<uint8_t>(-58);
  b[18] = 0x04;
  b[19] = 0x85;
  put_le32(b.data() + 36, 1u << 8);
  /* then a confirm */
  put_le16(b.data() + 68, 20);
  b[70] = 0x11;
  int data = 0, cfg = 0;
  const size_t used = walk_rx(
      b.data(), 92,
      [&](const uint8_t *p, size_t total) {
        ++data;
        CHECK(total == 66);
        const RxHdr h = decode_rx_hdr(p);
        CHECK(h.len == 6 && h.format == 2 && h.rssi == -58);
        CHECK(h.sgi && h.mcs == 5 && h.ldpc && h.fcs_err && !h.phy_err);
      },
      [&](uint8_t type, const uint8_t *, size_t len) {
        ++cfg;
        CHECK(type == 0x11 && len == 24);
      });
  CHECK(data == 1 && cfg == 1 && used == 92);
  /* a record longer than the transfer stops the walk */
  put_le16(b.data(), 200);
  CHECK(walk_rx(b.data(), 92, [](const uint8_t *, size_t) {},
                [](uint8_t, const uint8_t *, size_t) {}) == 0);
}

static void test_radiotap() {
  devourer::TxMode m;
  m.mode = devourer::TxMode::Mode::HT;
  m.ht_mcs = 3;
  auto rt = devourer::build_stream_radiotap(m);
  rt.resize(rt.size() + 30, 0);
  TxParsed p = parse_tx_radiotap(rt.data(), rt.size(), TxRate{});
  CHECK(p.status == TxParse::Ok);
  CHECK(p.rate.fmt == TxFmt::HtMixed && p.rate.rate == 3);
  CHECK(p.radiotap_len == rt.size() - 30);

  m.ht_mcs = 8;
  rt = devourer::build_stream_radiotap(m);
  rt.resize(rt.size() + 30, 0);
  CHECK(parse_tx_radiotap(rt.data(), rt.size(), TxRate{}).status == TxParse::Unsupported);

  m = devourer::TxMode{};
  m.legacy_rate_500kbps = 108;
  rt = devourer::build_stream_radiotap(m);
  rt.resize(rt.size() + 30, 0);
  p = parse_tx_radiotap(rt.data(), rt.size(), TxRate{});
  CHECK(p.status == TxParse::Ok && p.rate.fmt == TxFmt::NonHt && p.rate.rate == 11);

  CHECK(parse_tx_radiotap(rt.data(), 3, TxRate{}).status == TxParse::Malformed);
  CHECK(legacy_rate_from_rx_code(11).value() == 4);
  CHECK(!legacy_rate_from_rx_code(5).has_value());
}

int main() {
  test_msg();
  test_rate_word();
  test_tx();
  test_walk();
  test_radiotap();
  if (g_fail == 0)
    std::puts("aic8800_proto: ok");
  return g_fail == 0 ? 0 : 1;
}
