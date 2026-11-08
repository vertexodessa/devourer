#include "aic8800/Aic8800Fw.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iterator>
#include <thread>
#include <vector>

#include "UsbOpen.h"
#include "aic8800/Aic8800Proto.h"

namespace devourer {
namespace aic8800 {
namespace {

constexpr uint32_t kChipIdReg = 0x40500000u;
constexpr uint8_t kChipIdD80 = 0xc7;
constexpr uint32_t kFmacAddr = 0x120000u;
constexpr size_t kBlockSize = 1024;
constexpr auto kReenumTimeout = std::chrono::seconds(12);

constexpr uint32_t kPtInfo = 0;
constexpr uint32_t kPtBtMode = 3;
constexpr uint32_t kPtPowerOn = 4;
constexpr uint32_t kPtVersion = 6;

class Boot {
public:
  Boot(libusb_device_handle *h, const Logger_t &log) : _h(h), _log(log) {}

  bool mem_read(uint32_t addr, uint32_t &value) {
    uint8_t p[4];
    put_le32(p, addr);
    uint8_t cfm[8];
    if (!transact(kDbgMemRead, p, sizeof p, cfm, sizeof cfm) || get_le32(cfm) != addr)
      return false;
    value = get_le32(cfm + 4);
    return true;
  }

  bool mem_write(uint32_t addr, uint32_t value) {
    uint8_t p[8];
    put_le32(p, addr);
    put_le32(p + 4, value);
    return transact(kDbgMemWrite, p, sizeof p, nullptr, 0);
  }

  bool upload(uint32_t addr, const std::vector<uint8_t> &data) {
    /* The firmware expects the full-size parameter block on every message,
     * the short last one included; the true length travels in `size`. */
    std::vector<uint8_t> p(8 + kBlockSize);
    for (size_t off = 0; off < data.size(); off += kBlockSize) {
      const size_t n = std::min(kBlockSize, data.size() - off);
      std::fill(p.begin(), p.end(), 0);
      put_le32(p.data(), addr + static_cast<uint32_t>(off));
      put_le32(p.data() + 4, static_cast<uint32_t>(n));
      std::memcpy(p.data() + 8, data.data() + off, n);
      if (!transact(kDbgMemBlockWrite, p.data(), static_cast<uint16_t>(p.size()), nullptr, 0)) {
        _log->error("aic8800: upload failed at 0x{:08x}", addr + off);
        return false;
      }
    }
    return true;
  }

  bool start_app(uint32_t addr) {
    uint8_t p[8];
    put_le32(p, addr);
    put_le32(p + 4, kStartAppAuto);
    auto msg = build_msg(kDbgStartApp, kTaskDbg, p, sizeof p);
    int done = 0;
    return libusb_bulk_transfer(_h, kEpBootOut, msg.data(), static_cast<int>(msg.size()),
                                &done, 2000) == 0;
  }

private:
  bool transact(uint16_t id, const uint8_t *p, uint16_t len, uint8_t *out, size_t out_len) {
    auto msg = build_msg(id, kTaskDbg, p, len);
    int done = 0;
    int rc = libusb_bulk_transfer(_h, kEpBootOut, msg.data(), static_cast<int>(msg.size()),
                                  &done, 2000);
    if (rc != 0) {
      _log->error("aic8800: boot message 0x{:04x} send failed: {}", id, libusb_error_name(rc));
      return false;
    }
    uint8_t rx[2048];
    for (int attempt = 0; attempt < 10; ++attempt) {
      int n = 0;
      rc = libusb_bulk_transfer(_h, kEpBootIn, rx, sizeof rx, &n, 1000);
      if (rc == LIBUSB_ERROR_TIMEOUT)
        continue;
      if (rc != 0)
        return false;
      if (n >= 16 && (rx[2] & 0x7f) == kTypeCmdRsp && get_le16(rx + 4) == cfm_id(id)) {
        if (out_len != 0) {
          if (static_cast<size_t>(n) < 16 + out_len)
            return false;
          std::memcpy(out, rx + 16, out_len);
        }
        return true;
      }
    }
    _log->error("aic8800: no confirm for boot message 0x{:04x}", id);
    return false;
  }

  libusb_device_handle *_h;
  const Logger_t &_log;
};

bool read_file(const std::string &path, std::vector<uint8_t> &out, const Logger_t &log) {
  std::ifstream f(path, std::ios::binary);
  if (!f) {
    log->error("aic8800: cannot read firmware file {}", path);
    return false;
  }
  out.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
  return !out.empty();
}

void fill_bt_mode(uint8_t *data) {
  static constexpr struct {
    unsigned word;
    uint32_t value;
  } kFields[] = {{1, 1}, {3, 0xffffffffu}, {5, 0},  {7, 5},     {9, 1},
                 {11, 1500000}, {13, 1},  {15, 0}, {17, 0x6f2f}};
  for (const auto &f : kFields)
    put_le32(data + f.word * 4, f.value);
}

bool load_patches(Boot &boot, const std::string &dir, const Logger_t &log) {
  std::vector<uint8_t> tbl;
  if (!read_file(dir + "/fw_patch_table_8800d80_u02.bin", tbl, log))
    return false;
  if (tbl.size() < 16 + 24 || std::memcmp(tbl.data(), "AICBT_PT_TAG", 12) != 0) {
    log->error("aic8800: bad patch table header");
    return false;
  }
  const uint8_t *first = tbl.data() + 16;
  const uint32_t info_len = get_le32(first + 20);
  /* The info entry is read as consecutive u32 fields up to ext_patch_nb at +36. */
  if (get_le32(first + 16) != kPtInfo || info_len < 5 ||
      16 + 24 + static_cast<size_t>(info_len) * 8 > tbl.size()) {
    log->error("aic8800: patch table does not start with an info entry");
    return false;
  }
  const uint8_t *info = first + 24;
  const uint32_t addr_adid = get_le32(info + 4);
  const uint32_t addr_patch = get_le32(info + 12);
  const uint32_t ext_nb = get_le32(info + 36);
  if (40 + static_cast<size_t>(ext_nb) * 8 > static_cast<size_t>(info_len) * 8) {
    log->error("aic8800: patch table lists {} extensions beyond its info entry", ext_nb);
    return false;
  }

  std::vector<uint8_t> blob;
  if (!read_file(dir + "/fw_adid_8800d80_u02.bin", blob, log) || !boot.upload(addr_adid, blob))
    return false;
  if (!read_file(dir + "/fw_patch_8800d80_u02.bin", blob, log) || !boot.upload(addr_patch, blob))
    return false;
  for (uint32_t i = 0; i < ext_nb; ++i) {
    const uint32_t id = get_le32(info + 40 + i * 8);
    const uint32_t addr = get_le32(info + 44 + i * 8);
    if (!read_file(dir + "/fw_patch_8800d80_u02_ext" + std::to_string(id) + ".bin", blob, log) ||
        !boot.upload(addr, blob))
      return false;
  }

  size_t off = 16;
  while (off + 24 <= tbl.size()) {
    const uint32_t type = get_le32(tbl.data() + off + 16);
    const uint32_t len = get_le32(tbl.data() + off + 20);
    off += 24;
    if (type >= 1000 || len == 0)
      continue;
    if (off + static_cast<size_t>(len) * 8 > tbl.size()) {
      log->error("aic8800: patch table entry type {} truncated", type);
      return false;
    }
    uint8_t *data = tbl.data() + off;
    if (type == kPtBtMode && len >= 9)
      fill_bt_mode(data);
    if (type != kPtVersion) {
      for (uint32_t i = 0; i < len; ++i)
        if (!boot.mem_write(get_le32(data + i * 8), get_le32(data + i * 8 + 4)))
          return false;
      if (type == kPtPowerOn)
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
    off += static_cast<size_t>(len) * 8;
  }
  return true;
}

/* Tells the firmware which features to enable; bit 0 of the 0xb4 value is 5 GHz. */
bool apply_patch_config(Boot &boot, bool use_5g) {
  const uint32_t table[3][2] = {{0x00b4, use_5g ? 0xf3010001u : 0xf3010000u},
                                {0x0170, 0x0001000au},
                                {0x0188, 0x00000001u}};
  uint32_t config_base = 0, str_base = 0, version = 0, start_addr = 0x001d7000u;
  if (!boot.mem_read(kFmacAddr + 0x198, config_base) ||
      !boot.mem_read(kFmacAddr + 0x198 + 8, str_base) ||
      !boot.mem_read(kFmacAddr + 0x1c, version))
    return false;
  if (version > 0x06090100u && !boot.mem_read(kFmacAddr + 0x198 + 12, start_addr))
    return false;
  if (!boot.mem_write(str_base + 0x00, 0x48435450u) ||
      !boot.mem_write(str_base + 0x04, start_addr) ||
      !boot.mem_write(str_base + 0x08, 0x50544348u) ||
      !boot.mem_write(str_base + 0x0c, 3))
    return false;
  for (uint32_t i = 0; i < 3; ++i) {
    if (!boot.mem_write(start_addr + 8 * i, config_base + table[i][0]) ||
        !boot.mem_write(start_addr + 8 * i + 4, table[i][1]))
      return false;
  }
  for (uint32_t i = 0; i < 4; ++i)
    if (!boot.mem_write(str_base + 0x30 + i * 4, 0))
      return false;
  return true;
}

libusb_device_handle *wait_for_pid(libusb_context *ctx, uint16_t pid, const Logger_t &log) {
  const auto t0 = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - t0 < kReenumTimeout) {
    if (auto *h = libusb_open_device_with_vid_pid(ctx, kVid, pid)) {
      log->info("aic8800: {:04x}:{:04x} present after {} ms", kVid, pid,
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - t0)
                    .count());
      return h;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  log->error("aic8800: {:04x}:{:04x} did not appear", kVid, pid);
  return nullptr;
}

/* Sends the chip back to the bootrom. No confirmation comes: it drops off the
 * bus 2 s later, which is the delay this message carries. */
int reboot_to_bootrom(libusb_device_handle *h) {
  libusb_set_auto_detach_kernel_driver(h, 1);
  int rc = libusb_claim_interface(h, kIfaceFmac);
  if (rc != 0)
    return rc;
  uint8_t p[8];
  put_le32(p, 2000);
  put_le32(p + 4, kStartAppReboot);
  auto msg = build_msg(kDbgStartApp, kTaskDbg, p, sizeof p);
  int done = 0;
  rc = libusb_bulk_transfer(h, kEpMsgOut, msg.data(), static_cast<int>(msg.size()), &done, 2000);
  libusb_release_interface(h, kIfaceFmac);
  return rc;
}

int load_from_bootrom(libusb_device_handle *h, const FwConfig &cfg, const Logger_t &log) {
  libusb_set_auto_detach_kernel_driver(h, 1);
  int rc = libusb_claim_interface(h, kIfaceBoot);
  if (rc != 0) {
    log->error("aic8800: claim of the bootrom interface failed: {}", libusb_error_name(rc));
    return rc;
  }
  Boot boot(h, log);
  rc = LIBUSB_ERROR_IO;
  uint32_t chip = 0;
  std::vector<uint8_t> fmac;
  if (!boot.mem_read(kChipIdReg, chip)) {
    log->error("aic8800: chip does not answer in the bootrom state");
  } else if (((chip >> 16) & 0xff) != kChipIdD80) {
    log->error("aic8800: chip register 0x{:08x} is not an 8800D80", chip);
    rc = LIBUSB_ERROR_NOT_SUPPORTED;
  } else if (load_patches(boot, cfg.dir, log) &&
             read_file(cfg.dir + "/fmacfw_8800d80_h_u02.bin", fmac, log) &&
             boot.upload(kFmacAddr, fmac) && apply_patch_config(boot, cfg.use_5g) &&
             boot.start_app(kFmacAddr)) {
    log->info("aic8800: firmware started, chip register 0x{:08x}", chip);
    rc = 0;
  }
  libusb_release_interface(h, kIfaceBoot);
  return rc;
}

} /* namespace */

int prepare_firmware(libusb_context *ctx, const FwConfig &cfg, const Logger_t &logger) {
  if (auto *h = libusb_open_device_with_vid_pid(ctx, kVid, kPidFmac)) {
    logger->info("aic8800: firmware already running, rebooting it for a clean start");
    const int rc = reboot_to_bootrom(h);
    libusb_close(h);
    if (rc != 0) {
      logger->error("aic8800: reboot request failed: {}", libusb_error_name(rc));
      return rc;
    }
  }
  libusb_device_handle *boot = wait_for_pid(ctx, kPidBoot, logger);
  if (boot == nullptr)
    return LIBUSB_ERROR_NO_DEVICE;
  const int rc = load_from_bootrom(boot, cfg, logger);
  libusb_close(boot);
  if (rc != 0)
    return rc;
  libusb_device_handle *fmac = wait_for_pid(ctx, kPidFmac, logger);
  if (fmac == nullptr)
    return LIBUSB_ERROR_NO_DEVICE;
  libusb_close(fmac);
  return 0;
}

int open_radio(libusb_context *ctx, const FwConfig &cfg, const Logger_t &logger,
               libusb_device_handle **out, std::shared_ptr<UsbDeviceLock> &lock) {
  *out = nullptr;
  int rc = prepare_firmware(ctx, cfg, logger);
  if (rc != 0)
    return rc;
  libusb_device_handle *h = libusb_open_device_with_vid_pid(ctx, kVid, kPidFmac);
  if (h == nullptr)
    return LIBUSB_ERROR_NO_DEVICE;
  /* No reset: a USB reset would drop the firmware that was just loaded. */
  rc = claim_interface_then_reset(h, kIfaceFmac, logger, /*do_reset=*/false, lock);
  if (rc != 0) {
    libusb_close(h);
    return rc;
  }
  *out = h;
  return 0;
}

} /* namespace aic8800 */
} /* namespace devourer */
