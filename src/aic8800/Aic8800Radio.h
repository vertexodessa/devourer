#ifndef DEVOURER_AIC8800_RADIO_H
#define DEVOURER_AIC8800_RADIO_H

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

#include <libusb.h>

#include "DeviceConfig.h"
#include "IRtlRadio.h"
#include "RxPacket.h"
#include "UsbDeviceLock.h"
#include "aic8800/Aic8800Proto.h"
#include "logger.h"

/*
 * The AIC8800D80 in its firmware state (a69c:8d81), behind IRtlRadio so mabur's
 * radio slot accepts it; the Realtek-only members keep their "not available"
 * defaults. Firmware loading is aic8800::open_radio, because the chip
 * re-enumerates during it and no handle survives.
 *
 * Bulk-IN 0x81 is drained from InitWrite on, by a thread this class owns until
 * StartRxLoop takes over on the caller's thread. Command confirmations arrive
 * on 0x82 and are read only by the command caller under `_ctl_mu`.
 *
 * Only 20 MHz HT and non-HT are implemented. The firmware has no TX status and
 * no flow control beyond the bulk write completing.
 */
class Aic8800Radio : public IRtlRadio {
public:
  Aic8800Radio(libusb_device_handle *handle, libusb_context *ctx,
               std::shared_ptr<devourer::UsbDeviceLock> usb_lock, Logger_t logger,
               devourer::DeviceConfig cfg);
  ~Aic8800Radio() override;

  Aic8800Radio(const Aic8800Radio &) = delete;
  Aic8800Radio &operator=(const Aic8800Radio &) = delete;

  void Init(Action_ParsedRadioPacket packetProcessor, SelectedChannel channel) override;
  void InitWrite(SelectedChannel channel) override;
  void StartRxLoop(Action_ParsedRadioPacket packetProcessor) override;
  void StopRxLoop() override;
  void SetMonitorChannel(SelectedChannel channel) override;
  bool send_packet(const uint8_t *packet, size_t length) override;
  SelectedChannel GetSelectedChannel() override;
  void SetCcaMode(bool disabled) override;
  void Stop() override;
  devourer::AdapterCaps GetAdapterCaps() override;
  devourer::TxStats GetTxStats() override;
  bool GetPermanentMacAddress(uint8_t out[6]) override;
  void SetTxMode(const devourer::TxMode &mode) override;
  void ClearTxMode() override;

  /* Bring-up aid: log the first `urbs` bulk-IN transfers as hex. */
  void SetRxDump(int urbs) { _rx_dump_left = urbs; }

private:
  bool cmd(uint16_t id, uint16_t dest, const uint8_t *params, uint16_t len, uint8_t *out,
           size_t out_max, unsigned timeout_ms = 3000);
  bool firmware_init();
  bool add_monitor();
  bool tune(const SelectedChannel &channel);
  void rx_pump(const Action_ParsedRadioPacket *processor);
  void start_drain();
  void stop_drain();
  void deliver(const uint8_t *record, size_t total, const Action_ParsedRadioPacket &processor);

  libusb_device_handle *_h;
  libusb_context *_ctx;
  std::shared_ptr<devourer::UsbDeviceLock> _usb_lock;
  Logger_t _logger;
  devourer::DeviceConfig _cfg;

  std::mutex _ctl_mu;
  bool _up = false;
  uint8_t _vif = 0;
  uint8_t _mac[6] = {};
  SelectedChannel _channel{};

  std::mutex _rate_mu;
  devourer::aic8800::TxRate _default_rate;

  std::atomic<bool> _rx_stop{false};
  std::thread _drain;
  std::atomic<int> _rx_dump_left{0};

  std::atomic<uint64_t> _tx_ok{0};
  std::atomic<uint64_t> _tx_fail{0};
  std::atomic<bool> _warned_rate{false};
};

#endif /* DEVOURER_AIC8800_RADIO_H */
