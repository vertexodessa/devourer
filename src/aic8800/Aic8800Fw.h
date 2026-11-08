#ifndef DEVOURER_AIC8800_FW_H
#define DEVOURER_AIC8800_FW_H

#include <memory>
#include <string>

#include <libusb.h>

#include "UsbDeviceLock.h"
#include "logger.h"

namespace devourer {
namespace aic8800 {

struct FwConfig {
  /* Holds the fw_*_8800d80_u02 files and fmacfw_8800d80_h_u02.bin, which are
   * not part of this repository. */
  std::string dir = "/lib/firmware/aic8800D80";
  bool use_5g = true;
};

/* Leaves the chip running freshly loaded firmware. Repeating the init on
 * firmware that is already running is unreliable, so a chip found in the
 * firmware state is rebooted to the bootrom first. The chip re-enumerates
 * during this, so no device handle can survive it: call it before opening.
 * Returns 0 or a negative libusb error. */
int prepare_firmware(libusb_context *ctx, const FwConfig &cfg, const Logger_t &logger);

/* prepare_firmware, then opens the firmware-state device and claims its Wi-Fi
 * interface, ready for WiFiDriver::CreateRadio. The caller closes `out`. */
int open_radio(libusb_context *ctx, const FwConfig &cfg, const Logger_t &logger,
               libusb_device_handle **out, std::shared_ptr<UsbDeviceLock> &lock);

} /* namespace aic8800 */
} /* namespace devourer */

#endif /* DEVOURER_AIC8800_FW_H */
