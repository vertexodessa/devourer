# AIC8800D80 backend

Protocol reference and measurements: `docs/aic8800d80-userspace-injection.md`.

- `Aic8800Proto.h` is pure (no libusb): every wire format lives there and `tests/aic8800_proto_selftest.cpp` covers it.
- `open_radio` replaces the usual open path because the chip re-enumerates while firmware loads. Never reset the
  device afterwards: a USB reset drops the firmware.
- Firmware that is already running is rebooted before every load. Repeating the init on it is unreliable.
- Bulk-IN `0x81` is drained from `InitWrite` on; confirms are read from `0x82` only by the caller holding `_ctl_mu`.
- Received frames carry no FCS (`fcs_present = false`).
- Firmware blobs are not in this repository.
