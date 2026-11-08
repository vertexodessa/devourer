# AIC8800D80: raw 802.11 injection from userspace

Status: implemented in devourer as `src/aic8800/` (`DEVOURER_AIC8800=ON`, default OFF), TX and RX, 20 MHz, HT MCS 0-7
and non-HT only. The spike measurements below date from 2026-11-08. This document is the protocol reference and the
record of what was measured, written so the chip can be driven from another project without re-deriving any of it. It covers the USB states, the message and TX formats, the firmware load and init
sequences, the per-frame rate encoding, recovery, the on-air results and what is still unknown.

The result in one paragraph: the stock AIC firmware accepts raw 802.11 frames with a rate chosen per frame, on 2.4 and
5 GHz, from a static binary using only usbfs. No kernel driver and no kernel Wi-Fi stack are needed (the test camera's
kernel has no `CONFIG_WIRELESS`). The firmware lives in chip RAM and is gone after a power cycle; no flash is written.

## Provenance

- Wire formats and sequences were read out of [openipc/aic8800](https://github.com/openipc/aic8800) commit `0e218ed`
  (`aic_load_fw/`, `aic8800_fdrv/`).
- The chip firmware is a set of binary blobs from `fw/aic8800D80/` in that repository. They are not part of this
  repository: pass their directory in `aic8800::FwConfig::dir`.
- The spike sources are not part of this repository: `aic_common.h`, `aic_probe.c`, `aic_load.c`, `aic_inject.c`, plus test scripts. Everything needed to
  reimplement them is written down below.
- `src/aic8800/` was written against the facts in this document, plus the RX record layout below, which was read out of
  the same driver and checked against captured transfers. No driver code was copied.

## Test hardware

| Item | Value |
|---|---|
| Host | HiEasy camera, Hi3516CV610, OpenIPC 2.6.11.07, kernel without `CONFIG_WIRELESS` |
| Chip register `0x40500000` | `0x73c78820`, chip_id `0xc7` (`(reg >> 16) & 0xff`), U03, H variant |
| Firmware version | `0x06090101` |
| Receivers | mt7921u in monitor mode with tcpdump; RTL8812AU under devourer `rxdemo` |
| Build | `zig cc -target arm-linux-musleabihf -mcpu=cortex_a7 -static -Os` |

## USB states

| State | VID:PID | Wi-Fi interface | Bulk endpoints |
|---|---|---|---|
| Bootrom | `a69c:8d80` | 0 | `0x01` OUT, `0x82` IN, maxpacket 512. Messages and replies use this pair |
| Firmware running | `a69c:8d81` | 2 | `0x01` data OUT, `0x81` data IN, `0x02` message OUT, `0x82` message IN |

In the `8d81` state, interfaces 0 and 1 belong to Bluetooth (endpoints 04/83/84, iso 05/85). Starting the firmware makes
the chip drop off the bus and come back with the new PID, so the device must be reopened. On `8d81`, every command
confirmation arrived on `0x82`, and firmware prints arrived on `0x81`.

## Message format

All integers are little-endian. The same format is used in both USB states.

```
offset  size  field
0       1     (msg_len + 4) & 0xff
1       1     ((msg_len + 4) >> 8) & 0x0f
2       1     0x11 (config command)
3       5     zero
8       2     message id
10      2     destination task
12      2     source task, always DRV_TASK_ID = 100
14      2     param_len
16      n     params
```

`msg_len = 8 + param_len`. The bytes written to the endpoint are `msg_len + 8`.

Tasks: `TASK_MM = 0`, `TASK_DBG = 1`, `TASK_ME = 5`.

A received buffer is classified by `byte[2] & 0x7f`:

| Type | Meaning | Parsing |
|---|---|---|
| `0x11` | Command confirmation | id at +4, param_len at +10, params at +16 (a pattern u32 sits at +12) |
| `0x12` | TX confirmation | Never observed, see Open questions |
| `0x13` | Firmware print | u16 length at +0, text at +4 |

One URB can hold several messages. They are 4-byte aligned: the next one starts `((len + 3) & ~3) + 4` bytes later.

## Message ids

| Name | Request | Confirm | Params (bytes) |
|---|---|---|---|
| `DBG_MEM_READ` | `0x400` | `0x401` | 4: addr. Confirm: addr, value |
| `DBG_MEM_WRITE` | `0x402` | `0x403` | 8: addr, value |
| `DBG_MEM_BLOCK_WRITE` | `0x40B` | `0x40C` | always 1032: addr, size, data[1024] |
| `DBG_START_APP` | `0x40D` | none | 8: addr or delay, type |
| `MM_RESET` | `0x00` | `0x01` | 0 |
| `MM_START` | `0x02` | `0x03` | 72 |
| `MM_VERSION` | `0x04` | `0x05` | 0, confirm 28 |
| `MM_ADD_IF` | `0x06` | `0x07` | 10, confirm status, vif_index |
| `MM_REMAIN_ON_CHANNEL` | `0x46` | `0x47` | 20 |
| `MM_SET_COEX` | `0x65` | `0x66` | 16 |
| `MM_SET_RF_CALIB` | `0x69` | `0x6a` | 24, confirm 16 |
| `MM_GET_MAC_ADDR` | `0x73` | `0x74` | 4 |
| `MM_SET_TXPWR_IDX_LVL` | `0x77` | `0x78` | 95 |
| `MM_SET_TXPWR_OFST` | `0x79` | `0x7a` | not used |
| `MM_SET_STACK_START` | `0x7b` | `0x7c` | 4 |
| `ME_CONFIG` | `0x1400` | `0x1401` | 112 |
| `ME_CHAN_CONFIG` | `0x1402` | `0x1403` | 254 |
| `ME_CONFIG_MONITOR` | `0x1411` | `0x1412` | 14, confirm 12 echoes the channel |

Also seen in the driver and not used: `MM_CHANNEL_SWITCH_IND 0x44`, `MM_CHANNEL_PRE_SWITCH_IND 0x45`,
`MM_REMAIN_ON_CHANNEL_EXP_IND 0x48`, `DBG_GET_SYS_STAT 0x409`.

`DBG_*` go to `TASK_DBG`, `MM_*` to `TASK_MM`, `ME_*` to `TASK_ME`.

## Firmware load (state `8d80`)

Files for chip_id `0xc7`. Hashes are the first 16 hex digits of SHA-256, from commit `0e218ed`:

| File | SHA-256 prefix | Load address |
|---|---|---|
| `fw_patch_table_8800d80_u02.bin` | `fc1cb49136b840ec` | parsed on the host |
| `fw_adid_8800d80_u02.bin` | `a526cbd02fcdc495` | `addr_adid` from the table |
| `fw_patch_8800d80_u02.bin` | `8249567fdeb544e9` | `addr_patch` from the table |
| `fw_patch_8800d80_u02_ext0.bin` | `b463647ae1f4aaac` | ext entry from the table |
| `fmacfw_8800d80_h_u02.bin` | `33af31ac2e79e5bf` | `0x120000` |

Other chip revisions use other files. Only this one was tested.

Patch table layout: a 16-byte header starting with `AICBT_PT_TAG`, then entries of `name[16]`, `type u32`, `len u32`,
then `len * 8` bytes of data. An entry with `type >= 1000` or `len == 0` is a bare 24-byte header: no data is consumed.

The first entry must be type 0 (info). Its data is read as consecutive u32 fields: `adid_addrinf`, `addr_adid` (+4),
`patch_addrinf`, `addr_patch` (+12), `reset_addr`, `reset_val`, `adid_flag_addr`, `adid_flag`, `ext_patch_nb_addr`,
`ext_patch_nb` (+36), then `ext_patch_nb` pairs of (id, addr) from +40. Observed: `addr_adid = 0x00201940`,
`addr_patch = 0x001e0000`, one ext at `0x0020b43c`, 7 entries, 116 writes.

Sequence:

1. Read the chip register with `DBG_MEM_READ` to confirm the chip answers.
2. Upload adid, patch, and each `fw_patch_8800d80_u02_ext<id>.bin` at their table addresses.
3. Walk every entry in file order and `DBG_MEM_WRITE` each (addr, value) pair of its data. The info entry is included.
   - Type 3 (BT mode): overwrite these u32 words of the data before writing them: `[1]=1`, `[3]=0xFFFFFFFF`, `[5]=0`,
     `[7]=5`, `[9]=1`, `[11]=1500000`, `[13]=1`, `[15]=0`, `[17]=0x6F2F`.
   - Type 4 (power on): wait 500 us after the writes.
   - Type 6 (version string): do not write.
4. Upload `fmacfw_8800d80_h_u02.bin` at `0x120000`.
5. Apply the patch config, below.
6. Send `DBG_START_APP(0x120000, 1)` (`HOST_START_APP_AUTO`). There is no confirmation. The chip re-enumerates as `8d81`;
   the spike polled for it for up to 5 to 12 seconds and did not measure the delay.

Upload is in 1024-byte `DBG_MEM_BLOCK_WRITE` messages, each confirmed. `param_len` stays 1032 even for a short last
block, with the true length in `size`.

### Patch config, which also enables 5 GHz

```
config_base = read(0x120000 + 0x198)
str_base    = read(0x120000 + 0x198 + 8)
version     = read(0x120000 + 0x1C)
start_addr  = version > 0x06090100 ? read(0x120000 + 0x198 + 12) : 0x001D7000

write(str_base + 0x00, 0x48435450)
write(str_base + 0x04, start_addr)
write(str_base + 0x08, 0x50544348)
write(str_base + 0x0C, 3)

write(start_addr + 0x00, config_base + 0x00b4)
write(start_addr + 0x04, use_5g ? 0xf3010001 : 0xf3010000)
write(start_addr + 0x08, config_base + 0x0170)
write(start_addr + 0x0C, 0x0001000A)
write(start_addr + 0x10, config_base + 0x0188)
write(start_addr + 0x14, 1)

write(str_base + 0x30 .. 0x3C, 0)   four u32 words
```

## Init sequence (state `8d81`, interface 2)

Send on `0x02`, wait for the confirm on `0x82`. Params not listed are zero. Offsets are bytes.

| Step | Message | Params |
|---|---|---|
| 1 | `MM_SET_STACK_START` (4) | `[0]=1`, `[2]=0x20`, `[3]=fwtrace_redir` (1 sends firmware prints to USB). Confirm: `is_5g_support`, `vendor_info` |
| 2 | `MM_SET_TXPWR_IDX_LVL` (95), optional | first 69 bytes are `txpwr_lvl_conf_v3_t` |
| 3 | `MM_SET_RF_CALIB` (24) | u32 `@0=0x0f8f`, `@4=0x0f0f`, `@8=0x0c34c008`, `@12=0`, `@16=0x264203`, xtal cap `@20`, fine `@21` |
| 4 | `MM_GET_MAC_ADDR` (4) | `[0]=1`. Confirm: efuse MAC |
| 5 | `MM_RESET` | none |
| 6 | `MM_VERSION` | none |
| 7 | `ME_CONFIG` (112) | `[3]=0xff` (MCS 0-7), u16 `tx_lft @100=100`, `phy_bw_max @102=0`, `ht_supp @103=1` |
| 8 | `ME_CHAN_CONFIG` (254) | see below |
| 9 | `MM_SET_COEX` (16) | `[0]=1`, `[2]=1` |
| 10 | `MM_START` (72) | u32 `uapsd_timeout @64=300`, u16 `lp_clk_accuracy @68=20` |
| 11 | `MM_ADD_IF` (10) | `type @0=4` (monitor), MAC `@2`, `p2p @8=0`. Confirm: status (0 = ok), vif_index |
| 12 | `ME_CONFIG_MONITOR` (14) | see below |

`ME_CHAN_CONFIG`: 2.4 GHz entries start at 0 (max 14), 5 GHz entries at 84 (max 28), each 6 bytes:
`{u16 freq, u8 band @2, u8 flags @3, s8 tx_power @4}`, with count bytes at 252 (2.4 GHz) and 253 (5 GHz). The spike
listed 2412..2472 and 25 channels from 36 to 165 with `tx_power = 20`.

`ME_CONFIG_MONITOR`: `band @0` (0 = 2.4 GHz, 1 = 5 GHz), `type @1` (0 = 20 MHz), u16 `prim20 @2`, u16 `center1 @4`,
u16 `center2 @6`, `tx_power @8 = 20`, `chan_set @10 = 1`, `uf @11`, `auto_reply @12`.

**Retuning:** re-sending `ME_CONFIG_MONITOR` on running firmware moves the radio. It was verified across 25 channels in
one firmware load; the spike waited 200 ms after each retune and did not measure how much of that is needed.

**Repeating the init on running firmware is unreliable.** Reboot and reload the firmware for each fresh start.

## TX frame format (endpoint `0x01`)

```
offset  size  field
0       1     total & 0xff            total = 4 + 28 + frame_len
1       1     (total >> 8) & 0x0f
2       1     0x01 (data)
3       1     0
4       28    txdesc
32      n     802.11 frame, no FCS
```

txdesc, with offsets from its own start:

| Offset | Size | Field | Value |
|---|---|---|---|
| 0 | 2 | `packet_len` | frame length |
| 2 | 2 | `flags_ext` | 0 |
| 4 | 4 | `status_desc_addr` | rate word, below |
| 8 | 6 | `eth_dest` | 0 |
| 14 | 6 | `eth_src` | 0 |
| 20 | 2 | `ethertype` | 0 |
| 22 | 1 | `ac` | 3 (VO) |
| 23 | 1 | `tid` | `0xFF` |
| 24 | 1 | `vif_idx` | from `MM_ADD_IF` |
| 25 | 1 | `staid` | `0xFF` |
| 26 | 2 | `flags` | `TXU_CNTRL_MGMT` (bit 3) |

Per-frame fixed rate:

```
status_desc_addr = (1 << 30) | (fmt << 11) | (bw << 7) | rate
```

| fmt | Mode | rate |
|---|---|---|
| 0 | non-HT | 0..3 = CCK 1, 2, 5.5, 11; 4..11 = OFDM 6..54 |
| 2 | HT mixed | MCS index |
| 4 | VHT | not tested |
| 5 | HE | not tested |

Bit 9 is short GI and bit 10 is preamble; both come from the driver and were not exercised. `bw` was always 0 (20 MHz).
A receiver reported the requested rate for MCS 0, 3, 5, 7 and 6, 24, 54 Mb/s; the mt7921 confirmed 1, 2, 5.5, 11 Mb/s.

The firmware assigns the sequence numbers itself: received frames count from 1 regardless of the value in the frame.

The driver appends one byte when `total % 512 == 0`, to avoid a transfer ending exactly on a packet boundary. The
spike does not, and it never sent such a length. Implement the rule.

The 8-byte aggregated header found in parts of the driver is wrong for this build (`CONFIG_USB_TX_AGGR=n`). With it,
the firmware logs `wdt err E8` and `wdl err ... 232 236` per frame. Those messages are the signature of a bad USB length
or type.

Bit 31 of `status_desc_addr` requests a TX confirmation. The driver uses bit 31 or bit 30, not both, so a confirmed
frame goes out at the firmware's own rate. It also never produced a confirmation here. Do not build on it.

## RX record format (endpoint `0x81`)

One transfer can hold several records. A record whose type byte (`byte[2]`) lacks bit `0x10` is a received frame; the
next record starts at `(len + 60)` rounded up to 4. Otherwise it is a config message, as above. All offsets are from
the record start:

| Offset | Size | Field |
|---|---|---|
| 0 | 2 | `len`, the 802.11 frame length |
| 2 | 1 | type |
| 3 | 1 | bits 6-7 A-MPDU count |
| 4 | 8 | TSF |
| 12 | 1 | bits 0-3 format (0/1 non-HT, 2/3 HT, 4 VHT, 5+ HE), bits 4-6 bandwidth |
| 13 | 1 | antenna set |
| 14 | 1 | RSSI, s8 dBm |
| 15 | 2 | legacy length (12 bits), then legacy rate code in the top 4 bits of byte 16 |
| 17 | 1 | RSSI chain 1, s8 dBm; equal to byte 14 in the three dumped transfers |
| 18 | 1 | HT: bit 2 short GI, bit 4 STBC |
| 19 | 1 | HT: bits 0-6 MCS, bit 7 LDPC |
| 28 | 8 | RCPI and EVM per chain |
| 36 | 4 | status: bit 7 PHY error, bit 8 FCS error; byte 39 echoes frame-control byte 0 |
| 40 | 8 | band, channel type, primary and center frequency |
| 48 | 4 | flags, then VIF / STA / destination index |
| 60 | n | 802.11 frame, **no FCS** |

Legacy rate code to rate: 0..3 = CCK 1, 2, 5.5, 11; 8..15 = 48, 24, 12, 6, 54, 36, 18, 9 Mb/s.

Verified on the adapter on channel 36: `prim20` read 5180 and 16-byte RTS frames came through whole. The last 4 bytes
matched an FCS on 0 of 7428 frames. HT MCS decoding was checked with a known transmitter, an RTL8812AU sending MCS3,
which arrived as MCS3 in 3505 records. Only HT MCS3 was exercised, so MCS other than 3 and the LDPC and STBC bits are
read from the driver and untested. The delivery rate was not measured.

## Recovery

The firmware sometimes stops answering messages while USB stays up, so even `DBG_MEM_READ` times out. To recover, send
`DBG_START_APP(delay = 2000, type = 3)` (`HOST_START_APP_REBOOT`) on `0x02`; no confirmation comes. The chip returns to
`8d80` and the full load can be repeated. Wedges were seen after an action frame (`fc0 = 0xd0`) with confirm bit 31 set,
and once after a long run with firmware logging on. The reboot was used before every test run and worked each time in
the scripted runs; recovery from an actual wedge was observed once.

## Measurements

Short-range, indoors, one antenna arrangement, default firmware power. Treat these as a relative picture, not a link
budget: **absolute power, temperature and range were not measured.**

### Throughput

Channel 149, 1400-byte frames, no pacing, 1000 frames. The rate is what USB accepted, not proven airtime.

| Rate | Frames/s | Mbit/s | RTL8812 received | mt7921 received |
|---|---|---|---|---|
| MCS3 | 2073 | 23.2 | 977 | 1000 |
| MCS7 | 4569 | 51.2 | 458 | 803 |

At MCS7 the unpaced sender runs close to the PHY rate, so losses here may include receiver limits. No flow-control
mechanism was identified: the spike only relies on the bulk write completing.

### Channel map at 5 GHz

mt7921 receiver, one firmware load, retuning between channels, 100 frames of 200 bytes per rate, one pass. Received out
of 100. MCS3 and MCS4 received 98 to 100 on every channel, so they are omitted.

| Channel | MHz | MCS5 | MCS6 | MCS7 | 54M | dBm at MCS7 |
|---|---|---|---|---|---|---|
| 36 | 5180 | 98 | 100 | 100 | 99 | -62 |
| 40 | 5200 | 97 | 95 | 95 | 94 | -62 |
| 44 | 5220 | 81 | 88 | 66 | 59 | -62 |
| 48 | 5240 | 48 | 41 | 10 | 6 | -61 |
| 52 | 5260 | 71 | 65 | 56 | 40 | -64 |
| 56 | 5280 | 26 | 13 | 6 | 0 | -62 |
| 60 | 5300 | 3 | 4 | 0 | 0 | none |
| 64 | 5320 | 2 | 1 | 0 | 0 | none |
| 100 | 5500 | 100 | 94 | 89 | 73 | -59 |
| 104 | 5520 | 99 | 99 | 84 | 82 | -56 |
| 108 | 5540 | 94 | 98 | 93 | 91 | -58 |
| 112 | 5560 | 100 | 97 | 95 | 93 | -60 |
| 116 | 5580 | 99 | 97 | 91 | 96 | -59 |
| 120 | 5600 | 98 | 99 | 90 | 89 | -59 |
| 124 | 5620 | 95 | 98 | 87 | 85 | -59 |
| 128 | 5640 | 99 | 99 | 91 | 85 | -61 |
| 132 | 5660 | 90 | 90 | 44 | 53 | -61 |
| 136 | 5680 | 93 | 68 | 27 | 46 | -59 |
| 140 | 5700 | 89 | 71 | 22 | 38 | -58 |
| 144 | 5720 | 85 | 67 | 26 | 40 | -57 |
| 149 | 5745 | 95 | 90 | 72 | 74 | -58 |
| 153 | 5765 | 99 | 93 | 79 | 87 | -57 |
| 157 | 5785 | 99 | 97 | 89 | 93 | -54 |
| 161 | 5805 | 99 | 100 | 96 | 94 | -53 |
| 165 | 5825 | 99 | 100 | 98 | 99 | -54 |

Loss above MCS4 depends strongly on frequency while the signal level barely moves, which points at signal quality
rather than strength. The cause is unknown. The bad regions look as if they could line up with RF calibration groups,
but that is an unverified guess. It is one pass with one receiver, so check the channel you intend to use before
trusting a cell of this table.

Channels 52-64 and 100-144 are DFS channels. Transmission on them was one short indoor pass with no radar detection.
Regulatory limits on the channels and power are the integrator's responsibility.

### Power table

`MM_SET_TXPWR_IDX_LVL` with the values from the driver's `aic_userconfig_8800d80.txt`, A/B over four alternating runs
on channel 149, 200 frames per rate. The firmware confirmed the request, but received counts and signal did not change
(MCS7 on mt7921: 131 and 120 without, 116 and 120 with). Whether the table applies to fixed-rate frames is unknown.

### Receivers

The RTL8812AU under `rxdemo` shows no CCK at all, while the mt7921 received 96 to 99 of 100 at 1, 2, 5.5 and 11 Mb/s; the
cause on the receiver side was not investigated. On busy channels the RTL8812 lost more than the mt7921 (channel 36: 64
to 78 of 100 against 96 to 100). Do not use this RTL8812 setup alone to judge the AIC transmitter.

## Open questions

- Cause of the loss above MCS4 on some channels.
- TX status: bit 31 gave no `0x12` message in any run, so the sender has no per-frame delivery or queue feedback.
- Conditions that wedge the firmware.
- How long a retune needs before the radio is usable (the spike waited 200 ms; devourer returns at the confirm).
- RX throughput limits: devourer posts one synchronous 32 KB bulk read at a time, which has not been loaded.
- Whether `tx_power`, `MM_SET_TXPWR_OFST` or the power table control fixed-rate output.
- 40 MHz, VHT, HE, channels 169-177, long range, sustained thermal behaviour.
- A foreign source of Probe Requests with SA `57:42:75:05:d6:00` appeared in mt7921 captures during test runs (1961 on
  channel 36, 126 on 149). It continued without a gap across AIC reboots and was absent when the test was off. Its origin
  is unknown. The spike's counters exclude Probe Requests, so use a different test SA or filter them out.

## devourer backend

`aic8800::open_radio` reboots the chip if firmware is already running, loads the firmware, waits for `8d81` and claims
interface 2. Measured on the camera: 2.6 to 2.7 s from the reboot request to `8d80`, 0.66 s from `DBG_START_APP` to
`8d81`, 0.97 s for `InitWrite`. `WiFiDriver::CreateRadio` returns an `Aic8800Radio`, an `IRtlRadio` whose Realtek-only
members keep their not-ported defaults. `aic8800probe` drives it through the same path.

Differences from the Realtek generations: host-supplied firmware on every power-up, re-enumeration during bring-up,
channel control through firmware messages, and no TX status. `SetCcaMode(true)` throws because no message for it is
known. A frame asking for 40 MHz, VHT or MCS above 7 is refused rather than sent at another rate; LDPC and STBC flags
have no bit in the rate word, so those frames go out BCC.

mabur's transport over it, `linkbench-tx` on the camera to `linkbench-rx` on an RTL8812AU, one 15 s run on channel 149
at MCS3, 4 Mbit/s of application data, 50 % FEC overhead: 119897 of 119897 packets recovered, 4.09 % of frames lost at
the MAC, 283 frames with FCS errors. One run, one channel, short range.
