# The pgpilot fork of kv4p-ht

This repository is a fork of [VanceVagell/kv4p-ht](https://github.com/VanceVagell/kv4p-ht),
maintained for [pgpilot](https://pgpilot.app), a paragliding app that uses a
kv4p HT as a VHF radio over Bluetooth. Its firmware branch is
`pgpilot/i2s-tx-guard` (it continues `pgpilot/reset-diagnostics`, which
shipped 19, and `pgpilot/ble-robust`, which shipped 18), and the firmware
calls itself **20 pgpilot** (`FIRMWARE_VER = 20`).

## What it changes

Firmware only (`microcontroller-src/`); the Android app is untouched. The
changes are aimed at upstream and are described in detail in
[microcontroller-src/docs/ble-robustness.md](microcontroller-src/docs/ble-robustness.md):

* Reports why the radio restarted (reset reason, a breadcrumb saying whether it
  was transmitting, and a core dump summary) at boot and on every connect,
  plus a power-cycle-proof boot counter and how the SA818 came up.
* Never freezes the main loop on an SA818 that does not answer: it is marked
  missing, probed again in the background (with power-down pulses) and
  configured once it answers. PTT and power-down are driven before Bluetooth
  starts.
* Keeps one damaged TX audio frame from silencing every later transmission:
  the ADPCM decoder realigns at every key-down and block, frames that are not
  whole blocks are dropped, and a `TX ADPCM desync:` line says so.
* Never crashes at key-down when the TX audio driver cannot get its DMA
  memory: the TX ring is mono (half the memory, the same as RX frees), every
  I2S call is guarded, and a failed install refuses the key with a
  `TX refused:` line, without keying the carrier, and retries a few times
  while PTT is held.
* Names itself on every transport: the boot report opens with
  `Radio id: kv4p-XXXXXX mac=... bt=...` (the advertised BLE name and the MACs
  it comes from), so a host can tell which radio is on the USB cable.
* Requests LE data length 251 and iOS-friendly BLE connection parameters.
* Buffers TX audio from the host and plays it out at the DAC's pace, so a
  congested Bluetooth link (for example with a headset microphone open) gives
  steady audio instead of dropouts. Advertised with HELLO feature bit 7.
* Cleaner start and end of each transmission: the first audio waits until the
  module has keyed, and the tail drains before unkey.
* Names the build `20 pgpilot` in the boot banner, the reset reason line and
  the ESP-IDF app descriptor.

The HELLO layout and every command are unchanged; only the version number
differs from upstream 17. The upstream Android app accepts it (it only rejects
older firmware).

## License and source

GPL-3.0, like upstream (see [LICENSE](LICENSE)). The exact source of each
binary pgpilot distributes is the tag it was built from, for example
[`v19-pgpilot.1`](https://github.com/simeneide/kv4p-ht/tree/v19-pgpilot.1).
To rebuild it:

```sh
git clone https://github.com/simeneide/kv4p-ht.git
cd kv4p-ht
git checkout v19-pgpilot.1
cd microcontroller-src
pio run -e esp32dev-release
```

The images are in `.pio/build/esp32dev-release/` (`bootloader.bin`,
`partitions.bin`, `firmware.bin`; `boot_app0.bin` comes from the
Arduino-ESP32 framework package).
