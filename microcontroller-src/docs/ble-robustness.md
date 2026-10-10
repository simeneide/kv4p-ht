# BLE robustness and crash diagnostics

This note covers firmware changes that make TX audio over Bluetooth LE survive
a congested link, and that make unexpected restarts diagnosable. None of them
change the wire format: HELLO, DeviceState and every command keep their
layout, and hosts that do not care about the new log lines can ignore them,
as they already ignore debug frames.

Background: two field problems prompted this.

1. **Choppy TX audio when the phone also has a Bluetooth headset microphone
   open (HFP/SCO).** The phone's controller serves the SCO link first and LE
   connection events get cut short or skipped. Audio then reaches the radio in
   bursts with gaps, and the old code played every frame the moment it arrived,
   so each gap became a dropout. Because the I2S DMA runs with `auto_clear`
   off, a gap longer than the 64 ms DMA ring also replayed stale audio.
2. **Radios restarting during TX at high power.** Captured UART logs (#472)
   show panics (`SW_CPU_RESET` after a Guru Meditation), not brownouts. One
   suspect is Bluedroid's ACL reassembly (`packet_fragmenter.c`) on IDF 4.4,
   which never got the later hardening (CVE-2020-0022 family, esp-idf#18414).

## 1. Reset reason and crash diagnostics

Files: `resetDiagnostics.h`, `kv4p_ht_esp32_wroom_32.ino`.

On every boot the firmware records:

* `esp_reset_reason()` (POWERON, SW, PANIC, INT_WDT, TASK_WDT, WDT, BROWNOUT, ...).
* A breadcrumb kept in RTC memory (`RTC_NOINIT_ATTR`) that survives software
  resets, panics, watchdogs and usually brownouts: boot counter, last uptime,
  whether the radio was keyed, whether high power was selected and how long
  the transmission had lasted. It is updated every 100 ms and on every
  key/unkey (including AX.25 packets), and protected by a check word. It does
  not survive a power-on, so it is not reported after one.
* A summary of the core dump in flash, if there is one (read and formatted
  once at boot, since checking it scans up to 64 KB of flash): crashing task, PC,
  exception cause, faulting address, app ELF SHA and up to 16 backtrace
  addresses. Arduino-ESP32 2.0.17 is built with
  `CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH=y` and `..._DATA_FORMAT_ELF=y`, and
  `min_spiffs.csv` has a 64 KB `coredump` partition at `0x3F0000`, so a panic
  writes a dump there. A dump stays until the next panic overwrites it, so it
  can be older than the current boot; the reset reason says whether it is fresh.

Where it is reported:

* As plain text on the USB serial port at boot, inside the existing banner and
  before any KISS framing, so any serial monitor shows it.
* As `COMMAND_DEBUG_INFO` text frames to each host session right after HELLO
  (USB, Classic and BLE), and also on USB when a Bluetooth session starts.
  This repeats on every connect, so count restarts by boot number, not by
  lines.

Example lines (values illustrative):

```
Reset reason: PANIC; prev keyed=1 high=1 keyed_for=6120ms up=734120ms; boot #4; Exception/Panic Reset
Core dump: 11364 bytes at 0x3f0000, task=BTC_TASK pc=0x400d6a1c exccause=29 excvaddr=0x00000002 elf=1a2b3c4d5e6f7a8b bt=0x400d6a1c,0x400d6b80,...
```

The first line keeps the wording "Reset reason" that hosts may already look
for (pgpilot matches it and keeps the first 120 characters, so the keyed state
comes first). The protocol is unchanged. Upstream issue #473
suggested a separate debug GATT characteristic for richer BLE diagnostics, and
that is still open; these lines only use the debug log channel that already
exists.

### Reading and decoding a core dump

The ELF of the exact build is needed, so archive `firmware.elf` next to every
`firmware.bin` you ship. The PC and backtrace in the summary line can be
resolved straight away:

```sh
~/.platformio/packages/toolchain-xtensa-esp32/bin/xtensa-esp32-elf-addr2line \
  -pfiaC -e .pio/build/esp32dev-release/firmware.elf 0x400d6a1c 0x400d6b80
```

For the full dump (all tasks, registers, stacks), read the partition over USB
before reflashing (flashing the app does not erase it, but a new panic would
overwrite it). esptool resets the board into its bootloader, so note the reset
reason line first:

```sh
PORT=/dev/cu.usbserial-XXXX
uvx esptool --chip esp32 --port $PORT read-flash 0x3F0000 0x10000 coredump.bin
uvx --from esp-coredump esp-coredump --chip esp32 info_corefile \
  --gdb ~/.platformio/packages/tool-xtensa-esp-elf-gdb/bin/xtensa-esp-elf-gdb-no-python \
  -t raw -c coredump.bin .pio/build/esp32dev-release/firmware.elf
# Optional, once saved: clear it so the next one is unambiguous.
uvx esptool --chip esp32 --port $PORT erase-region 0x3F0000 0x10000
```

`esp-coredump` may warn that the ELF SHA does not match if the ELF comes from a
rebuild rather than the original build; the addresses are still valid as long
as the same commit, toolchain and platform version were used.

Opening the serial port normally toggles DTR/RTS, which resets most ESP32
boards (an EN reset reports as POWERON and clears the RTC breadcrumb). Use
`tools/kv4p_log.py` or `pio device monitor --dtr 0 --rts 0` to avoid that.

## 2. LE Data Length Extension

File: `BluedroidBleKissGattStream.h`.

On connect the firmware calls `esp_ble_gap_set_pkt_data_len(peer, 251)`
(`Config::preferredTxOctets`, 0 disables). With the 247-byte MTU the host
uses, a full write is 244 B of data + 3 B ATT + 4 B L2CAP = 251 B, which then
travels as one link-layer PDU instead of about ten 27-byte fragments. That
roughly halves the per-event airtime overhead and, more importantly here,
means each incoming write reaches Bluedroid as a single, unfragmented HCI ACL
packet, so the reassembly path suspected in #472 is not exercised (see 5).

The negotiated values are logged from `ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT`
(`dle_tx`/`dle_rx`). Many phones start the data length procedure themselves;
if the values were already in place the controller may not report anything,
in which case the log says `(dle not reported)`.

## 3. Connection parameters

File: `BluedroidBleKissGattStream.h` (defaults in `Config`).

| Constant | Value | Why |
| --- | --- | --- |
| `connMinInterval` | 12 (15 ms) | Apple: min >= 15 ms and a multiple of 15 ms |
| `connMaxInterval` | 12 (15 ms) | Apple: max = min, or >= min + 15 ms |
| `connLatency` | 0 | latency 1 lets the radio skip every other event; downlink data then waits up to two intervals |
| `connTimeout` | 600 (6 s) | Apple: 6 s to 18 s; also rides through multi-second starvation |
| `maxConnParamRequests` | 3 per connection | never loop against a central that prefers other values |
| `connRequestSpacingMs` | 30 s | |

The previous request (7.5 to 15 ms, 4 s timeout) broke two of Apple's rules,
and iOS rejects such a request outright, so iOS links stayed on whatever iOS
chose. Android usually accepts either request and ends up at 15 ms.

The request is sent on connect, as before. When TX starts on a BLE session and
the current parameters are worse than requested (interval above 15 ms or
latency above 0), it is sent once more, at most three requests per connection
and never closer than 30 s apart. Every change of interval, latency, timeout,
MTU or data length is logged on the main loop (the Bluetooth task only records
the values):

```
BLE link: connected=1 interval=15.00 ms latency=0 timeout=6000 ms param_status=0 requests=1 mtu=247 dle_tx=251 dle_rx=251 dle_status=0
```

The phone has the last word: a request only helps if the central accepts it.
Shorter intervals give the controller more anchor points to fit around SCO
slots, but they cannot create airtime that SCO has taken.

## 4. TX jitter buffer

Files: `txJitterBuffer.h` and `txKeyPolicy.h` (pure logic, unit tested),
`txAudio.h`, `protocol.h`, `kv4p_ht_esp32_wroom_32.ino`.

`COMMAND_HOST_TX_AUDIO` frames are no longer decoded on arrival. They go into a
FIFO of encoded ADPCM (8 KB, about 1 s, 72 entries), and the main loop plays
them out at the DAC's pace. Frames may carry any number of 128-byte IMA ADPCM
blocks: the Android app sends one per frame (~15.6 ms), pgpilot sends four
(512 B, ~62 ms). Every duration below (pre-roll level, buffered and peak time,
played time) is computed from the audio the bytes actually hold, not from a
frame count.

* **Pre-roll.** Playout starts when `TX_PREROLL_WIRELESS_MS` (300 ms) of audio
  is buffered, or when the first buffered frame is 300 ms old (a host that
  cannot fill the target is never stalled), or when the host is nearly out of
  window credit, or when PTT is released. USB gets `TX_PREROLL_USB_MS` (40 ms)
  because it does not burst. Classic Bluetooth uses the 300 ms target too (its
  2048 B window no longer limits the pre-roll, since credit comes back on
  receipt; see Flow control). Until playout starts the DAC gets explicit
  silence.
* **Underrun.** If the buffer runs dry mid-transmission, the firmware writes
  silence (never the stale DMA contents) and goes back to pre-rolling, instead
  of playing each late frame the moment it trickles in. After a stall the
  backlog usually arrives as a burst, refills the target at once, and playout
  resumes; each stall therefore costs one gap rather than a string of stutters.
* **Flow control: credit on receipt, bounded by free space.** The parser
  normally returns a frame's credit (`COMMAND_WINDOW_UPDATE`) right after
  dispatching it. A TX audio frame's credit is claimed by the buffer
  (`protocolClaimFrameAck()`), and handed back as soon as the frame is queued,
  as long as the buffer could still take everything the host may send with the
  credit it then has. The rule (`TxJitterBuffer::releaseCredit()`): the credit
  held back for a session never drops below its window minus the free buffer
  space. A host with window W and H bytes held back can have at most W - H
  bytes on the way or still to send, and W - H <= free space, so whatever it
  sends next always fits. Credit counts encoded KISS bytes, which are never
  fewer than the payload they carry, so the bound is conservative. Credit
  still held for a frame goes back when the frame is played.

  Why not return credit only when a frame is played (the first version)? With
  the buffer as large as the BLE window, that was the only safe choice, but it
  meant the pre-roll held about 300 ms of credit: a host measuring credit per
  second about one second into an over saw 0.33 to 0.66 of real time and could
  take a healthy link for a starved one. The buffer is now twice the BLE window
  (8 KB vs 4 KB), so with the pre-roll and normal jitter in it the free space
  still exceeds the window and every frame's credit goes straight back: the
  host sees real-time credit from the first frame. Only a faster than
  real-time burst (record-then-send) fills it past about 4 KB (~500 ms); from
  there credit is held back and released as audio plays, so the buffer, not
  the host window, is the bound, and a compliant host can never overflow it.
  The summary's `credit_held` field shows the most credit held back at once
  (0 for a real-time over). If a host overruns its window anyway, the oldest
  audio is played early (nothing is dropped, since dropping part of the ADPCM
  byte stream would misalign every later block). Credits for frames still
  buffered when TX ends (watchdog, runaway timer, switching to FreeDV while
  keyed) are returned too, and credits belonging to a session that
  disconnected are dropped, since a reconnecting host starts a fresh window.
  Credits are sent as one `COMMAND_WINDOW_UPDATE` per session per loop pass,
  so returning many frames at once does not flood a congested BLE notify
  queue. A session with an unknown window (0) keeps the old behaviour (credit
  returned when played). The 72 entries cover frames of one ADPCM block or
  more, which is what every known host sends; a host sending smaller frames
  could run out of entries first and would then get early playout.
* **Pacing.** The I2S TX DMA is a ring of 6 x 512 samples (64 ms at 48 kHz)
  that never stops. Playout is done one ADPCM block (128 B, 249 samples at
  16 kHz, 747 at 48 kHz) at a time whatever the host frame size, written each
  time a block plus one DMA buffer has been consumed, measured with
  `esp_timer_get_time()` against the PCM bytes actually written. A 512 B frame
  is therefore played as four blocks over four slots, not written at once (a
  62 ms write into a 64 ms ring would wait about 30 ms for space, which is what
  the first version did with pgpilot frames: `i2s_blocked` at about half of
  every over). Writes never block on I2S, the loop keeps spinning, and
  incoming frames are parsed as fast as they arrive. If the loop is held up
  long enough for the DAC to replay old buffers, the summary counts a `late`
  tick and pacing resumes without trying to catch up. The DMA geometry
  (6 x 512) is set explicitly in `initI2STx()`. The pacing is open loop against
  `esp_timer`. Both clocks derive from the same crystal and the margin is about
  35 ms, so drift should not matter within the 200 s runaway limit. The
  summary's `i2s_blocked` field checks this on real hardware: it should stay
  at 0 ms.
* **PTT release.** When PTT is released with audio still buffered, the radio
  stays keyed until the buffer and the DMA ring have played out
  (`TX_DRAIN_MAX_MS` = 1.5 s cap, above the ~1 s a full buffer holds), so the
  last 300 ms of speech is not cut. A filter or emphasis change that arrives
  during the drain waits for it and is applied right after unkey. Hosts often
  release in two writes (pgpilot: PTT off with pre-emphasis still set, then
  pre-emphasis off); the first version refused the second write's filter
  change under carrier by unkeying at once, which threw away the buffered tail
  of every over (4 to 5 frames, about 250 to 310 ms). Re-keying during the
  drain cancels it, continues the same over (no new pre-roll while audio is
  still buffered, one summary line at the end) and restarts the TX watchdog.
  The TX watchdog (778 ms without frames), the runaway timer, revoking
  TX_ALLOWED, and a release that also retunes the SA818 (frequency, bandwidth,
  tones, squelch) still unkey immediately: the SA818 is never sent a group
  command under a carrier it was asked to drop. A filter change that waited
  for a drain is also applied after those unkeys. While PTT is held,
  configuration changes apply as they arrive, as upstream always did. The
  decision is `planTxReconcile()` in `txKeyPolicy.h`. End-of-TX bookkeeping
  (credit returns, the summary line) runs only after the transmitter is off.
* **FreeDV 2400B and AX.25** do not use this path, and AX.25 packets get no
  summary line.
* **Feature bit.** HELLO advertises `FEATURE_HAS_TX_JITTER_BUFFER` (bit 7,
  chosen to stay clear of upstream's next allocations until agreed) so hosts
  can tell which behaviour they are talking to. Existing hosts ignore unknown
  bits.

**What a host should do with bit 7.** When the bit is set, the radio buffers
about 300 ms of audio before it plays (40 ms on USB) and keeps the carrier
after PTT release until that audio has played. A host should then:

* keep sending audio in real time from PTT-down and not pre-buffer on its
  side for jitter (the radio does that); frames of one or more whole 128-byte
  ADPCM blocks are fine;
* expect credit to come back as frames arrive (real time from the first
  frame), and only hold back during a faster than real-time burst, so a credit
  rate measured from the first WINDOW_UPDATE is a fair link measure;
* expect `DEVICE_STATE_TX_ACTIVE` to stay set for up to about 300 to 400 ms
  after PTT release (longer after a burst, up to about 1 s), and not treat
  that as a stuck transmitter; filter or emphasis changes sent with or after
  the release are applied once it drops;
* expect a PTT re-press within that window to continue the same over;
* not count the pre-roll as latency to compensate for: mouth-to-air is about
  300 ms more than without the bit.

Without the bit, the radio plays frames as they arrive and returns credit at
once, as upstream firmware does.

**When to key the SA818.** The transmitter is keyed at PTT-down, as before,
and the pre-roll goes out as silence on the carrier. Receivers need time to
open: carrier squelch, and especially CTCSS decoders, typically need 100 to
250 ms of carrier before they unmute. Keying first means that time overlaps
the pre-roll instead of adding to it, so the first syllable is not clipped. It
also keeps PTT semantics unchanged for hosts: `DEVICE_STATE_TX_ACTIVE` still
follows PTT-down immediately, and host-side checks that wait for it (pgpilot's
key watch, for example) are unaffected. The cost is up to 300 ms of carrier
before speech, which is normal for a voice over.

**Latency.** Mouth-to-air goes up by the pre-roll (about 300 ms over
Bluetooth, 40 ms over USB) plus about 50 ms of DMA. On a link that stalls and
then bursts, the radio-side backlog can now grow to the buffer size (about
1 s) instead of one window (about 470 ms); the total delay is the same either
way, since the rest of the backlog used to wait on the host instead.

**Memory.** The 8 KB of audio storage is allocated on the heap once, at boot,
so it does not count against the static DRAM segment, which is the tight
resource on this build: with Bluetooth enabled `dram0_0_seg` is 124,580 B and
upstream main already used 110,740 B. The static part of the buffer (entry
table and counters) is under 1 KB, so this build uses less static DRAM than
the first version with its 4 KB static buffer. The heap is 8 KB smaller
instead; free and minimum free heap are printed at boot and in each TX
summary (`heap`, `min`).

### TX summary line

When a voice transmission ends, one `COMMAND_DEBUG_INFO` line is sent to
every connected session including USB, in release builds as well (values
illustrative, a 10 s pgpilot over with 512 B frames):

```
TX summary: keyed=10652 ms frames_in=161 bytes_in=82432 played=161 (~10022 ms) preroll=300 ms reached=yes@312 ms underruns=0 gap_silence=0 ms lead_silence=312 ms max_frame_gap=81 ms max_ble_write_gap=45 ms peak=2560 B (311 ms) overflows=0 late=0 i2s_blocked=0 ms discarded=0 drain=331 ms ble=yes interval=15.00 ms latency=0 mtu=247 dle=251/251 heap=53042 min=40698 credit_held=0 B
```

The fields and their order are those of the first version, with
`credit_held` appended at the end, so parsers that split on spaces and `=`
keep working.

| Field | Meaning |
| --- | --- |
| `keyed` | Time in TX mode (includes pre-roll and drain) |
| `frames_in`, `bytes_in` | TX audio frames and payload bytes received |
| `played` | Frames played out in full, with the decoded audio time actually written to the DAC (real duration, whatever the frame size) |
| `preroll`, `reached` | Pre-roll target, and when playout first started after keying |
| `underruns` | Times the buffer ran dry while playing |
| `gap_silence` | Silence inserted mid-transmission (underruns and re-pre-roll): **the audible dropouts** |
| `lead_silence` | Silence before the first audio (the pre-roll) |
| `max_frame_gap` | Longest gap between two TX audio frames reaching the buffer |
| `max_ble_write_gap` | Longest gap between two GATT writes from the phone (BLE starvation) |
| `peak` | Highest buffer fill, in bytes and audio time |
| `overflows` | Frames that did not fit and forced an early play (host overran its credit) |
| `late` | Times the main loop fell behind the DAC |
| `i2s_blocked` | Time spent in I2S writes that took over 6 ms, i.e. waited for a DMA buffer (should be 0; growth means the DAC runs slower than the pacing model) |
| `discarded` | Frames still buffered when TX ended without a drain (watchdog, timeout, retune) |
| `drain` | Time between PTT release and unkey |
| `ble`, `interval`, `latency`, `mtu`, `dle` | Link state at the end of the transmission |
| `heap`, `min` | Free heap now and the lowest it has been since boot |
| `credit_held` | Most flow-control credit held back at once; 0 when the host sent in real time, up to one window after a burst |

A clean over has `underruns=0 gap_silence=0 overflows=0 late=0 i2s_blocked=0 ms discarded=0`,
and `played` close to `keyed` minus `lead_silence` and the ~50 ms DMA tail.
Headset starvation shows up as a large `max_ble_write_gap` and, once a gap
exceeds what the buffer holds, as `underruns` and `gap_silence`.

## 5. Fragmented ACL reassembly (#472) and the IDF upgrade path

This change does not patch Bluedroid. Its only guard is item 2: with 251-byte
link-layer PDUs and a 247-byte MTU, a full write from the phone arrives as one
HCI ACL packet, so `reassemble_and_dispatch()` has nothing to reassemble. The
esp-idf#18414 reporter saw problems only with fragmented L2CAP PDUs and never
with single-fragment 244-byte ones. Whether this removes the #472 panic is
unverified until a symbolicated core dump shows where it crashes (item 1).

The proper fix is a newer Bluedroid. What that would take:

* Arduino-ESP32 2.0.17 (`espressif32 @ 6.10.0`) is on IDF v4.4.7, and
  release/v4.4 never received the `packet_fragmenter.c` fixes. They landed in
  IDF v5.1 and v5.5 (CVE-2020-0022 fix, length/pointer checks) and the
  esp-idf#18414 fix in v5.5.5 and v6.0.1. IDF 5.5.3 and 5.5.4 introduced the
  #18414 regression and are worse than 4.4 here.
* The first Arduino core with all of them is **Arduino-ESP32 3.3.12** (IDF
  v5.5.5). Avoid 3.3.7 to 3.3.11.
* PlatformIO's official `espressif32` platform stops at Arduino 2.x; 3.x needs
  the community `pioarduino` platform.
* Expected porting work: the I2S driver API (Arduino 3 / IDF 5 drop the legacy
  driver that `arduino-audio-tools` 1.2.3 uses here, so that library and the
  PDM/ADC paths need updating), `BluetoothSerial` changes, the BLE library's
  `std::string` to `String` API change, ADC and timer API changes, and
  re-validating the DRA818, esp32-afsk and FreeDV libraries. The README already
  notes that the code needs updating for 3.x.
* Alternatives: backport the small Apache-2.0 fixes into a custom IDF 4.4 build
  of the Arduino libraries (esp32-arduino-lib-builder), or use NimBLE for BLE in
  a BLE-only build (Classic SPP needs Bluedroid).

## Bench test plan

Goal: judge TX audio delivery under a headset link from logs alone, with no
second receiver and no audio on the air.

Equipment: the radio on USB to a computer (power and logs), a phone running
the host app connected to the radio over BLE, and a Bluetooth headset paired
to that phone. The radio either runs the `esp32dev-bench-no-rf` build (never
keys the SA818: silent, no antenna needed) or the release build at **low
power into a dummy load**. Do not transmit without an antenna or dummy load on
a normal build.

0. **Save evidence first.** If the radio has restarted in the field, read the
   reset reason and the core dump before flashing anything (section 1).
1. **Flash.** `pio run -e esp32dev-bench-no-rf -t upload --upload-port $PORT`
   (or the release env). Flashing the app does not touch the coredump partition.
2. **Log.** `uv run tools/kv4p_log.py $PORT --raw-out bench.bin | tee bench.log`.
   Expect the banner with `Reset reason: ...` and, for the bench build, the
   `BENCH BUILD` line.
3. **Connect** the phone over BLE. Expect `Reset reason` again and
   `BLE link: ... interval=15.00 ms latency=0 ... mtu=247 dle_tx=251`. Note
   the interval and latency the phone actually granted, and whether DLE was
   reported.
4. **Baseline, no headset.** Five overs of about 10 s each, with any sound into
   the phone mic. Expect per over: `underruns=0 gap_silence=0 overflows=0
   late=0 i2s_blocked=0 ms discarded=0 credit_held=0 B`, `reached` near
   300 ms, `drain` about 300 to 400 ms (the tail is played even when the host
   resets emphasis in a second write after the release), `played` close to
   `keyed` minus `lead_silence`, and `max_ble_write_gap` well under 100 ms. On
   the host side, the credit rate from the first WINDOW_UPDATE should read as
   real time (pgpilot: no starvation verdict, no fallback).
5. **Headset mic open (HFP/SCO active).** Repeat ten overs of 10 s, with the
   headset as the input. Record `max_ble_write_gap`, `max_frame_gap`,
   `underruns`, `gap_silence` and the `BLE link` interval. This is the
   comparison that matters: if `max_ble_write_gap` stays below about 300 ms,
   the buffer should hide it (`gap_silence=0`); if `underruns` appear, compare
   `gap_silence` with the gaps the old firmware produced (old firmware: every
   gap longer than about 64 ms was audible).
6. **Record-then-send fallback.** Five overs in the app's fallback mode. Expect
   `reached` at or under 300 ms, `peak` up to the buffer size (up to about
   8 KB, 1 s), `credit_held` up to about one window (4 KB), `overflows=0`,
   `underruns=0`, `i2s_blocked=0 ms`, and `keyed` about equal to the recording
   length plus pre-roll and drain (`drain` up to about 1 s).
7. **Long over.** One 60 s over with the headset. Expect `late=0` and
   `i2s_blocked=0 ms` (this is the clock-drift check) and a stable `heap`/`min`.
8. **Release during drain.** Release PTT and press it again within 300 ms,
   then keep talking for a few seconds. Expect one continuous over (the drain
   is cancelled, one summary line at the end) that is not cut by the TX
   watchdog shortly after the re-press.
9. **Disconnect mid-over.** Turn off phone Bluetooth while keyed. Expect the
   TX watchdog to unkey within about 0.8 s (`discarded` may be above 0), and
   on reconnect the same boot number in the `Reset reason` line (no restart).
10. **USB regression check (optional).** With the stock Android app over USB,
    one over: expect `preroll=40 ms`, `underruns=0`.

Pass criteria for the change: steps 4, 6, 7 and 8 clean, and in step 5 fewer
underruns and less gap silence than the per-gap dropouts of the old firmware
for the same `max_ble_write_gap`.

Later, with RF (low power into a dummy load, ideally with a second receiver):
confirm that the first syllable is not clipped on a CTCSS-coded receiver, that
the end of an over is not cut, and repeat step 5 at high power into the dummy
load to see whether `max_ble_write_gap` grows while keyed.
