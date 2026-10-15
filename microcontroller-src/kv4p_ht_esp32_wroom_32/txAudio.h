/*
KV4P-HT (see http://kv4p.com)
Copyright (C) 2026 Vance Vagell

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#pragma once

#include <Arduino.h>
#include <AudioTools.h>
#include <AudioTools/AudioCodecs/CodecADPCM.h>
#include <esp_task_wdt.h>
#include <AfskModulator.h>
#include <FreeDv2400b.h>
#include "globals.h"
#include "protocol.h"
#include "state.h"
#include "utils.h"
#include "dsp/audioResampler.h"
#include "txJitterBuffer.h"

// TX playout jitter buffer. See microcontroller-src/docs/ble-robustness.md.
//
// Audio that arrives over Bluetooth is held until TX_PREROLL_WIRELESS_MS of it
// is buffered (or the first frame is that old) before it is played, and again
// after the buffer runs dry. USB delivers smoothly, so it only gets a small
// pre-roll to absorb loop and scheduling jitter.
static constexpr uint32_t TX_PREROLL_WIRELESS_MS = 300;
static constexpr uint32_t TX_PREROLL_USB_MS = 40;
// Minimum carrier before speech: no host audio is played until this long after
// the SA818 is keyed, on every transport. Receivers need carrier for a while
// before they unmute (carrier squelch, CTCSS decode, the SA818's own TX ramp:
// roughly 150 to 300 ms), and a host that sends faster than real time (record
// then send) fills the pre-roll within tens of ms of keying, which used to put
// the first syllable on the air before the far end had opened. While waiting
// the buffer keeps filling, bounded by free space and credit as always. A live
// host fills the pre-roll in real time from keying, so it reaches this point
// anyway and gets no extra delay over the BLE pre-roll.
static constexpr uint32_t TX_CARRIER_LEAD_MS = 300;
// The I2S driver's TX DMA is a ring of 6 x 512 samples (64 ms at 48 kHz) that
// never stops: a buffer is refilled only when we write, otherwise the DAC
// replays whatever it held last (auto_clear is off). A write only proceeds
// without blocking into buffers the DAC has finished, so playout is paced by
// the clock: write one chunk once a chunk plus one DMA buffer has been consumed
// (the extra buffer keeps i2s_write from ever waiting). Written audio reaches
// the DAC about 40 to 55 ms later.
//
// Host frames vary in size (one 128-byte ADPCM block from the Android app,
// four blocks, ~62 ms, from pgpilot), and a 62 ms frame would not fit the
// 64 ms ring without waiting. So playout is done one ADPCM block (one chunk)
// at a time, whatever the frame size; a chunk decodes to at most
// AUDIO_FRAME_SAMPLES_WIRE samples. The pacing counts the PCM actually
// written, so it follows the real decoded length either way.
static constexpr int TX_DMA_BUFFER_COUNT = 6;
static constexpr int64_t TX_DMA_BUFFER_SAMPLES_48K = 512;
static constexpr int64_t TX_DMA_RING_SAMPLES_48K = TX_DMA_BUFFER_COUNT * TX_DMA_BUFFER_SAMPLES_48K;
static constexpr size_t TX_PLAYOUT_CHUNK_BYTES = AUDIO_FRAME_BYTES;
static constexpr int64_t TX_PLAYOUT_CHUNK_SAMPLES_48K = AUDIO_FRAME_SAMPLES_48K;
static constexpr int64_t TX_PLAYOUT_WRITE_DEFICIT_48K = TX_PLAYOUT_CHUNK_SAMPLES_48K + TX_DMA_BUFFER_SAMPLES_48K;
// If the loop falls this far behind, the DAC has started replaying old
// buffers. Do not try to make up the lost time; resume as if one frame late.
static constexpr int64_t TX_PLAYOUT_STALE_SAMPLES_48K = TX_DMA_RING_SAMPLES_48K - TX_DMA_BUFFER_SAMPLES_48K;
static constexpr int64_t TX_PLAYOUT_RESYNC_DEFICIT_48K = TX_PLAYOUT_WRITE_DEFICIT_48K + TX_PLAYOUT_CHUNK_SAMPLES_48K;
static_assert(TX_PLAYOUT_RESYNC_DEFICIT_48K < TX_PLAYOUT_STALE_SAMPLES_48K, "resync point must be below the stale point");
// Time for the last written frame to leave the DMA ring, plus margin.
static constexpr uint32_t TX_PLAYOUT_FLUSH_US =
    (uint32_t)(TX_DMA_RING_SAMPLES_48K * 1000000LL / AUDIO_SAMPLE_RATE) + 6000;
// Upper bound on how long PTT release waits for buffered audio to play out.
static constexpr uint32_t TX_DRAIN_MAX_MS = 1500;
// Credit kept free for control frames (PTT release, state) when deciding that
// the host is out of window and playout must start.
static constexpr size_t TX_CREDIT_RESERVE_BYTES = 512;
// Two BLE windows (BLE_KISS_WINDOW_SIZE is 4096). Credit is returned when a
// frame is queued as long as the free space still covers the host's whole
// window, so the buffer must be larger than one window plus the pre-roll for
// the host to see real-time credit from the start (BLE: free stays above
// 4096 B up to ~4 KB, ~500 ms, buffered). A host that honours flow control can
// never overflow it; if one does, the oldest frame is played early. Allocated
// on the heap (see txJitterBuffer.h).
static constexpr size_t TX_JITTER_CAPACITY_BYTES = 8192;
// Enough entries for frames of one ADPCM block or more (every known host), so
// the byte limit is reached first; the +8 covers a partly played head frame.
static constexpr size_t TX_JITTER_CAPACITY_ENTRIES = TX_JITTER_CAPACITY_BYTES / AUDIO_FRAME_BYTES + 8;
static constexpr uint32_t TX_JITTER_CAPACITY_MS =
    (uint32_t)((uint64_t)TX_JITTER_CAPACITY_BYTES * AUDIO_FRAME_SAMPLES_WIRE / AUDIO_FRAME_BYTES * 1000 / AUDIO_WIRE_SAMPLE_RATE);
static_assert(TX_CARRIER_LEAD_MS + TX_JITTER_CAPACITY_MS + TX_PLAYOUT_FLUSH_US / 1000 < TX_DRAIN_MAX_MS,
              "a full buffer released right at keying must drain (after the carrier lead) before the drain cap");
static_assert(TX_JITTER_CAPACITY_MS > TX_PREROLL_WIRELESS_MS, "the pre-roll must fit");

// Counts PCM bytes on their way to I2S, so playout can be paced against the
// clock without querying the DMA. Also measures time spent blocked in
// i2s_write, which should stay near zero; if it grows, the DAC is running
// slower than the clock model assumes.
class CountingPrint : public Print {
public:
  explicit CountingPrint(Print &target) : target(target) {}
  size_t write(uint8_t b) override {
    return write(&b, 1);
  }
  size_t write(const uint8_t *data, size_t len) override {
    bytes += len;
    int64_t start = esp_timer_get_time();
    size_t written = target.write(data, len);
    int64_t took = esp_timer_get_time() - start;
    // A mono frame is written one sample at a time by the I2S wrapper, which
    // costs a few ms of CPU on its own; only count waits for a DMA buffer
    // (up to ~11 ms each).
    if (took > 6000) {
      blockedUs += (uint64_t)took;
    }
    return written;
  }
  uint64_t bytes = 0;
  uint64_t blockedUs = 0;
private:
  Print &target;
};

bool txStreamConfigured = false;
bool txDecodeStreamStarted = false;
I2SStream out;
AudioInfo txInfo(AUDIO_SAMPLE_RATE, 1, 16);
AudioInfo txAudioInfo(AUDIO_WIRE_SAMPLE_RATE, 1, 16);
CountingPrint txOutCounter(out);
AudioUpsampleOutput txUpsample(txOutCounter);
VolumeMeter txVolumeMeter(txUpsample);
// The library decoder collects bytes into a 128-byte block and decodes each
// full block. Its position in the block (current_byte) is never reset: end()
// is not safe to re-begin on the pinned library, so the decoder lives across
// every transmission and every host session. One payload that is not a whole
// number of blocks therefore shifts every later block, for good: a shifted IMA
// WAV block header reads a reserved byte into a 16-bit step index > 88, the
// library rejects the block (AVERROR_INVALIDDATA) and outputs no samples, and
// playout fills the gap with silence. The result is a carrier with no audio on
// every over until the ESP32 restarts, surviving reconnects.
//
// This subclass exposes the block position so it can be put back to the start
// at every key-down and before every chunk (chunks start on block
// boundaries), and counts blocks the library rejected, so a desync is both
// bounded and visible.
class TxAdpcmDecoder : public ADPCMDecoder {
public:
  using ADPCMDecoder::ADPCMDecoder;

  // Bytes of a partial block held from earlier writes.
  int pendingBytes() const { return current_byte; }

  // Forget a partial block; returns how many bytes were dropped.
  int realign() {
    int dropped = current_byte;
    current_byte = 0;
    return dropped;
  }

  uint32_t goodBlocks = 0;
  uint32_t badBlocks = 0;  // Decoded to no samples (invalid header or data).

protected:
  bool decode(uint8_t byte) override {
    if (p_decoder == nullptr) return false;
    adpcm_block[current_byte++] = byte;
    if (current_byte >= block_size) {
      adpcm_ffmpeg::AVFrame &frame = p_decoder->decode(&adpcm_block[0], block_size);
      if (frame.nb_samples == 0) {
        badBlocks++;
      } else {
        goodBlocks++;
        p_print->write((uint8_t *)frame.data[0], frame.nb_samples * sizeof(int16_t) * info.channels);
      }
      current_byte = 0;
    }
    return true;
  }
};

TxAdpcmDecoder txAdpcmDecoder(AV_CODEC_ID_ADPCM_IMA_WAV, AUDIO_FRAME_BYTES);
EncodedAudioStream txDecodeStream(&txVolumeMeter, &txAdpcmDecoder);

// Tx runaway detection stuff
uint32_t txStartTime = -1;
const uint16_t RUNAWAY_TX_SEC = 200;

// A host holding PTT must continuously provide voice frames. This releases PTT
// quickly if USB, Bluetooth, or BLE disappears without sending PTT-up.
TxWatchDog txWatchDog;

float txAfskBlock[TX_AFSK_BLOCK_SAMPLES];
int16_t txAfskPcm[TX_AFSK_BLOCK_SAMPLES];

const float TX_AUDIO_LEVEL_FULL_SCALE_RSSI = 90.0f;
const float TX_AUDIO_DB_PER_RSSI = 1.2f;
const float TX_AUDIO_LEVEL_RELEASE = 0.65f;

static float peakHoldTxAudioVolumeRatio(float measuredRatio) {
  static float heldRatio = 0.0f;
  if (measuredRatio > heldRatio) {
    heldRatio = measuredRatio;
    return measuredRatio;
  }
  heldRatio = heldRatio * TX_AUDIO_LEVEL_RELEASE + measuredRatio * (1.0f - TX_AUDIO_LEVEL_RELEASE);
  return heldRatio;
}

static void updateTxAudioLevel(float measuredRatio) {
  float heldRatio = peakHoldTxAudioVolumeRatio(measuredRatio);
  if (heldRatio <= 0.0f) {
    txAudioLevel = 0.0f;
    return;
  }
  txAudioLevel = constrain(TX_AUDIO_LEVEL_FULL_SCALE_RSSI + (20.0f * log10f(heldRatio) / TX_AUDIO_DB_PER_RSSI), 0.0f, 255.0f);
}

static void onAfskTxSamples(const float *samples, size_t count) {
  if (!samples || count == 0 || !txStreamConfigured) {
    return;
  }
  if (count > TX_AFSK_BLOCK_SAMPLES) {
    count = TX_AFSK_BLOCK_SAMPLES;
  }
  for (size_t i = 0; i < count; i++) {
    float s = samples[i] * TX_AFSK_GAIN;
    if (s > 1.0f) s = 1.0f;
    if (s < -1.0f) s = -1.0f;
    txAfskPcm[i] = (int16_t)lroundf(s * 32767.0f);
  }
  out.write((uint8_t *)txAfskPcm, count * sizeof(int16_t));
  esp_task_wdt_reset();
}

AfskModulator afskMod(AUDIO_SAMPLE_RATE, onAfskTxSamples);

static void onFreeDvTxSamples(const int16_t *samples, size_t count) {
  if (samples && count && txStreamConfigured) {
    out.write((const uint8_t *)samples, count * sizeof(int16_t));
    esp_task_wdt_reset();
  }
}

FreeDv2400bModulator freeDvTx(onFreeDvTxSamples);
int16_t freeDvTxScratch[256];

TxJitterBuffer<TX_JITTER_CAPACITY_BYTES, TX_JITTER_CAPACITY_ENTRIES> txJitter(
    AUDIO_WIRE_SAMPLE_RATE, AUDIO_FRAME_SAMPLES_WIRE, AUDIO_FRAME_BYTES);
static const uint8_t txSilenceChunk[AUDIO_FRAME_SAMPLES_WIRE * sizeof(int16_t) / 6] = {};
static_assert((AUDIO_FRAME_SAMPLES_WIRE * sizeof(int16_t)) % sizeof(txSilenceChunk) == 0,
              "silence chunk must divide one wire frame");

struct TxPlayoutStats {
  bool active = false;
  bool hostVoice = false;           // Keyed by host PTT (not an AX.25 packet).
  bool prerollChosen = false;
  uint32_t startMs = 0;
  uint32_t endMs = 0;
  uint32_t leadSilenceSamplesWire = 0;  // Silence before the first audio (pre-roll).
  uint32_t gapSilenceSamplesWire = 0;   // Silence filling underruns mid-transmission.
  uint64_t playedSamples48k = 0;        // Decoded voice actually written to I2S.
  uint32_t lateTicks = 0;           // The main loop fell behind the DAC (DMA ran dry).
  uint32_t framesDiscardedAtEnd = 0;
  uint32_t drainMs = 0;
  bool releasePending = false;
  uint32_t releaseStartMs = 0;
  const ProtocolSession *lastSession = nullptr;
  int64_t anchorUs = 0;
  uint64_t anchorBytes = 0;
  int64_t deficitAdjust = 0;
  int64_t lastAudioWriteUs = 0;
  bool audioWritten = false;
  uint64_t blockedUsAtStart = 0;
  // ADPCM framing (see TxAdpcmDecoder). Per transmission.
  uint32_t badBlocksAtStart = 0;
  uint32_t goodBlocksAtStart = 0;
  uint32_t strayBytes = 0;          // Partial-block bytes dropped to realign.
  uint32_t framesRejected = 0;      // Host frames that were not whole blocks.
  bool desyncReported = false;
};
TxPlayoutStats txPlayout;

using TxJitterEntry = TxJitterBuffer<TX_JITTER_CAPACITY_BYTES, TX_JITTER_CAPACITY_ENTRIES>::Entry;

// Credits are collected per session and sent as one WINDOW_UPDATE per flush,
// so returning many frames at once (catching up, or discarding at the end of
// a transmission) does not flood a congested BLE notify queue.
struct TxCreditBatch {
  static constexpr size_t MAX_SESSIONS = 3;  // USB, Classic, BLE
  const ProtocolSession *session[MAX_SESSIONS] = {};
  uint32_t bytes[MAX_SESSIONS] = {};
};
TxCreditBatch txCredits;

static void txQueueCredit(const ProtocolSession *session, size_t bytes) {
  if (session == nullptr || bytes == 0) {
    return;
  }
  for (size_t i = 0; i < TxCreditBatch::MAX_SESSIONS; i++) {
    if (txCredits.session[i] == session || txCredits.session[i] == nullptr) {
      txCredits.session[i] = session;
      txCredits.bytes[i] += bytes;
      return;
    }
  }
}

static void txQueueCredit(const TxJitterEntry &meta) {
  txQueueCredit(static_cast<const ProtocolSession *>(meta.ackTag), meta.ackBytes);
}

// Return whatever credit the session's window no longer needs held (see
// TxJitterBuffer::releaseCredit). Queued; sent by txFlushCredits().
static void txReleaseCredit(const ProtocolSession *session) {
  if (session != nullptr) {
    txQueueCredit(session, txJitter.releaseCredit(session, session->windowSize));
  }
}

static void txFlushCredits() {
  for (size_t i = 0; i < TxCreditBatch::MAX_SESSIONS; i++) {
    const ProtocolSession *session = txCredits.session[i];
    if (session != nullptr && txCredits.bytes[i] > 0 && protocolSessionConnected(*session)) {
      sendWindowAck(*session->stream, txCredits.bytes[i]);
    }
    txCredits.session[i] = nullptr;
    txCredits.bytes[i] = 0;
  }
}

// Samples the DAC has consumed since keying minus samples we have written: how
// far behind the writer is. Kept between one DMA buffer and one frame more
// than that in steady state.
static int64_t txPlayoutDeficitSamples() {
  int64_t written = (int64_t)((txOutCounter.bytes - txPlayout.anchorBytes) / sizeof(int16_t));
  int64_t elapsed = (esp_timer_get_time() - txPlayout.anchorUs) * AUDIO_SAMPLE_RATE / 1000000;
  return elapsed - written + txPlayout.deficitAdjust;
}

// True while the last audio frame written may still be in the DMA ring.
static bool txPlayoutAudioInFlight() {
  return txPlayout.audioWritten
    && (esp_timer_get_time() - txPlayout.lastAudioWriteUs) < (int64_t)TX_PLAYOUT_FLUSH_US;
}

void initI2STx() {  
  freeDvTx.setMagnitude(16383);
  auto config = out.defaultConfig(TX_MODE);
  config.copyFrom(txInfo);
  config.pin_data = hw.pins.pinAudioOut;
  config.pin_ws = 27;
  config.use_apll = true;
  config.auto_clear = false;
  config.signal_type = PDM;
  // Playout pacing below depends on this DMA geometry; set it explicitly.
  config.buffer_count = TX_DMA_BUFFER_COUNT;
  config.buffer_size = TX_DMA_BUFFER_SAMPLES_48K;
  out.begin(config);
  txUpsample.begin();
  txVolumeMeter.begin(txAudioInfo);
  if (!txDecodeStreamStarted) {
    txDecodeStream.begin(txAudioInfo);
    txDecodeStreamStarted = true;
  }
  i2s_zero_dma_buffer(I2S_NUM_0);
  // Start TX meter at full scale to match radio-style TX indication.
  // Subsequent audio frames update this to the measured TX audio level.
  updateTxAudioLevel(1.0f);
  txStreamConfigured = true;
}

void endI2STx() {
  if (txStreamConfigured) {
    // Set pin to INPUT before stopping I2S to avoid end-of-TX click.
    // If left as output, the last PDM bit may hold the line high or low,
    // causing a DC step across the AC-coupling cap and producing a pop.
    // Forcing the pin to high-Z prevents this.
    pinMode(hw.pins.pinAudioOut, INPUT);
    // ADPCMDecoder::end() is not safe to re-begin on the pinned adpcm library.
    // Keep the decoder alive across PTT transitions and only stop the hardware output path.
    txUpsample.end();
    out.end();
  }
  txStreamConfigured = false;
}

// Report a desync once per transmission, with a stable keyword hosts can match.
static void txReportAdpcmDesync(const char *what) {
  if (txPlayout.desyncReported) {
    return;
  }
  txPlayout.desyncReported = true;
  debug_log_printf(COMMAND_DEBUG_WARN, "TX ADPCM desync: %s; bad_blocks=%lu good_blocks=%lu stray_bytes=%lu rejected_frames=%lu",
                   what, (unsigned long)(txAdpcmDecoder.badBlocks - txPlayout.badBlocksAtStart),
                   (unsigned long)(txAdpcmDecoder.goodBlocks - txPlayout.goodBlocksAtStart),
                   (unsigned long)txPlayout.strayBytes, (unsigned long)txPlayout.framesRejected);
}

// Every call starts on a block boundary (a chunk, or a whole frame of whole
// blocks), so a partial block left in the decoder can only be a leftover:
// drop it rather than let it shift this block.
static void txDecode(const uint8_t *src, size_t len) {
  int stray = txAdpcmDecoder.realign();
  if (stray > 0) {
    txPlayout.strayBytes += (uint32_t)stray;
    txReportAdpcmDesync("partial block before a block boundary, realigned");
  }
  uint32_t badBefore = txAdpcmDecoder.badBlocks;
  uint64_t before = txOutCounter.bytes;
  txDecodeStream.write(src, len);
  txPlayout.playedSamples48k += (txOutCounter.bytes - before) / sizeof(int16_t);
  if (txAdpcmDecoder.badBlocks != badBefore) {
    txReportAdpcmDesync("block decoded to no audio");
  }
}

static void txAfterAudioWrite() {
  updateTxAudioLevel(txVolumeMeter.volumeRatio());
  txPlayout.lastAudioWriteUs = esp_timer_get_time();
  txPlayout.audioWritten = true;
}

static void txPlayFrame(const uint8_t *src, size_t len) {
  txDecode(src, len);
  txAfterAudioWrite();
}

// Play the next chunk (at most one ADPCM block) of the oldest frame.
static void txPlayChunk() {
  TxJitterEntry meta;
  if (txJitter.popChunk(txDecode, TX_PLAYOUT_CHUNK_BYTES, meta) > 0) {
    txAfterAudioWrite();
    txQueueCredit(meta);
  }
}

// Drop everything buffered, returning its credit.
static void txDiscardBuffered() {
  TxJitterEntry meta;
  while (!txJitter.empty()) {
    txJitter.discard(meta);
    txPlayout.framesDiscardedAtEnd++;
    txQueueCredit(meta);
  }
  txFlushCredits();
}

// One wire frame (249 samples) of silence through the upsampler.
static void txWriteSilenceFrame() {
  for (size_t i = 0; i < AUDIO_FRAME_SAMPLES_WIRE * sizeof(int16_t); i += sizeof(txSilenceChunk)) {
    txUpsample.write(txSilenceChunk, sizeof(txSilenceChunk));
  }
}

// Called when entering MODE_TX. hostVoice is false for AX.25 packets.
void txPlayoutBegin(bool hostVoice) {
  uint32_t now = millis();
  txJitter.reset(now, TX_PREROLL_WIRELESS_MS, TX_CARRIER_LEAD_MS);
  txPlayout = TxPlayoutStats{};
  txPlayout.active = true;
  txPlayout.hostVoice = hostVoice;
  txPlayout.startMs = now;
  txPlayout.blockedUsAtStart = txOutCounter.blockedUs;
  txPlayout.anchorUs = esp_timer_get_time();
  txPlayout.anchorBytes = txOutCounter.bytes;
  // A new over always starts on a block boundary, whatever the last one left.
  txPlayout.strayBytes = (uint32_t)txAdpcmDecoder.realign();
  txPlayout.badBlocksAtStart = txAdpcmDecoder.badBlocks;
  txPlayout.goodBlocksAtStart = txAdpcmDecoder.goodBlocks;
  if (txPlayout.strayBytes > 0) {
    txReportAdpcmDesync("previous transmission ended inside a block, realigned at key-down");
  }
}

// Called when leaving MODE_TX. Anything still buffered is discarded, but its
// flow-control credit is returned so the host's window is not lost.
void txPlayoutEnd() {
  if (!txPlayout.active) {
    return;
  }
  txFlushCredits();
  txDiscardBuffered();
  if (txPlayout.releasePending) {
    txPlayout.drainMs = millis() - txPlayout.releaseStartMs;
  }
  txPlayout.releasePending = false;
  txPlayout.active = false;
  txPlayout.endMs = millis();
}

// A host session went away: keep its queued audio but drop its credits.
void txPlayoutForgetSession(const ProtocolSession &session) {
  txJitter.forgetAckTag(&session);
  if (txPlayout.lastSession == &session) {
    txPlayout.lastSession = nullptr;
  }
}

// True while analog voice is still buffered or in the DMA ring, i.e. a PTT
// release would keep the carrier to play it out.
bool txPlayoutCanDrain() {
  if (!txPlayout.active || !txPlayout.hostVoice || freeDv2400bEnabled()) {
    return false;
  }
  return !txJitter.empty() || txPlayoutAudioInFlight();
}

// PTT was released while audio is still buffered: keep the carrier until it
// has played. Returns true if the caller should keep transmitting for now.
bool txPlayoutRequestDrain() {
  if (!txPlayoutCanDrain()) {
    return false;
  }
  if (!txPlayout.releasePending) {
    txPlayout.releasePending = true;
    txPlayout.releaseStartMs = millis();
  }
  return true;
}

// PTT pressed again while draining: carry on with the same transmission.
void txPlayoutCancelDrain() {
  if (txPlayout.releasePending) {
    txPlayout.releasePending = false;
    // setMode(MODE_TX) is a no-op while still keyed, so restart the TX
    // watchdog here; otherwise the new over inherits the old frame time.
    txWatchDog.reset(millis());
  }
}

void processTxAudio(ProtocolSession &session, uint8_t *src, size_t len) {
  if (!src || len == 0 || !txStreamConfigured) {
    return;
  }
  uint32_t now = millis();
  txWatchDog.onFrame(now);
  if (len % AUDIO_FRAME_BYTES != 0) {
    // Not whole ADPCM blocks: bytes were lost or added on the way (for
    // example a full receive ring). Decoding it would shift every block
    // after it, so drop the frame; its credit goes back with the frame ack.
    txPlayout.framesRejected++;
    char what[64];
    snprintf(what, sizeof(what), "frame of %u bytes is not whole %u-byte blocks, dropped", (unsigned)len,
             (unsigned)AUDIO_FRAME_BYTES);
    txReportAdpcmDesync(what);
    return;
  }
  if (!txPlayout.prerollChosen) {
    txJitter.setPrerollMs(&session == &protocolUsbSession ? TX_PREROLL_USB_MS : TX_PREROLL_WIRELESS_MS);
    txPlayout.prerollChosen = true;
  }
  txPlayout.lastSession = &session;
  uint32_t ack = protocolClaimFrameAck(session);
  while (!txJitter.push(src, len, &session, (uint16_t)ack, now)) {
    if (txJitter.empty()) {
      // Larger than the whole buffer: play it directly, as before buffering existed.
      txPlayFrame(src, len);
      if (ack > 0 && protocolSessionConnected(session)) {
        sendWindowAck(*session.stream, ack);
      }
      break;
    }
    // The host sent more than its credit allowed. Play the oldest audio
    // early (blocking on I2S if needed) rather than dropping it: dropping part
    // of the ADPCM byte stream would misalign every block after it.
    txPlayChunk();
  }
  // Credit on receipt, bounded by free space: the host sees real-time credit
  // during the pre-roll, and a faster than real-time burst stops at the buffer.
  txReleaseCredit(&session);
  txFlushCredits();
  esp_task_wdt_reset();
}

// Feed the DAC: one call per loop iteration while keyed for voice.
void txPlayoutLoop() {
  if (!txPlayout.active || !txStreamConfigured) {
    return;
  }
  if (freeDv2400bEnabled()) {
    // Switched to digital voice while keyed: buffered analog audio will never
    // play, so hand its credit back now rather than starving the host.
    if (!txJitter.empty()) {
      txDiscardBuffered();
    }
    return;
  }
  int64_t deficit = txPlayoutDeficitSamples();
  if (deficit > TX_PLAYOUT_STALE_SAMPLES_48K) {
    // The loop was blocked long enough for the DAC to replay old DMA buffers.
    txPlayout.lateTicks++;
    txPlayout.deficitAdjust -= deficit - TX_PLAYOUT_RESYNC_DEFICIT_48K;
    deficit = TX_PLAYOUT_RESYNC_DEFICIT_48K;
  }
  uint32_t now = millis();
  bool creditPressure = false;
  const ProtocolSession *session = txPlayout.lastSession;
  if (session != nullptr && session->windowSize > TX_CREDIT_RESERVE_BYTES) {
    creditPressure = txJitter.heldAckBytesFor(session) + TX_CREDIT_RESERVE_BYTES >= session->windowSize;
  }
  // A few frames at most per call, so catching up never stalls the main loop.
  for (int i = 0; i < 4 && deficit >= TX_PLAYOUT_WRITE_DEFICIT_48K; i++) {
    if (txJitter.shouldPlay(now, creditPressure, txPlayout.releasePending)) {
      txPlayChunk();
    } else {
      // Explicit silence: with auto_clear off the DMA would otherwise loop
      // over its last buffers, which is the stutter this buffer exists to fix.
      // While draining after PTT release this is just the tail, not an underrun.
      txWriteSilenceFrame();
      if (!txPlayout.releasePending) {
        txJitter.noteEmptySlot();
        if (txJitter.stats().prerollReached) {
          txPlayout.gapSilenceSamplesWire += AUDIO_FRAME_SAMPLES_WIRE;
        } else {
          txPlayout.leadSilenceSamplesWire += AUDIO_FRAME_SAMPLES_WIRE;
        }
      }
    }
    deficit = txPlayoutDeficitSamples();
  }
  // Playing freed space: credit held back for a burst can go now.
  txReleaseCredit(txPlayout.lastSession);
  txFlushCredits();
}

// True once a pending PTT release has played out (or waited too long).
bool txPlayoutDrainComplete() {
  if (!txPlayout.releasePending) {
    return false;
  }
  if ((uint32_t)(millis() - txPlayout.releaseStartMs) >= TX_DRAIN_MAX_MS) {
    return true;
  }
  return txJitter.empty() && !txPlayoutAudioInFlight();
}

void processTxAx25(const uint8_t *src, size_t len, float txDelayMs, float txTailMs) {
  if (!src || len == 0) {
    return;
  }
  // esp32-afsk's public API supplies a fixed flag preamble; its runtime lead
  // parameter is silence, so TXDELAY is currently carrier lead time, not an
  // exactly sized flag train.
  afskMod.modulate(src, len, txAfskBlock, TX_AFSK_BLOCK_SAMPLES, txDelayMs,
    txTailMs);
}

void processTxDigital(uint8_t *src, size_t len) {
  if (!src || len != freedv2400b::PAYLOAD_BYTES || !txStreamConfigured) return;
  txWatchDog.onFrame(millis());
  if (!freeDvTx.modulate(src, len, freeDvTxScratch,
                         sizeof(freeDvTxScratch) / sizeof(freeDvTxScratch[0]))) {
    _LOGE("Unable to modulate FreeDV 2400B TX frame");
    return;
  }
  updateTxAudioLevel(1.0f);
}

void releaseHostPtt() {
  if (desiredState.flags & HOST_STATE_PTT_REQUESTED) {
    desiredState.flags &= ~HOST_STATE_PTT_REQUESTED;
    desiredState.sequence++;
  }
  setMode(rxIdleMode());
  applyDeferredFilters();
  markDeviceStateDirty();
}

void inline txAudioLoop() {
  if (mode == MODE_TX) {
    if (txPlayoutDrainComplete()) {
      setMode(rxIdleMode());
      // A filter or emphasis change that arrived during the drain.
      applyDeferredFilters();
      markDeviceStateDirty();
      esp_task_wdt_reset();
      return;
    }
    txPlayoutLoop();
    if ((desiredState.flags & HOST_STATE_PTT_REQUESTED) && txWatchDog.expired(millis())) {
      _LOGW("TX audio timeout; releasing host PTT");
      releaseHostPtt();
      esp_task_wdt_reset();
      return;
    }
    // Check for runaway tx
    if ((millis() - txStartTime) > RUNAWAY_TX_SEC * 1000) {
      releaseHostPtt();
      esp_task_wdt_reset();
    }
  }
}
