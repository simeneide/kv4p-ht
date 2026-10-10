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
// The I2S driver's TX DMA is a ring of 6 x 512 samples (64 ms at 48 kHz) that
// never stops: a buffer is refilled only when we write, otherwise the DAC
// replays whatever it held last (auto_clear is off). A write only proceeds
// without blocking into buffers the DAC has finished, so playout is paced by
// the clock: write a frame once a frame plus one DMA buffer has been consumed
// (the extra buffer keeps i2s_write from ever waiting). Written audio reaches
// the DAC about 40 to 55 ms later.
static constexpr int TX_DMA_BUFFER_COUNT = 6;
static constexpr int64_t TX_DMA_BUFFER_SAMPLES_48K = 512;
static constexpr int64_t TX_DMA_RING_SAMPLES_48K = TX_DMA_BUFFER_COUNT * TX_DMA_BUFFER_SAMPLES_48K;
static constexpr int64_t TX_PLAYOUT_WRITE_DEFICIT_48K = AUDIO_FRAME_SAMPLES_48K + TX_DMA_BUFFER_SAMPLES_48K;
// If the loop falls this far behind, the DAC has started replaying old
// buffers. Do not try to make up the lost time; resume as if one frame late.
static constexpr int64_t TX_PLAYOUT_STALE_SAMPLES_48K = TX_DMA_RING_SAMPLES_48K - TX_DMA_BUFFER_SAMPLES_48K;
static constexpr int64_t TX_PLAYOUT_RESYNC_DEFICIT_48K = TX_PLAYOUT_WRITE_DEFICIT_48K + AUDIO_FRAME_SAMPLES_48K;
static_assert(TX_PLAYOUT_RESYNC_DEFICIT_48K < TX_PLAYOUT_STALE_SAMPLES_48K, "resync point must be below the stale point");
// Time for the last written frame to leave the DMA ring, plus margin.
static constexpr uint32_t TX_PLAYOUT_FLUSH_US =
    (uint32_t)(TX_DMA_RING_SAMPLES_48K * 1000000LL / AUDIO_SAMPLE_RATE) + 6000;
// Upper bound on how long PTT release waits for buffered audio to play out.
static constexpr uint32_t TX_DRAIN_MAX_MS = 1500;
// Credit kept free for control frames (PTT release, state) when deciding that
// the host is out of window and playout must start.
static constexpr size_t TX_CREDIT_RESERVE_BYTES = 512;
// Sized for one BLE window (4096 B of KISS frames carry less than 4096 B of
// audio payload), so a single host that honours flow control can never
// overflow it. Static DRAM is tight on this build (BT enabled), so this is
// deliberately not larger; if it does fill, the oldest frame is played early.
static constexpr size_t TX_JITTER_CAPACITY_BYTES = 4096;
static constexpr size_t TX_JITTER_CAPACITY_ENTRIES = 64;

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
ADPCMDecoder txAdpcmDecoder(AV_CODEC_ID_ADPCM_IMA_WAV, AUDIO_FRAME_BYTES);
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

static void txQueueCredit(const TxJitterEntry &meta) {
  const ProtocolSession *session = static_cast<const ProtocolSession *>(meta.ackTag);
  if (session == nullptr || meta.ackBytes == 0) {
    return;
  }
  for (size_t i = 0; i < TxCreditBatch::MAX_SESSIONS; i++) {
    if (txCredits.session[i] == session || txCredits.session[i] == nullptr) {
      txCredits.session[i] = session;
      txCredits.bytes[i] += meta.ackBytes;
      return;
    }
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

static void txDecode(const uint8_t *src, size_t len) {
  txDecodeStream.write(src, len);
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

static void txPlayOldest() {
  TxJitterEntry meta;
  if (txJitter.pop(txDecode, meta) > 0) {
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
  txJitter.reset(now, TX_PREROLL_WIRELESS_MS);
  txPlayout = TxPlayoutStats{};
  txPlayout.active = true;
  txPlayout.hostVoice = hostVoice;
  txPlayout.startMs = now;
  txPlayout.blockedUsAtStart = txOutCounter.blockedUs;
  txPlayout.anchorUs = esp_timer_get_time();
  txPlayout.anchorBytes = txOutCounter.bytes;
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

// PTT was released while audio is still buffered: keep the carrier until it
// has played. Returns true if the caller should keep transmitting for now.
bool txPlayoutRequestDrain() {
  if (!txPlayout.active || freeDv2400bEnabled()) {
    return false;
  }
  if (txJitter.empty() && !txPlayoutAudioInFlight()) {
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
    // The host sent more than its window. Play the oldest frame early
    // (blocking on I2S if needed) rather than dropping audio: dropping part of
    // the ADPCM byte stream would misalign every block after it.
    txPlayOldest();
  }
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
      txPlayOldest();
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
  markDeviceStateDirty();
}

void inline txAudioLoop() {
  if (mode == MODE_TX) {
    if (txPlayoutDrainComplete()) {
      setMode(rxIdleMode());
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
