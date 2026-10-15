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

#include <Arduino.h>
#include <BluetoothSerial.h>
#include <DRA818.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <functional>
#include "globals.h"
#include "firmwareVersion.h"
#include "debug.h"
#include "BluedroidBleKissGattStream.h"
#include "resetDiagnostics.h"
#include "led.h"
#include "protocol.h"
#include "state.h"
#include "rxAudio.h"
#include "txKeyPolicy.h"
#include "txAudio.h"
#include "ax25TxScheduler.h"
#include "buttons.h"
#include "utils.h"
#include "board.h"

const uint32_t RSSI_REPORT_INTERVAL_MS = 100;
const uint32_t DEVICE_STATE_REPORT_INTERVAL_MS = 500;
const uint16_t USB_BUFFER_SIZE = 1024*2;
const uint16_t BLE_KISS_WINDOW_SIZE = 4096;

using Kv4pBleKissStream = BluedroidBleKissGattStream<4096, 512, 32>;

char bleKissDeviceName[12] = "kv4p";

Kv4pBleKissStream::Config bleKissConfig() {
  Kv4pBleKissStream::Config cfg;
  cfg.deviceName = bleKissDeviceName;
  cfg.preferredMtu = 247;
  cfg.requireNotifySubscription = true;
  cfg.maxNotifyChunksPerLoop = 4;
  cfg.minNotifyIntervalMs = 0;
  cfg.notifyFailureBackoffMs = 25;
  cfg.writeQueueWaitMs = 15;
  cfg.txPower = ESP_PWR_LVL_P7;
  return cfg;
}

DRA818 sa818_vhf(&Serial2, SA818_VHF);
DRA818 sa818_uhf(&Serial2, SA818_UHF);
DRA818 &sa818 = sa818_vhf;
BluetoothSerial SerialBT;
Kv4pBleKissStream bleKissStream(bleKissConfig());
bool bluetoothStarted = false;
bool bluetoothProtocolConnected = false;
bool bleKissProtocolConnected = false;
KissParser bluetoothParser(protocolBtSession, &handleCommands, &handleAx25Data,
  &handleKissParameter);
KissParser bleKissParser(protocolBleSession, &handleCommands, &handleAx25Data,
  &handleKissParameter);
Ax25TxScheduler ax25TxScheduler;
// SoftSQ needs its 250 ms close/open interval after a retune before its raw
// HF-noise carrier decision is reliable. Keep a small loop-timing margin.
static constexpr uint16_t AX25_OVERRIDE_RX_SETTLE_MS = 260;
bool ax25OverrideChannelPrepared = false;
uint32_t ax25OverrideChannelReadyAt = 0;
// RF module recovery: power-down pulse length and settle time, and the
// background probe for a module that is missing.
static constexpr uint32_t SA818_PD_LOW_MS = 200;
static constexpr uint32_t SA818_PD_SETTLE_MS = 1000;
static constexpr int SA818_BOOT_HANDSHAKES = 9;
static constexpr uint32_t SA818_BOOT_ANSWER_MS = 2000;
static constexpr uint32_t SA818_PROBE_INTERVAL_MS = 5000;
static constexpr uint32_t SA818_PROBE_ANSWER_MS = 300;

float moduleMinRadioFreq() {
  return hw.rfModuleType == RF_SA818_UHF ? 400.0f : 134.0f;
}

float moduleMaxRadioFreq() {
  return hw.rfModuleType == RF_SA818_UHF ? 480.0f : 174.0f;
}

float moduleDefaultRadioFreq() {
  return hw.rfModuleType == RF_SA818_UHF ? 435.0f : 144.0f;
}

float clampModuleRadioFreq(float freq) {
  return min(max(freq, moduleMinRadioFreq()), moduleMaxRadioFreq());
}

bool isModuleRadioFreq(float freq) {
  return freq >= moduleMinRadioFreq() && freq <= moduleMaxRadioFreq();
}

void loadPersistedRadioState() {
  prefs.begin("radio", true);
  uint16_t flags = desiredState.flags;
  flags &= ~(HOST_STATE_RADIO_CONFIG_VALID | HOST_STATE_HIGH_POWER | HOST_STATE_RSSI_ENABLED | HOST_STATE_FILTER_PRE | HOST_STATE_FILTER_HIGH | HOST_STATE_FILTER_LOW | HOST_STATE_TX_ALLOWED);
  flags |= HOST_STATE_RADIO_CONFIG_VALID;
  desiredState.bw = prefs.getUChar("bw", DRA818_25K);
  desiredState.freq_tx = prefs.getFloat("freq_tx", moduleDefaultRadioFreq());
  desiredState.freq_rx = prefs.getFloat("freq_rx", moduleDefaultRadioFreq());
  desiredState.ctcss_tx = prefs.getUChar("ctcss_tx", 0);
  desiredState.squelch = prefs.getUChar("squelch", 0);
  desiredState.ctcss_rx = prefs.getUChar("ctcss_rx", 0);
  desiredState.memoryId = prefs.getInt("memory_id", -1);
  if (!isModuleRadioFreq(desiredState.freq_tx) || !isModuleRadioFreq(desiredState.freq_rx)) {
    desiredState.freq_tx = clampModuleRadioFreq(desiredState.freq_tx);
    desiredState.freq_rx = clampModuleRadioFreq(desiredState.freq_rx);
    desiredState.memoryId = -1;
  }
  if (prefs.getBool("high", true)) {
    flags |= HOST_STATE_HIGH_POWER;
  }
  if (prefs.getBool("rssi", true)) {
    flags |= HOST_STATE_RSSI_ENABLED;
  }
  if (prefs.getBool("flt_pre", false)) {
    flags |= HOST_STATE_FILTER_PRE;
  }
  if (prefs.getBool("flt_high", false)) {
    flags |= HOST_STATE_FILTER_HIGH;
  }
  if (prefs.getBool("flt_low", false)) {
    flags |= HOST_STATE_FILTER_LOW;
  }
  if (prefs.getBool("tx_allowed", false)) {
    flags |= HOST_STATE_TX_ALLOWED;
  }
  desiredState.flags = flags;
  softSquelchEffect.setDeadbandLevel(desiredState.squelch);
  desiredState.sequence = 0;
  desiredState.flags &= ~(HOST_STATE_PTT_REQUESTED | HOST_STATE_SESSION_FLAG_MASK);
  appliedState.memoryId = desiredState.memoryId;
  persistedState = desiredState;
  prefs.end();
}

uint16_t persistedStateFlags(uint16_t flags) {
  return flags & (HOST_STATE_RADIO_CONFIG_VALID | HOST_STATE_HIGH_POWER | HOST_STATE_RSSI_ENABLED | HOST_STATE_FILTER_PRE | HOST_STATE_FILTER_HIGH | HOST_STATE_FILTER_LOW | HOST_STATE_TX_ALLOWED);
}

bool persistedRadioStateMatchesDesired() {
  return persistedState.bw == desiredState.bw
    && persistedState.freq_tx == desiredState.freq_tx
    && persistedState.freq_rx == desiredState.freq_rx
    && persistedState.ctcss_tx == desiredState.ctcss_tx
    && persistedState.squelch == desiredState.squelch
    && persistedState.ctcss_rx == desiredState.ctcss_rx
    && persistedState.memoryId == desiredState.memoryId
    && persistedStateFlags(persistedState.flags) == persistedStateFlags(desiredState.flags);
}

void savePersistedRadioStateIfChanged() {
  if (persistedRadioStateMatchesDesired()) {
    return;
  }
  prefs.begin("radio", false);
  prefs.putUChar("bw", desiredState.bw);
  prefs.putFloat("freq_tx", desiredState.freq_tx);
  prefs.putFloat("freq_rx", desiredState.freq_rx);
  prefs.putUChar("ctcss_tx", desiredState.ctcss_tx);
  prefs.putUChar("squelch", desiredState.squelch);
  prefs.putUChar("ctcss_rx", desiredState.ctcss_rx);
  prefs.putInt("memory_id", desiredState.memoryId);
  prefs.putBool("high", (desiredState.flags & HOST_STATE_HIGH_POWER) != 0);
  prefs.putBool("rssi", (desiredState.flags & HOST_STATE_RSSI_ENABLED) != 0);
  prefs.putBool("flt_pre", (desiredState.flags & HOST_STATE_FILTER_PRE) != 0);
  prefs.putBool("flt_high", (desiredState.flags & HOST_STATE_FILTER_HIGH) != 0);
  prefs.putBool("flt_low", (desiredState.flags & HOST_STATE_FILTER_LOW) != 0);
  prefs.putBool("tx_allowed", (desiredState.flags & HOST_STATE_TX_ALLOWED) != 0);
  prefs.end();
  persistedState = desiredState;
}

uint8_t getFirmwareFeatures() {
  return (hw.features.hasHL ? FEATURE_HAS_HL : 0)
    | (hw.features.hasPhysPTT ? FEATURE_HAS_PHY_PTT : 0)
    | FEATURE_HAS_ESP32_AFSK
    | FEATURE_HAS_FREEDV_2400B
    | FEATURE_HAS_TX_JITTER_BUFFER;
}

Mode rxIdleMode() {
  return protocolAnySessionFlag(HOST_STATE_RX_AUDIO_OPEN) ? MODE_RX : MODE_STOPPED;
}

uint16_t desiredFilterFlags() {
  return desiredState.flags & (HOST_STATE_FILTER_PRE | HOST_STATE_FILTER_HIGH | HOST_STATE_FILTER_LOW);
}

bool txAllowedByHost() {
  return desiredState.flags & HOST_STATE_TX_ALLOWED;
}

bool freeDv2400bEnabled() {
  return desiredState.flags & HOST_STATE_FREEDV_2400B;
}

uint16_t deviceStateFlags(uint16_t sessionFlags) {
  uint16_t flags = desiredState.flags;
  flags |= sessionFlags & HOST_STATE_SESSION_FLAG_MASK;
  if (isPhysPttDown) {
    flags |= DEVICE_STATE_PHYS_PTT_DOWN;
  }
  if (mode == MODE_TX) {
    flags |= DEVICE_STATE_TX_ACTIVE;
  }
  if (squelched) {
    flags |= DEVICE_STATE_SQUELCHED;
  }
  return flags;
}

uint8_t deviceMode() {
  switch (mode) {
    case MODE_TX:
      return DEVICE_MODE_TX;
    case MODE_RX:
      return DEVICE_MODE_RX;
    case MODE_STOPPED:
    default:
      return DEVICE_MODE_STOPPED;
  }
}

DeviceState currentDeviceState(uint16_t sessionFlags = 0) {
  return {
    .appliedSequence = desiredState.sequence,
    .memoryId = appliedState.memoryId,
    .flags = deviceStateFlags(sessionFlags),
    .bw = appliedState.bw,
    .freq_tx = appliedState.freq_tx,
    .freq_rx = appliedState.freq_rx,
    .ctcss_tx = appliedState.ctcss_tx,
    .squelch = appliedState.squelch,
    .ctcss_rx = appliedState.ctcss_rx,
    .radioModuleStatus = radioModuleStatus,
    .mode = deviceMode(),
    .lastError = lastDeviceStateError,
    .latestRssi = latestRssi,
  };
}

void sendCurrentDeviceState() {
  bool sent = false;
  if (protocolSessionConnected(protocolUsbSession) && (protocolUsbSession.flags & HOST_STATE_ENABLE_STATUS_REPORTS)) {
    sendDeviceState(*protocolUsbSession.stream, currentDeviceState(protocolUsbSession.flags));
    sent = true;
  }
  if (protocolHasBtSession() && (protocolBtSession.flags & HOST_STATE_ENABLE_STATUS_REPORTS)) {
    sendDeviceState(*protocolBtSession.stream, currentDeviceState(protocolBtSession.flags));
    sent = true;
  }
  if (protocolHasBleSession() && (protocolBleSession.flags & HOST_STATE_ENABLE_STATUS_REPORTS)) {
    sendDeviceState(*protocolBleSession.stream, currentDeviceState(protocolBleSession.flags));
    sent = true;
  }
  if (sent) {
    deviceStateDirty = false;
  }
}

void markDeviceStateDirty() {
  deviceStateDirty = true;
}

bool radioConfigChanged() {
  return !radioConfigApplied
    || appliedState.bw != desiredState.bw
    || appliedState.freq_tx != desiredState.freq_tx
    || appliedState.freq_rx != desiredState.freq_rx
    || appliedState.ctcss_tx != desiredState.ctcss_tx
    || appliedState.squelch != desiredState.squelch
    || appliedState.ctcss_rx != desiredState.ctcss_rx
    || appliedState.memoryId != desiredState.memoryId;
}

void drainRadioSerial() {
  while (Serial2.available()) {
    Serial2.read();
  }
}

uint16_t appliedFilterFlags() {
  return appliedState.flags & (HOST_STATE_FILTER_PRE | HOST_STATE_FILTER_HIGH | HOST_STATE_FILTER_LOW);
}

// The SA818 answers a command within milliseconds, and the library waits up
// to 2 s for that answer. These used to retry forever while feeding the task
// watchdog, so a module that stopped answering (or was never found at boot)
// froze the main loop for good: the radio stayed connected but silent until
// someone reset it. Now a module that misses this many answers in a row is
// marked lost, and radioModuleProbeLoop() looks for it again.
static constexpr int SA818_COMMAND_ATTEMPTS = 3;

// Probes since the module went missing (radioModuleProbeLoop()); reset by
// radioModuleFound() and markRadioModuleLost() so every episode starts the same.
uint32_t sa818Probes = 0;

bool sa818Handshake(uint32_t timeoutMs);

void markRadioModuleLost() {
  sa818Probes = 0;
  if (radioModuleStatus == RADIO_MODULE_FOUND) {
    radioBootInfo.lostCount++;
    radioBootInfo.present = false;
    debug_log_printf(COMMAND_DEBUG_WARN, "SA818 stopped answering; probing for it again");
  }
  radioModuleStatus = RADIO_MODULE_NOT_FOUND;
  radioConfigApplied = false;
  filtersApplied = false;
  markDeviceStateDirty();
}

bool sa818Command(const std::function<bool()> &command, uint8_t errorCode) {
  if (radioModuleStatus != RADIO_MODULE_FOUND) {
    lastDeviceStateError = errorCode;
    return false;
  }
  for (int i = 0; i < SA818_COMMAND_ATTEMPTS; i++) {
    esp_task_wdt_reset();
    if (command()) {
      return true;
    }
    lastDeviceStateError = errorCode;
  }
  // The library cannot tell a refusal from silence. A module that still
  // answers a handshake is there and only refused this command: keep it, so
  // a module that NAKs is not marked lost, re-found and refused again forever.
  if (!sa818Handshake(SA818_PROBE_ANSWER_MS)) {
    markRadioModuleLost();
  }
  return false;
}

void applyFilterFlags() {
  uint16_t filterFlags = desiredFilterFlags();
  if (!sa818Command([&] {
        drainRadioSerial();
        return sa818.filters((filterFlags & HOST_STATE_FILTER_PRE), false, false);
      }, DEVICE_STATE_ERROR_FILTERS_FAILED)) {
    return;  // filtersApplied stays false, so the change is applied once the module answers.
  }
  rxDownsample.setFilters((filterFlags & HOST_STATE_FILTER_HIGH) != 0,
                          (filterFlags & HOST_STATE_FILTER_LOW) != 0);
  appliedState.flags = (appliedState.flags & ~(HOST_STATE_FILTER_PRE | HOST_STATE_FILTER_HIGH | HOST_STATE_FILTER_LOW)) | filterFlags;
  filtersApplied = true;
}

// A filter change that waited for a PTT-release drain: apply it now that the
// carrier is down. Called from every path that unkeys outside
// reconcileDesiredState() (drain complete, TX watchdog, runaway timer).
void applyDeferredFilters() {
  if (mode != MODE_TX && filtersApplied && desiredFilterFlags() != appliedFilterFlags()) {
    applyFilterFlags();
    markDeviceStateDirty();
  }
}

// Keys refused because the TX driver could not be installed (i2sGuard.h).
// The heap shortage is usually brief (Bluetooth buffers draining), so a key
// is retried; txKeyPolicy.h has the rules. Receive keeps running meanwhile.
static TxKeyRetryPolicy hostKeyPolicy;
static TxKeyRetryPolicy ax25KeyPolicy;
// Set by setMode(MODE_TX) when it had to go back to receive.
static bool txKeyLastAttemptRefused = false;

static void logTxKeyRefusal(const char *source, const TxKeyRetryPolicy &policy, const char *givingUp) {
  debug_log_printf(COMMAND_DEBUG_WARN,
                   "TX refused: I2S driver install failed, attempt %lu of %lu%s; source=%s dma_free=%lu "
                   "dma_largest=%lu internal_free=%lu refusals=%lu",
                   (unsigned long)policy.refusals, (unsigned long)TX_KEY_MAX_ATTEMPTS,
                   policy.gaveUp() ? givingUp : "", source, (unsigned long)txKeyHeap.dmaFree,
                   (unsigned long)txKeyHeap.dmaLargest, (unsigned long)txKeyHeap.internalFree,
                   (unsigned long)(hostKeyPolicy.refusalsTotal + ax25KeyPolicy.refusalsTotal));
}

// One gated key attempt for `policy`. Returns whether the radio is keyed.
static bool attemptKey(TxKeyRetryPolicy &policy, const char *source, const char *givingUp) {
  if (mode == MODE_TX) return true;
  uint32_t now = millis();
  if (!policy.mayAttempt(now)) return false;
  policy.attempting(now);
  setMode(MODE_TX);
  if (!txKeyLastAttemptRefused) {
    policy.accepted();
    return true;
  }
  if (policy.refused(now)) {
    logTxKeyRefusal(source, policy, givingUp);
  }
  return false;
}

void reconcileDesiredState(bool sendReport = true) {
  lastDeviceStateError = DEVICE_STATE_ERROR_NONE;
  // See txKeyPolicy.h: a PTT release drains buffered voice while keyed, a
  // filter change during that drain waits for it, and a retune unkeys first.
  TxReconcilePlan plan = planTxReconcile({
    .keyed = mode == MODE_TX,
    .pttRequested = (desiredState.flags & HOST_STATE_PTT_REQUESTED) != 0,
    .txAllowed = txAllowedByHost(),
    .retuning = (desiredState.flags & HOST_STATE_RADIO_CONFIG_VALID) && radioConfigChanged(),
    .refiltering = !filtersApplied || desiredFilterFlags() != appliedFilterFlags(),
    .drainPossible = txPlayoutCanDrain(),
  });
  if (plan.unkeyFirst) {
    setMode(rxIdleMode());
  }
  bool wantHigh = desiredState.flags & HOST_STATE_HIGH_POWER;
  if (hw.features.hasHL) {
    digitalWrite(hw.pins.pinHl, wantHigh ? LOW : HIGH);
  }
  rssiOn = desiredState.flags & HOST_STATE_RSSI_ENABLED;

  if (plan.applyFilters) {
    applyFilterFlags();
  }

  bool retuneWanted = (desiredState.flags & HOST_STATE_RADIO_CONFIG_VALID) && radioConfigChanged();
  if (retuneWanted && mode == MODE_TX && radioModuleStatus == RADIO_MODULE_FOUND) {
    // A retune while PTT is held: drop the carrier first, so group() (up to
    // 3 x 2 s if the module does not answer) never runs keyed. Key again
    // below once the new channel is applied.
    setMode(rxIdleMode());
  }
  if (retuneWanted
      && sa818Command([&] {
           drainRadioSerial();
           return sa818.group(desiredState.bw, desiredState.freq_tx, desiredState.freq_rx, desiredState.ctcss_tx, 0, 0);
         }, DEVICE_STATE_ERROR_RADIO_CONFIG_FAILED)) {
    appliedState.bw = desiredState.bw;
    appliedState.freq_tx = desiredState.freq_tx;
    appliedState.freq_rx = desiredState.freq_rx;
    appliedState.ctcss_tx = desiredState.ctcss_tx;
    appliedState.squelch = desiredState.squelch;
    softSquelchEffect.setDeadbandLevel(appliedState.squelch);
    freeDvSquelch.setLevel(appliedState.squelch);
    if (freeDv2400bEnabled()) squelched = !freeDvSquelch.open();
    softSquelchEffect.setCtcssTone(desiredState.ctcss_rx);
    appliedState.ctcss_rx = desiredState.ctcss_rx;
    appliedState.memoryId = desiredState.memoryId;
    appliedState.flags |= HOST_STATE_RADIO_CONFIG_VALID;
    radioConfigApplied = true;
  }

  // Never key a module that is missing, or one still on a channel other than
  // the one asked for (a retune that failed): end idle instead.
  bool radioReady = radioModuleStatus == RADIO_MODULE_FOUND
    && !((desiredState.flags & HOST_STATE_RADIO_CONFIG_VALID) && radioConfigChanged());
  if (plan.end != TxReconcileEnd::Idle && !radioReady) {
    plan.end = TxReconcileEnd::Idle;
  }
  switch (plan.end) {
    case TxReconcileEnd::Key:
      // A re-press while draining continues the same over.
      txPlayoutCancelDrain();
      // Gated: a refused key is retried on the policy's clock, whichever path
      // asks (txKeyRetryLoop() or another host write).
      attemptKey(hostKeyPolicy, "ptt", ", giving up until PTT is released");
      break;
    case TxReconcileEnd::Drain:
      // PTT released with voice still buffered: stay keyed until it has been
      // played; txAudioLoop() unkeys when the buffer is empty and then applies
      // any filter change that arrived meanwhile.
      txPlayoutRequestDrain();
      break;
    case TxReconcileEnd::Idle:
      hostKeyPolicy.endEpisode();
      setMode(rxIdleMode());
      // A change deferred by an earlier drain pass, now that we are unkeyed.
      applyDeferredFilters();
      break;
  }
  savePersistedRadioStateIfChanged();
  if (sendReport) {
    markDeviceStateDirty();
  }
}

// Retries a refused host key once its backoff has passed, while PTT is held.
void txKeyRetryLoop() {
  if (mode == MODE_TX || !hostKeyPolicy.retryDue(millis())) {
    return;
  }
  if ((desiredState.flags & HOST_STATE_PTT_REQUESTED) == 0) {
    hostKeyPolicy.endEpisode();
    return;
  }
  reconcileDesiredState(false);
}

// Swap I2S0 from the RX driver to the TX driver for a key-down. On failure
// (the TX DMA ring did not fit in the heap) RX is restarted and false is
// returned. Runs before `mode` changes, so a refused key is never visible as
// MODE_TX to anything, a DeviceState included, and PTT is never pulled.
static bool installTxDriverForKey() {
  endI2SRx();
#ifdef KV4P_TEST_STARVE_I2S
  // Bench-only fault injection: hold the DMA heap in 1 KB pieces so the TX
  // driver install fails the way it does when Bluetooth has the heap.
  // KV4P_TEST_STARVE_I2S attempts out of every KV4P_TEST_STARVE_I2S + 1
  // fail (a large value fails them all).
  static void *starveHog[96];
  static uint32_t starveAttempts = 0;
  size_t starveHeld = 0;
  if (starveAttempts++ % (KV4P_TEST_STARVE_I2S + 1) < KV4P_TEST_STARVE_I2S) {
    while (starveHeld < 96 && heap_caps_get_largest_free_block(MALLOC_CAP_DMA) >= 1024) {
      starveHog[starveHeld] = heap_caps_malloc(1024, MALLOC_CAP_DMA);
      if (starveHog[starveHeld] == nullptr) break;
      ++starveHeld;
    }
  }
  bool txInstalled = initI2STx();
  for (size_t i = 0; i < starveHeld; ++i) heap_caps_free(starveHog[i]);
#else
  bool txInstalled = initI2STx();
#endif
  if (!txInstalled) {
    initI2SRx();
  }
  return txInstalled;
}

void setMode(Mode newMode) {
  if (mode == newMode) {
    return;
  }
  txKeyLastAttemptRefused = false;
  if (newMode == MODE_TX && !installTxDriverForKey()) {
    txKeyLastAttemptRefused = true;
    resetDiagnosticsUpdate(false, (desiredState.flags & HOST_STATE_HIGH_POWER) != 0, 0);
    return;
  }
  freeDvRx.reset();
  freeDvTx.reset();
  bool wasTx = mode == MODE_TX;
  mode = newMode;
  markDeviceStateDirty();
  switch (mode) {
    case MODE_STOPPED:
      _LOGI("MODE_STOPPED");
      digitalWrite(hw.pins.pinPtt, HIGH);
      endI2STx();
      initI2SRx();
    break;
    case MODE_RX:
      _LOGI("MODE_RX");
      digitalWrite(hw.pins.pinPtt, HIGH);
      endI2STx();
      initI2SRx();
    break;
    case MODE_TX:
      _LOGI("MODE_TX");
      // The TX driver is already in (installTxDriverForKey()); the carrier
      // comes after it, so a refused key never pulses PTT.
      txStartTime = millis();
      txWatchDog.reset(txStartTime);
#ifndef KV4P_BENCH_NO_RF
      digitalWrite(hw.pins.pinPtt, LOW);
#endif
      txPlayoutBegin((desiredState.flags & HOST_STATE_PTT_REQUESTED) != 0);
      bleKissStream.resetRxGapStats();
      if (txPlayout.hostVoice && protocolHasBleSession() && bleKissStream.requestConnParamsIfNeeded()) {
        debug_log_printf(COMMAND_DEBUG_INFO, "BLE link: re-requested %u-%u x1.25 ms, latency 0 for TX",
                         bleKissConfig().connMinInterval, bleKissConfig().connMaxInterval);
      }
    break;
  }
  // Bookkeeping only after the transmitter is off and I2S stopped, so slow
  // log or credit writes never extend the carrier.
  if (wasTx) {
    bool hostVoice = txPlayout.hostVoice;
    txPlayoutEnd();
    if (hostVoice) {
      logTxSummary();
    }
  }
  // Record key/unkey at once: AX.25 transmits synchronously, so loop() does
  // not run while it is keyed.
  resetDiagnosticsUpdate(mode == MODE_TX, (desiredState.flags & HOST_STATE_HIGH_POWER) != 0, 0);
}

void logBleLinkInfo(const char *prefix) {
  Kv4pBleKissStream::LinkInfo link = bleKissStream.linkInfo();
  debug_log_printf(COMMAND_DEBUG_INFO,
                   "%s: connected=%u interval=%u.%02u ms latency=%u timeout=%u ms param_status=%u requests=%u "
                   "mtu=%u dle_tx=%u dle_rx=%u dle_status=%u%s",
                   prefix, link.connected ? 1 : 0, (link.connInterval * 125) / 100, (link.connInterval * 125) % 100,
                   link.connLatency, link.connTimeout * 10, link.lastParamStatus, link.connParamRequests,
                   link.mtu, link.txOctets, link.rxOctets, link.lastDataLenStatus,
                   link.dataLenReported ? "" : " (dle not reported)");
}

// One line per transmission, always on (also in release builds), so TX audio
// starvation can be judged from logs alone without listening to the air.
void logTxSummary() {
  const auto &js = txJitter.stats();
  uint32_t keyedMs = txPlayout.endMs - txPlayout.startMs;
  // Decoded voice actually written, so it is right for any host frame size.
  uint32_t audioMs = (uint32_t)(txPlayout.playedSamples48k * 1000 / AUDIO_SAMPLE_RATE);
  Kv4pBleKissStream::LinkInfo link = bleKissStream.linkInfo();
  debug_log_printf(COMMAND_DEBUG_INFO,
                   "TX summary: keyed=%lu ms frames_in=%lu bytes_in=%lu played=%lu (~%lu ms) preroll=%lu ms "
                   "reached=%s@%lu ms underruns=%lu gap_silence=%lu ms lead_silence=%lu ms max_frame_gap=%lu ms "
                   "max_ble_write_gap=%lu ms peak=%lu B (%lu ms) overflows=%lu late=%lu i2s_blocked=%lu ms discarded=%lu drain=%lu ms "
                   "ble=%s interval=%u.%02u ms latency=%u mtu=%u dle=%u/%u heap=%u min=%u credit_held=%lu B "
                   "carrier_lead=%lu ms adpcm_bad=%lu adpcm_stray=%lu adpcm_rejected=%lu ble_rx_overflow=%lu "
                   "key_dma_free=%lu key_dma_largest=%lu key_retries=%lu tx_driver_dma=%lu i2s_blocked_calls=%lu "
                   "i2s_install_failures=%lu",
                   (unsigned long)keyedMs, (unsigned long)js.framesIn, (unsigned long)js.bytesIn,
                   (unsigned long)js.framesOut, (unsigned long)audioMs, (unsigned long)txJitter.prerollMs(),
                   js.prerollReached ? "yes" : "no", (unsigned long)js.prerollReachedMs, (unsigned long)js.underruns,
                   (unsigned long)(txPlayout.gapSilenceSamplesWire * 1000UL / AUDIO_WIRE_SAMPLE_RATE),
                   (unsigned long)(txPlayout.leadSilenceSamplesWire * 1000UL / AUDIO_WIRE_SAMPLE_RATE),
                   (unsigned long)js.maxArrivalGapMs, (unsigned long)bleKissStream.rxMaxGapMs(),
                   (unsigned long)js.peakBytes, (unsigned long)txJitter.bytesToMs(js.peakBytes),
                   (unsigned long)js.overflows, (unsigned long)txPlayout.lateTicks,
                   (unsigned long)((txOutCounter.blockedUs - txPlayout.blockedUsAtStart) / 1000),
                   (unsigned long)txPlayout.framesDiscardedAtEnd, (unsigned long)txPlayout.drainMs,
                   protocolHasBleSession() ? "yes" : "no", (link.connInterval * 125) / 100,
                   (link.connInterval * 125) % 100, link.connLatency, link.mtu, link.txOctets, link.rxOctets,
                   (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap(),
                   (unsigned long)js.peakHeldAckBytes, (unsigned long)txJitter.leadMs(),
                   (unsigned long)(txAdpcmDecoder.badBlocks - txPlayout.badBlocksAtStart),
                   (unsigned long)txPlayout.strayBytes, (unsigned long)txPlayout.framesRejected,
                   (unsigned long)bleKissStream.stats().rxIncomingOverflowDrops,
                   (unsigned long)txKeyHeap.dmaFree, (unsigned long)txKeyHeap.dmaLargest,
                   (unsigned long)hostKeyPolicy.lastEpisodeRetries, (unsigned long)txDriverDmaBytes,
                   (unsigned long)i2sGuardBlockedCalls, (unsigned long)i2sGuardInstallFailures);
}

// Put the RF module's control lines in a known state before anything else.
// While the ESP32 is in reset and until setup() configures them, these pins
// float. The module's PTT is active low, so a floating PTT can leave it keyed
// (drawing up to about an amp) through the Bluetooth start-up, which is the
// heaviest load the ESP32 itself puts on the supply. That turns one reset
// into several. Releasing power-down early also gives the module more time to
// start before the handshake.
uint32_t sa818PowerUpMs = 0;

void earlyRadioPinsSetup() {
  pinMode(hw.pins.pinPtt, OUTPUT);
  digitalWrite(hw.pins.pinPtt, HIGH);  // Rx
  pinMode(hw.pins.pinPd, OUTPUT);
  digitalWrite(hw.pins.pinPd, HIGH);   // Power on
  sa818PowerUpMs = millis();
}

void setup() {
  // Pins first: resetDiagnosticsBegin() writes NVS and scans the core dump,
  // and PTT must not float meanwhile.
  boardSetup();
  earlyRadioPinsSetup();
  resetDiagnosticsBegin();
  loadPersistedRadioState();
  // Communication with Android via USB cable
  Serial.setRxBufferSize(USB_BUFFER_SIZE);
  Serial.setTxBufferSize(USB_BUFFER_SIZE);
  protocolUsbSession.windowSize = USB_BUFFER_SIZE;
  Serial.begin(115200);
  Serial.println();
  Serial.println("===== kv4p serial output =====");
  Serial.println("Firmware " FIRMWARE_LABEL " (pgpilot fork: https://github.com/simeneide/kv4p-ht)");
  Serial.println("This port will emit binary data using the kv4p protocol.");
  Serial.println("You will see watchdog resets — this is expected behavior.");
  Serial.println("Use `logcat` or a kv4p decoder to view readable logs.");
  Serial.println("More info: https://github.com/VanceVagell/kv4p-ht/blob/main/microcontroller-src/kv4p_ht_esp32_wroom_32/readme.md");
  printResetDiagnostics(Serial);
#ifdef KV4P_BENCH_NO_RF
  Serial.println("BENCH BUILD: the radio module is never keyed (KV4P_BENCH_NO_RF). Not for use.");
#endif
  Serial.println("==============================");
  char bluetoothDeviceName[12];
  formatBluetoothDeviceName(bluetoothDeviceName, sizeof(bluetoothDeviceName));
  bluetoothStarted = SerialBT.begin(bluetoothDeviceName);
  if (bluetoothStarted) {
    protocolBtSession.stream = &SerialBT;
    protocolBtSession.windowSize = USB_BUFFER_SIZE;
  } else {
    Serial.println("Classic Bluetooth init failed");
  }
  formatBluetoothDeviceName(bleKissDeviceName, sizeof(bleKissDeviceName));
  protocolBleSession.stream = &bleKissStream;
  protocolBleSession.windowSize = BLE_KISS_WINDOW_SIZE;
  if (!bleKissStream.begin()) {
    Serial.println("BLE KISS init failed");
  } else {
    Serial.print("BLE KISS advertising as ");
    Serial.println(bleKissDeviceName);
  }
  // Configure watch dog timer (WDT), which will reset the system if it gets stuck somehow.
  esp_task_wdt_init(10, true);  // Reboot if locked up for a bit
  esp_task_wdt_add(NULL);       // Add the current task to WDT watch
  buttonsSetup();
  // Set up radio module defaults (PTT and power-down: earlyRadioPinsSetup())
  pinMode(hw.pins.pinSq, INPUT);
  if (hw.features.hasHL) {
    pinMode(hw.pins.pinHl, OUTPUT);
    digitalWrite(hw.pins.pinHl, LOW);  // High power
  }
  // Communication with DRA818V radio module via GPIO pins
  Serial2.begin(9600, SERIAL_8N1, hw.pins.pinRfModuleRxd, hw.pins.pinRfModuleTxd);
  Serial2.setTimeout(10);  // Very short so we don't tie up rx audio while reading from radio module (responses are tiny so this is ok)
  //
  debugSetup();
  // Begin in STOPPED mode
  squelched = true;
  setMode(MODE_STOPPED);
  initI2SRx();
  ledSetup();
  initRadio((desiredState.flags & HOST_STATE_HIGH_POWER) != 0);
  if (radioModuleStatus == RADIO_MODULE_FOUND) {
    reconcileDesiredState(false);
  }
  setupDoneMs = millis();
  sendHello(protocolUsbSession, FIRMWARE_VER, radioModuleStatus, hw.rfModuleType, moduleMinRadioFreq(), moduleMaxRadioFreq(), getFirmwareFeatures(), currentDeviceState(protocolUsbSession.flags));
  sendResetDiagnostics(Serial);
  _LOGI("Setup is finished");
}

void initRadio(bool isHigh) {
  if (hw.rfModuleType == RF_SA818_UHF) {
    sa818 = sa818_uhf;
  } else {
    sa818 = sa818_vhf;
  }
  if (hw.features.hasHL) {
    digitalWrite(hw.pins.pinHl, isHigh ? LOW : HIGH);
  }
  radioModuleStatus = RADIO_MODULE_NOT_FOUND;
  // Up to 9 handshakes of up to 2 s each, as before (the library's handshake
  // tried 3 times, 3 rounds of it). A module still silent after 3 gets its
  // power-down pin pulsed before the next 3: modules have come up silent after
  // a reset and stayed so until the next reset.
  for (int i = 0; i < SA818_BOOT_HANDSHAKES; i++) {
    esp_task_wdt_reset();
    if (i > 0 && i % 3 == 0) {
      pulseRadioPowerDown();
    }
    if (sa818Handshake(SA818_BOOT_ANSWER_MS)) {
      radioModuleFound();
      return;
    }
  }
  debug_log_printf(COMMAND_DEBUG_WARN, "SA818 did not answer at boot; probing for it in the background");
}

// One AT+DMOCONNECT. The module answers "+DMOCONNECT:0"; anything else, or
// nothing within timeoutMs, is a miss. Same test as the DRA818 library's
// handshake (the last character before the line end is '0').
bool sa818Handshake(uint32_t timeoutMs) {
  drainRadioSerial();
  Serial2.print("AT+DMOCONNECT\r\n");
  radioBootInfo.handshakes++;
  char line[32];
  size_t len = 0;
  uint32_t start = millis();
  while ((uint32_t)(millis() - start) < timeoutMs) {
    while (Serial2.available()) {
      char c = (char)Serial2.read();
      if (c == '\r') {
        continue;
      }
      if (c == '\n') {
        line[len] = '\0';
        if (len > 0 && line[len - 1] == '0' && strstr(line, "DMOCONNECT") != nullptr) {
          return true;
        }
        len = 0;
      } else if (len < sizeof(line) - 1) {
        line[len++] = c;
      }
    }
    delay(1);
  }
  return false;
}

// Power-down low, then high again, then give the module time to start. Feeds
// the task watchdog (10 s) while it waits.
void pulseRadioPowerDown() {
  radioBootInfo.pdPulses++;
  digitalWrite(hw.pins.pinPd, LOW);
  delay(SA818_PD_LOW_MS);
  digitalWrite(hw.pins.pinPd, HIGH);
  sa818PowerUpMs = millis();
  esp_task_wdt_reset();
  delay(SA818_PD_SETTLE_MS);
  esp_task_wdt_reset();
}

void radioModuleFound() {
  radioModuleStatus = RADIO_MODULE_FOUND;
  sa818Probes = 0;
  radioBootInfo.found = true;
  radioBootInfo.present = true;
  if (radioBootInfo.readyMs == 0) {
    radioBootInfo.readyMs = max<uint32_t>(1, millis() - sa818PowerUpMs);
  }
  sa818.volume(hw.volume);
  sa818.filters(false, false, false);
}

// A module that was not found at boot, or that stopped answering, is looked
// for again here without blocking the loop for long: one handshake with a
// short timeout every few seconds, every third one after a power-down pulse.
// Once it answers, the host's configuration is applied as if it had just
// arrived, and DeviceState (radioModuleStatus) tells hosts it is back.
void radioModuleProbeLoop() {
  static uint32_t lastProbeMs = 0;
  if (radioModuleStatus == RADIO_MODULE_FOUND || mode == MODE_TX) {
    return;
  }
  uint32_t now = millis();
  if ((uint32_t)(now - lastProbeMs) < SA818_PROBE_INTERVAL_MS) {
    return;
  }
  sa818Probes++;
  if (sa818Probes % 3 == 0) {  // Never on the first probe of an episode.
    pulseRadioPowerDown();
  }
  bool found = sa818Handshake(SA818_PROBE_ANSWER_MS);
  lastProbeMs = millis();
  if (!found) {
    return;
  }
  bool firstTime = !radioBootInfo.found;
  radioModuleFound();
  if (firstTime) {
    radioBootInfo.lateFoundMs = max<uint32_t>(1, millis());
  }
  filtersApplied = false;
  radioConfigApplied = false;
  reconcileDesiredState();
  char line[160];
  if (formatBootLine(line, sizeof(line)) > 0) {
    debug_log_printf(COMMAND_DEBUG_INFO, "%s", line);
  }
}

void handleCommands(ProtocolSession &session, RcvCommand command, uint8_t *params, size_t param_len) {
  switch (command) {
    case COMMAND_HOST_TX_AUDIO:
      if (mode == MODE_TX && !freeDv2400bEnabled()) {
        processTxAudio(session, params, param_len);
        esp_task_wdt_reset();
      }
      break;
    case COMMAND_HOST_TX_DIGITAL:
      // The command ID unambiguously selects the digital path. Do not also
      // gate it on the session flag: PTT and session-state snapshots travel
      // independently and the first voice frame can win that race.
      if (mode == MODE_TX && freeDv2400bEnabled()) {
        processTxDigital(params, param_len);
        esp_task_wdt_reset();
      }
      break;
    case COMMAND_HOST_DESIRED_STATE:
      if (param_len == sizeof(HostDesiredState)) {
        HostDesiredState incomingState;
        memcpy(&incomingState, params, sizeof(HostDesiredState));
        uint16_t oldSessionFlags = session.flags;
        session.flags = incomingState.flags & HOST_STATE_SESSION_FLAG_MASK;
        bool sessionFlagsChanged = oldSessionFlags != session.flags;
        // Desired-state sequence is global across transports; hosts must sync from
        // DeviceState.appliedSequence before sending their next update.
        bool globalStateChanged = incomingState.sequence > desiredState.sequence;
        if (globalStateChanged) {
          const bool freeDvModeChanged =
              ((incomingState.flags ^ desiredState.flags) &
               HOST_STATE_FREEDV_2400B) != 0;
          desiredState = incomingState;
          desiredState.flags &= HOST_STATE_GLOBAL_FLAG_MASK;
          if (freeDvModeChanged) {
            latestRssi = 0;
            freeDvSquelch.reset();
            squelched = freeDv2400bEnabled()
                ? !freeDvSquelch.open()
                : !softSquelchEffect.isSoftOpen();
          }
        }
        if (sessionFlagsChanged || globalStateChanged) {
          reconcileDesiredState();
        }
        esp_task_wdt_reset();
      }
      break;
    case COMMAND_HOST_TX_AX25:
      if (param_len > sizeof(Ax25TxOverride) && txAllowedByHost()) {
        Ax25TxOverride txOverride;
        memcpy(&txOverride, params, sizeof(txOverride));
        size_t ax25Len = param_len - sizeof(txOverride);
        if (isModuleRadioFreq(txOverride.freqTx) && ax25Len <= AX25_MAX_KISS_DATA_LEN
            && !ax25TxScheduler.enqueue(params + sizeof(txOverride), ax25Len, &txOverride)) {
          _LOGW("AX.25 TX queue full; dropped frequency-override job");
        }
      }
      break;
  }
}

void handleAx25Data(uint8_t *ax25, size_t ax25_len) {
  if (ax25_len > 0 && ax25_len <= AX25_MAX_KISS_DATA_LEN && txAllowedByHost()) {
    if (!ax25TxScheduler.enqueue(ax25, ax25_len)) {
      _LOGW("AX.25 TX queue full; dropped KISS DATA frame");
    }
  }
}

void handleKissParameter(uint8_t command, uint8_t value) {
  if (command == KISS_CMD_TXDELAY) ax25TxScheduler.setTxDelay(value);
  else if (command == KISS_CMD_PERSIST) ax25TxScheduler.setPersist(value);
  else if (command == KISS_CMD_SLOTTIME) ax25TxScheduler.setSlotTime(value);
}

void prepareAx25TxOverrideChannel(const Ax25TxOverride &txOverride) {
  // Tune RX to the packet's target before carrier sense. The normal radio
  // configuration is deliberately marked stale so it is restored after TX.
  if (!sa818Command([&] {
        drainRadioSerial();
        return sa818.group(txOverride.bw, txOverride.freqTx, txOverride.freqTx, txOverride.ctcssTx, 0, 0);
      }, DEVICE_STATE_ERROR_RADIO_CONFIG_FAILED)) {
    return;  // The job waits in the queue until the module is back.
  }
  radioConfigApplied = false;
  ax25OverrideChannelPrepared = true;
  ax25OverrideChannelReadyAt = millis() + AX25_OVERRIDE_RX_SETTLE_MS;
}

// Android sends COMMAND_HOST_TX_AX25 for every APRS packet, including a
// beacon on the channel that is already configured. Avoid an unnecessary
// SA818 group command in that case: it would otherwise also force a second
// group command immediately after PTT is released.
bool ax25OverrideMatchesActiveChannel(const Ax25TxOverride &txOverride) {
  static constexpr float FREQ_MATCH_EPSILON_MHZ = 0.0001f;
  return radioConfigApplied
    && (desiredState.flags & HOST_STATE_RADIO_CONFIG_VALID)
    && txOverride.bw == desiredState.bw
    && txOverride.ctcssTx == desiredState.ctcss_tx
    && fabsf(txOverride.freqTx - desiredState.freq_tx) < FREQ_MATCH_EPSILON_MHZ
    && fabsf(txOverride.freqTx - desiredState.freq_rx) < FREQ_MATCH_EPSILON_MHZ;
}

void ax25TxLoop() {
  const Ax25TxJob *pendingJob = ax25TxScheduler.head();
  if (pendingJob == nullptr) return;
  if (radioModuleStatus != RADIO_MODULE_FOUND) return;
  bool receiveIdle = mode == MODE_RX || mode == MODE_STOPPED;
  if (!receiveIdle || !txAllowedByHost()) return;
  uint32_t now = millis();
  if (pendingJob->hasTxOverride && !ax25OverrideMatchesActiveChannel(pendingJob->txOverride)) {
    // A host configuration update may have restored the normal radio while
    // this job was waiting, so prepare the target channel again in that case.
    if (!ax25OverrideChannelPrepared || radioConfigApplied) {
      prepareAx25TxOverrideChannel(pendingJob->txOverride);
      return;
    }
    if ((int32_t)(now - ax25OverrideChannelReadyAt) < 0) return;
  }

  // Use SoftSQ's raw HF-noise decision for RF/voice carrier detection. Audio
  // CTCSS and UI-squelch choices must not affect CSMA channel access.
  bool ourTx = mode == MODE_TX;
  bool afskDcd = afskDemod.carrierDetected();
  bool rfCarrierDetected = softSquelchEffect.isCarrierDetected();
  bool channelBusy = ourTx || afskDcd || rfCarrierDetected;
  bool channelClear = !channelBusy;
  if (!ax25TxScheduler.ready(now, channelClear, (uint8_t)esp_random())) {
    return;
  }
  const Ax25TxJob *job = ax25TxScheduler.head();
  if (job == nullptr) return;

  if (!attemptKey(ax25KeyPolicy, "ax25", ", dropping the AX.25 frame")) {
    // Refused (i2sGuard.h): the job stays queued for the next attempt, unless
    // this job has used all of its attempts.
    if (ax25KeyPolicy.gaveUp()) {
      bool droppedOverride = job->hasTxOverride;
      ax25TxScheduler.complete();
      ax25KeyPolicy.endEpisode();
      if (droppedOverride) {
        ax25OverrideChannelPrepared = false;
        reconcileDesiredState(false);
      }
    }
    return;
  }
  latestRssi = (uint8_t)TX_AUDIO_LEVEL_FULL_SCALE_RSSI;
  txAudioLevel = TX_AUDIO_LEVEL_FULL_SCALE_RSSI;
  sendCurrentDeviceState();
  // BLE state frames are queued until bleKissLoop() runs. AFSK modulation is
  // synchronous, so flush TX state now rather than delivering it together
  // with the RX state after the packet finishes.
  if (protocolHasBleSession()
      && (protocolBleSession.flags & HOST_STATE_ENABLE_STATUS_REPORTS)) {
    bleKissStream.flush();
  }
  pulseAprsTxLED();
  bool firstFrame = true;
  bool sentOverride = job->hasTxOverride;
  do {
    const Ax25TxJob *nextJob = ax25TxScheduler.next();
    bool batchNextStandardFrame = !job->hasTxOverride && nextJob != nullptr && !nextJob->hasTxOverride;
    processTxAx25(job->data, job->len, firstFrame ? ax25TxScheduler.txDelayMs() : 0,
                  batchNextStandardFrame ? 0 : TX_AFSK_TAIL_SILENCE_MS);
    ax25TxScheduler.complete();
    if (!batchNextStandardFrame) break;
    job = ax25TxScheduler.head();
    firstFrame = false;
  } while (job != nullptr);
  ax25KeyPolicy.endEpisode();
  setMode(rxIdleMode());
  if (sentOverride) ax25OverrideChannelPrepared = false;
  // Apply the latest desired normal configuration only after PTT is released.
  reconcileDesiredState(false);
  sendCurrentDeviceState();
  esp_task_wdt_reset();
}

void rssiLoop() {
  if (rssiOn) {
    EVERY_N_MILLISECONDS(RSSI_REPORT_INTERVAL_MS) {
      if (mode == MODE_TX) {
        uint8_t rssi = (uint8_t)roundf(constrain(txAudioLevel, 0.0f, 255.0f));
        if (latestRssi != rssi) {
          latestRssi = rssi;
          markDeviceStateDirty();
        }
      } else if (mode == MODE_RX && !freeDv2400bEnabled() && radioModuleStatus == RADIO_MODULE_FOUND) {
        // TODO fix the dra818 library's implementation of rssi(). Right now it just drops the
        // return value from the module, and just tells us success/fail.
        // int rssi = dra->rssi();
        Serial2.println("RSSI?");
        String rssiResponse = Serial2.readString();
        if (rssiResponse.length() > 7) {
          String rssiStr = rssiResponse.substring(5);
          int rssiInt    = rssiStr.toInt();
          if (rssiInt >= 0 && rssiInt <= 255) {
            uint8_t rssi = (uint8_t)rssiInt;
            if (latestRssi != rssi) {
              latestRssi = rssi;
              markDeviceStateDirty();
            }
          }
        }
      }
    }
    END_EVERY_N_MILLISECONDS();
  }
}

void deviceStateLoop() {
  if (!protocolAnySessionFlag(HOST_STATE_ENABLE_STATUS_REPORTS)) {
    return;
  }

  bool sent = false;
  if (deviceStateDirty) {
    sendCurrentDeviceState();
    sent = true;
  }
  EVERY_N_MILLISECONDS(DEVICE_STATE_REPORT_INTERVAL_MS) {
    if (!sent) {
      sendCurrentDeviceState();
    }
  }
  END_EVERY_N_MILLISECONDS();
}

void bluetoothLoop() {
  if (!bluetoothStarted) {
    return;
  }

  bool connected = SerialBT.hasClient();
  if (connected && !bluetoothProtocolConnected) {
    bluetoothProtocolConnected = true;
    protocolBtSession.connected = true;
    bluetoothParser.reset();
    sendHello(protocolBtSession, FIRMWARE_VER, radioModuleStatus, hw.rfModuleType, moduleMinRadioFreq(), moduleMaxRadioFreq(), getFirmwareFeatures(), currentDeviceState(protocolBtSession.flags));
    sendResetDiagnostics(*protocolBtSession.stream);
    sendResetDiagnostics(Serial);  // Also on USB, for bench logging.
  } else if (!connected && bluetoothProtocolConnected) {
    bluetoothProtocolConnected = false;
    protocolBtSession.connected = false;
    txPlayoutForgetSession(protocolBtSession);
    uint16_t oldSessionFlags = protocolBtSession.flags;
    protocolBtSession.flags = 0;
    bluetoothParser.reset();
    if (oldSessionFlags != 0) {
      reconcileDesiredState();
    }
  }

  if (bluetoothProtocolConnected) {
    bluetoothParser.loop();
  }
}

void bleKissLoop() {
  bleKissStream.loop();
  bool connected = bleKissStream.isConnected();
  bool protocolReady = bleKissStream.canSend();
  if (protocolReady && !bleKissProtocolConnected) {
    bleKissParser.reset();
    sendHello(protocolBleSession, FIRMWARE_VER, radioModuleStatus, hw.rfModuleType, moduleMinRadioFreq(), moduleMaxRadioFreq(), getFirmwareFeatures(), currentDeviceState(protocolBleSession.flags));
    sendResetDiagnostics(bleKissStream);
    sendResetDiagnostics(Serial);  // Also on USB, for bench logging.
    bleKissProtocolConnected = true;
    protocolBleSession.connected = true;
    _LOGI("BLE KISS sent HELLO after notify subscription: firmware=%u window=%u", FIRMWARE_VER, BLE_KISS_WINDOW_SIZE);
  } else if ((!connected || !protocolReady) && bleKissProtocolConnected) {
    bleKissProtocolConnected = false;
    protocolBleSession.connected = false;
    txPlayoutForgetSession(protocolBleSession);
    uint16_t oldSessionFlags = protocolBleSession.flags;
    protocolBleSession.flags = 0;
    bleKissParser.reset();
    if (oldSessionFlags != 0) {
      reconcileDesiredState();
    }
  }

  if (bleKissProtocolConnected) {
    bleKissParser.loop();
  }

  // Log every change of connection parameters, MTU or data length. The values
  // are recorded on the Bluetooth task and only logged here, on the main loop.
  static uint32_t loggedLinkGeneration = 0;
  Kv4pBleKissStream::LinkInfo link = bleKissStream.linkInfo();
  if (link.generation != loggedLinkGeneration) {
    loggedLinkGeneration = link.generation;
    logBleLinkInfo("BLE link");
  }
}

void squelchLoop() {
  freeDvSquelchLoop();
  if (freeDv2400bEnabled()) return;
  bool nextSquelched = !softSquelchEffect.isSoftOpen();
  if (nextSquelched != squelched) {
    squelched = nextSquelched;
    markDeviceStateDirty();
  }
}

void loop() {
  resetDiagnosticsUpdate(mode == MODE_TX, (desiredState.flags & HOST_STATE_HIGH_POWER) != 0, millis() - txStartTime);
  squelchLoop();
  debugLoop();
  ledLoop();
  buttonsLoop();
  protocolLoop();
  bluetoothLoop();
  bleKissLoop();
  rxAudioLoop();
  ax25TxLoop();
  txAudioLoop();
  txKeyRetryLoop();
  rssiLoop();
  radioModuleProbeLoop();
  deviceStateLoop();
}
