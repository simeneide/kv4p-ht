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

#include <stdint.h>

// What one pass of reconcileDesiredState() does with the transmitter.
//
// Pure so it can be unit tested natively. The rules:
//  - PTT requested and TX allowed: key (or stay keyed; a release in progress
//    is cancelled, so a re-press during the drain continues the same over).
//  - PTT released with voice still buffered: stay keyed until it has played.
//  - A filter or emphasis change that arrives while draining waits for the
//    drain and is applied after unkey. Hosts commonly release PTT and reset
//    emphasis in two separate writes; refusing the second under carrier used
//    to unkey at once and throw the buffered tail of every over away.
//  - A retune, or TX no longer allowed, unkeys before anything else: the
//    SA818 is never sent a group command under a carrier it was asked to drop.
//  - While PTT is held, configuration is applied as it arrives (unchanged
//    upstream behaviour).
struct TxReconcileInput {
  bool keyed;          // Currently transmitting (mode == MODE_TX).
  bool pttRequested;   // Host PTT bit set.
  bool txAllowed;      // Host TX_ALLOWED bit set.
  bool retuning;       // Frequency, bandwidth, tones or squelch differ from applied.
  bool refiltering;    // Pre-emphasis or audio filters differ from applied.
  bool drainPossible;  // Analog voice is still buffered or in the DMA ring.
};

enum class TxReconcileEnd {
  Key,    // Key, or stay keyed for the held PTT.
  Drain,  // Stay keyed until the buffered audio has played, then unkey.
  Idle,   // Not transmitting at the end of this pass.
};

struct TxReconcilePlan {
  bool unkeyFirst;    // Drop the carrier before touching the SA818.
  bool applyFilters;  // Apply a filter change now; false with refiltering means deferred until unkey.
  TxReconcileEnd end;
};

inline TxReconcilePlan planTxReconcile(const TxReconcileInput &in) {
  bool keepKeyed = in.pttRequested && in.txAllowed;
  bool drainAbsorbsChange = in.keyed && !keepKeyed && in.txAllowed && in.drainPossible && !in.retuning;
  TxReconcilePlan plan{};
  plan.unkeyFirst = in.keyed && !keepKeyed && (in.retuning || in.refiltering) && !drainAbsorbsChange;
  bool keyedAfterUnkey = in.keyed && !plan.unkeyFirst;
  plan.applyFilters = in.refiltering && (!keyedAfterUnkey || keepKeyed);
  if (keepKeyed) {
    plan.end = TxReconcileEnd::Key;
  } else if (keyedAfterUnkey && in.txAllowed && in.drainPossible) {
    plan.end = TxReconcileEnd::Drain;
  } else {
    plan.end = TxReconcileEnd::Idle;
  }
  return plan;
}

// Key attempts that the firmware refused because the TX audio driver could not
// be installed (i2sGuard.h). One instance per source of keys: the host's PTT
// and the AX.25 scheduler. An episode is one press, or one AX.25 job.
//
//  - The first attempt of an episode is never delayed.
//  - After a refusal the next attempt waits TX_KEY_RETRY_MS from the last
//    attempt, whoever asks for it (a host state write, a reconnect, a probe or
//    the retry loop): every path goes through mayAttempt().
//  - After TX_KEY_MAX_ATTEMPTS refusals the episode gives up until it ends.
//  - A refusal is logged only at the first and at giving up.
static constexpr uint32_t TX_KEY_RETRY_MS = 250;
static constexpr uint32_t TX_KEY_MAX_ATTEMPTS = 8;

struct TxKeyRetryPolicy {
  uint32_t lastAttemptMs = 0;
  uint32_t refusals = 0;            // In this episode.
  uint32_t refusalsTotal = 0;       // Since boot.
  uint32_t lastEpisodeRetries = 0;  // Refusals before the last accepted key.

  bool gaveUp() const { return refusals >= TX_KEY_MAX_ATTEMPTS; }

  bool mayAttempt(uint32_t nowMs) const {
    if (refusals == 0) return true;
    if (gaveUp()) return false;
    return (uint32_t)(nowMs - lastAttemptMs) >= TX_KEY_RETRY_MS;
  }

  // A retry is owed: refused, not given up, and the backoff has passed.
  bool retryDue(uint32_t nowMs) const { return refusals != 0 && mayAttempt(nowMs); }

  void attempting(uint32_t nowMs) { lastAttemptMs = nowMs; }

  // Returns whether this refusal should be logged.
  bool refused(uint32_t nowMs) {
    lastAttemptMs = nowMs;
    ++refusals;
    ++refusalsTotal;
    return refusals == 1 || refusals == TX_KEY_MAX_ATTEMPTS;
  }

  void accepted() {
    lastEpisodeRetries = refusals;
    refusals = 0;
  }

  // The press was released, or the job is finished or dropped.
  void endEpisode() { refusals = 0; }
};
