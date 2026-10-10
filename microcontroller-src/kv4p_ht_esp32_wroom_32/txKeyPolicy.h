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
