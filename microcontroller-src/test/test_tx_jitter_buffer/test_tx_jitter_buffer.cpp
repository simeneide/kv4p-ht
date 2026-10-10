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
#include <unity.h>

#include <vector>

#include "txJitterBuffer.h"

// 16 kHz IMA ADPCM: one 128-byte block is 249 samples, ~15.56 ms.
using Buffer = TxJitterBuffer<1024, 16>;
static constexpr uint32_t PREROLL_MS = 100;
static const int SESSION_A = 0;
static const int SESSION_B = 1;

static Buffer makeBuffer() {
  Buffer b(16000, 249, 128);
  b.reset(0, PREROLL_MS);
  return b;
}

static std::vector<uint8_t> block(uint8_t fill) {
  return std::vector<uint8_t>(128, fill);
}

static size_t popInto(Buffer &b, std::vector<uint8_t> &out, Buffer::Entry &meta) {
  return b.pop([&out](const uint8_t *data, size_t len) { out.insert(out.end(), data, data + len); }, meta);
}

void test_buffered_ms_counts_adpcm_blocks() {
  Buffer b = makeBuffer();
  auto f = block(1);
  TEST_ASSERT_TRUE(b.push(f.data(), f.size(), &SESSION_A, 137, 0));
  TEST_ASSERT_EQUAL_UINT32(15, b.bufferedMs());
  TEST_ASSERT_EQUAL_UINT32(137, b.heldAckBytes());
}

void test_holds_until_preroll_level_reached() {
  Buffer b = makeBuffer();
  auto f = block(1);
  // Six blocks are ~93 ms: below the 100 ms target, and none of them is old.
  for (int i = 0; i < 6; i++) {
    TEST_ASSERT_TRUE(b.push(f.data(), f.size(), &SESSION_A, 137, 10));
  }
  TEST_ASSERT_FALSE(b.shouldPlay(20, false, false));
  TEST_ASSERT_TRUE(b.push(f.data(), f.size(), &SESSION_A, 137, 20));
  TEST_ASSERT_TRUE(b.shouldPlay(20, false, false));
  TEST_ASSERT_TRUE(b.stats().prerollReached);
  TEST_ASSERT_EQUAL_UINT32(20, b.stats().prerollReachedMs);
}

void test_starts_when_first_frame_is_preroll_old() {
  Buffer b = makeBuffer();
  auto f = block(1);
  TEST_ASSERT_TRUE(b.push(f.data(), f.size(), &SESSION_A, 137, 50));
  TEST_ASSERT_FALSE(b.shouldPlay(50 + PREROLL_MS - 1, false, false));
  TEST_ASSERT_TRUE(b.shouldPlay(50 + PREROLL_MS, false, false));
}

void test_credit_pressure_and_drain_start_playout_early() {
  Buffer b = makeBuffer();
  auto f = block(1);
  TEST_ASSERT_TRUE(b.push(f.data(), f.size(), &SESSION_A, 137, 0));
  TEST_ASSERT_TRUE(b.shouldPlay(1, true, false));

  Buffer d = makeBuffer();
  TEST_ASSERT_TRUE(d.push(f.data(), f.size(), &SESSION_A, 137, 0));
  TEST_ASSERT_TRUE(d.shouldPlay(1, false, true));
}

void test_nothing_to_play_when_empty() {
  Buffer b = makeBuffer();
  TEST_ASSERT_FALSE(b.shouldPlay(1000, true, true));
  b.noteEmptySlot();
  TEST_ASSERT_EQUAL_UINT32(0, b.stats().underruns);
}

void test_underrun_returns_to_preroll() {
  Buffer b = makeBuffer();
  auto f = block(1);
  for (int i = 0; i < 7; i++) {
    b.push(f.data(), f.size(), &SESSION_A, 137, 0);
  }
  TEST_ASSERT_TRUE(b.shouldPlay(0, false, false));
  Buffer::Entry meta;
  while (!b.empty()) {
    b.discard(meta);
  }
  b.noteEmptySlot();
  TEST_ASSERT_EQUAL_UINT32(1, b.stats().underruns);
  TEST_ASSERT_FALSE(b.playing());

  // A single late frame is not played immediately: it waits for a new pre-roll.
  b.push(f.data(), f.size(), &SESSION_A, 137, 500);
  TEST_ASSERT_FALSE(b.shouldPlay(510, false, false));
  TEST_ASSERT_TRUE(b.shouldPlay(500 + PREROLL_MS, false, false));
  // The first pre-roll time is kept for the summary.
  TEST_ASSERT_EQUAL_UINT32(0, b.stats().prerollReachedMs);
}

void test_pop_preserves_bytes_order_and_ack_across_wrap() {
  Buffer b = makeBuffer();
  std::vector<uint8_t> out;
  Buffer::Entry meta;
  // Move the ring position so the next frames wrap around the end.
  for (int i = 0; i < 7; i++) {
    auto f = block((uint8_t)i);
    TEST_ASSERT_TRUE(b.push(f.data(), f.size(), &SESSION_A, 100, 0));
    out.clear();
    TEST_ASSERT_EQUAL(128, popInto(b, out, meta));
  }
  std::vector<uint8_t> big(300);
  for (size_t i = 0; i < big.size(); i++) big[i] = (uint8_t)(i * 7);
  TEST_ASSERT_TRUE(b.push(big.data(), big.size(), &SESSION_B, 321, 0));
  out.clear();
  TEST_ASSERT_EQUAL(300, popInto(b, out, meta));
  TEST_ASSERT_EQUAL_UINT8_ARRAY(big.data(), out.data(), big.size());
  TEST_ASSERT_EQUAL_PTR(&SESSION_B, meta.ackTag);
  TEST_ASSERT_EQUAL_UINT16(321, meta.ackBytes);
  TEST_ASSERT_EQUAL_UINT32(0, b.heldAckBytes());
}

void test_push_fails_when_full_and_counts_overflow() {
  Buffer b = makeBuffer();
  auto f = block(1);
  for (int i = 0; i < 8; i++) {
    TEST_ASSERT_TRUE(b.push(f.data(), f.size(), &SESSION_A, 137, 0));
  }
  TEST_ASSERT_FALSE(b.push(f.data(), f.size(), &SESSION_A, 137, 0));
  TEST_ASSERT_EQUAL_UINT32(1, b.stats().overflows);
  TEST_ASSERT_EQUAL_UINT32(1024, b.stats().peakBytes);
  Buffer::Entry meta;
  b.discard(meta);
  TEST_ASSERT_TRUE(b.push(f.data(), f.size(), &SESSION_A, 137, 0));
}

void test_entry_limit_is_enforced() {
  Buffer b = makeBuffer();
  uint8_t one = 0;
  for (int i = 0; i < 16; i++) {
    TEST_ASSERT_TRUE(b.push(&one, 1, &SESSION_A, 10, 0));
  }
  TEST_ASSERT_FALSE(b.push(&one, 1, &SESSION_A, 10, 0));
}

void test_forget_ack_tag_drops_credits_but_keeps_audio() {
  Buffer b = makeBuffer();
  auto f = block(1);
  b.push(f.data(), f.size(), &SESSION_A, 137, 0);
  b.push(f.data(), f.size(), &SESSION_B, 140, 0);
  b.forgetAckTag(&SESSION_A);
  TEST_ASSERT_EQUAL_UINT32(140, b.heldAckBytes());
  TEST_ASSERT_EQUAL(2, b.entries());
  Buffer::Entry meta;
  b.discard(meta);
  TEST_ASSERT_NULL(meta.ackTag);
  TEST_ASSERT_EQUAL_UINT16(0, meta.ackBytes);
}

void test_held_credit_is_tracked_per_session() {
  Buffer b = makeBuffer();
  auto f = block(1);
  b.push(f.data(), f.size(), &SESSION_A, 137, 0);
  b.push(f.data(), f.size(), &SESSION_B, 140, 0);
  b.push(f.data(), f.size(), &SESSION_A, 137, 0);
  TEST_ASSERT_EQUAL_UINT32(274, b.heldAckBytesFor(&SESSION_A));
  TEST_ASSERT_EQUAL_UINT32(140, b.heldAckBytesFor(&SESSION_B));
  TEST_ASSERT_EQUAL_UINT32(414, b.heldAckBytes());
}

void test_arrival_gap_and_peak_are_tracked() {
  Buffer b = makeBuffer();
  auto f = block(1);
  b.push(f.data(), f.size(), &SESSION_A, 137, 100);
  b.push(f.data(), f.size(), &SESSION_A, 137, 116);
  b.push(f.data(), f.size(), &SESSION_A, 137, 400);
  TEST_ASSERT_EQUAL_UINT32(284, b.stats().maxArrivalGapMs);
  TEST_ASSERT_EQUAL_UINT32(384, b.stats().peakBytes);
  TEST_ASSERT_EQUAL_UINT32(3, b.stats().framesIn);
}

void test_reset_clears_state() {
  Buffer b = makeBuffer();
  auto f = block(1);
  b.push(f.data(), f.size(), &SESSION_A, 137, 0);
  b.reset(1000, 300);
  TEST_ASSERT_TRUE(b.empty());
  TEST_ASSERT_EQUAL_UINT32(0, b.heldAckBytes());
  TEST_ASSERT_EQUAL_UINT32(300, b.prerollMs());
  TEST_ASSERT_EQUAL_UINT32(0, b.stats().framesIn);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_buffered_ms_counts_adpcm_blocks);
  RUN_TEST(test_holds_until_preroll_level_reached);
  RUN_TEST(test_starts_when_first_frame_is_preroll_old);
  RUN_TEST(test_credit_pressure_and_drain_start_playout_early);
  RUN_TEST(test_nothing_to_play_when_empty);
  RUN_TEST(test_underrun_returns_to_preroll);
  RUN_TEST(test_pop_preserves_bytes_order_and_ack_across_wrap);
  RUN_TEST(test_push_fails_when_full_and_counts_overflow);
  RUN_TEST(test_entry_limit_is_enforced);
  RUN_TEST(test_forget_ack_tag_drops_credits_but_keeps_audio);
  RUN_TEST(test_held_credit_is_tracked_per_session);
  RUN_TEST(test_arrival_gap_and_peak_are_tracked);
  RUN_TEST(test_reset_clears_state);
  return UNITY_END();
}
