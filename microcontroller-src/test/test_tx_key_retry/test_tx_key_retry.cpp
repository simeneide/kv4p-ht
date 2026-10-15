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

#include "txKeyPolicy.h"

void setUp() {}
void tearDown() {}

void test_first_attempt_is_never_delayed() {
  TxKeyRetryPolicy p;
  TEST_ASSERT_TRUE(p.mayAttempt(0));
  TEST_ASSERT_FALSE(p.retryDue(0));
}

void test_refusal_backs_off_from_the_last_attempt() {
  TxKeyRetryPolicy p;
  p.attempting(1000);
  TEST_ASSERT_TRUE(p.refused(1000));
  TEST_ASSERT_FALSE(p.mayAttempt(1000 + TX_KEY_RETRY_MS - 1));
  TEST_ASSERT_FALSE(p.retryDue(1000 + TX_KEY_RETRY_MS - 1));
  TEST_ASSERT_TRUE(p.mayAttempt(1000 + TX_KEY_RETRY_MS));
  TEST_ASSERT_TRUE(p.retryDue(1000 + TX_KEY_RETRY_MS));
}

// A host state write between retries is gated like the retry loop: however
// often anything asks, attempts stay TX_KEY_RETRY_MS apart.
void test_frequent_callers_cannot_exceed_the_retry_rate() {
  TxKeyRetryPolicy p;
  uint32_t attempts = 0;
  for (uint32_t now = 0; now < 1000; now += 10) {
    if (!p.mayAttempt(now)) continue;
    p.attempting(now);
    ++attempts;
    p.refused(now);
  }
  TEST_ASSERT_EQUAL_UINT32(4, attempts);  // 0, 250, 500, 750
}

void test_gives_up_after_the_cap_and_logs_only_first_and_last() {
  TxKeyRetryPolicy p;
  uint32_t logged = 0;
  uint32_t now = 0;
  for (uint32_t i = 0; i < TX_KEY_MAX_ATTEMPTS; ++i) {
    TEST_ASSERT_TRUE(p.mayAttempt(now));
    p.attempting(now);
    if (p.refused(now)) ++logged;
    now += TX_KEY_RETRY_MS;
  }
  TEST_ASSERT_EQUAL_UINT32(2, logged);
  TEST_ASSERT_TRUE(p.gaveUp());
  TEST_ASSERT_FALSE(p.mayAttempt(now + 100000));
  TEST_ASSERT_FALSE(p.retryDue(now + 100000));
}

void test_episode_end_resets_the_cap_but_not_the_total() {
  TxKeyRetryPolicy p;
  for (uint32_t i = 0; i < TX_KEY_MAX_ATTEMPTS; ++i) p.refused(i * TX_KEY_RETRY_MS);
  p.endEpisode();
  TEST_ASSERT_FALSE(p.gaveUp());
  TEST_ASSERT_TRUE(p.mayAttempt(0));
  TEST_ASSERT_FALSE(p.retryDue(0));
  TEST_ASSERT_EQUAL_UINT32(TX_KEY_MAX_ATTEMPTS, p.refusalsTotal);
}

void test_accept_records_the_retries_and_clears_the_episode() {
  TxKeyRetryPolicy p;
  p.refused(0);
  p.refused(250);
  p.accepted();
  TEST_ASSERT_EQUAL_UINT32(2, p.lastEpisodeRetries);
  TEST_ASSERT_EQUAL_UINT32(0, p.refusals);
  TEST_ASSERT_FALSE(p.retryDue(1000));
}

void test_backoff_survives_millis_wrap() {
  TxKeyRetryPolicy p;
  p.refused(0xFFFFFFF0u);
  TEST_ASSERT_FALSE(p.mayAttempt(0xFFFFFFF0u + 100));  // wrapped to 0x54
  TEST_ASSERT_TRUE(p.mayAttempt(0xFFFFFFF0u + TX_KEY_RETRY_MS));
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_first_attempt_is_never_delayed);
  RUN_TEST(test_refusal_backs_off_from_the_last_attempt);
  RUN_TEST(test_frequent_callers_cannot_exceed_the_retry_rate);
  RUN_TEST(test_gives_up_after_the_cap_and_logs_only_first_and_last);
  RUN_TEST(test_episode_end_resets_the_cap_but_not_the_total);
  RUN_TEST(test_accept_records_the_retries_and_clears_the_episode);
  RUN_TEST(test_backoff_survives_millis_wrap);
  return UNITY_END();
}
