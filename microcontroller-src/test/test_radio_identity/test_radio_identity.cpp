/*
KV4P-HT (see http://kv4p.com), pgpilot fork
Copyright (C) 2026 Simen Eide

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

#include <string.h>

#include "radioIdentity.h"

void setUp() {}
void tearDown() {}

// ESP.getEfuseMac() layout: MAC byte i in bits 8i..8i+7.
static uint64_t efuse(const uint8_t m[6]) {
  uint64_t v = 0;
  for (int i = 0; i < 6; i++) {
    v |= (uint64_t)m[i] << (8 * i);
  }
  return v;
}

// Two bench radios whose Bluetooth names and Android addresses are known.
void test_id_is_the_advertised_name_suffix() {
  const uint8_t dino[6] = {0x28, 0x56, 0x2F, 0x8D, 0xBC, 0xF0};
  const uint8_t wally[6] = {0x94, 0x51, 0xDC, 0x16, 0x6C, 0xF4};
  TEST_ASSERT_EQUAL_HEX32(0xF0BC8D, radioIdFromEfuseMac(efuse(dino)));
  TEST_ASSERT_EQUAL_HEX32(0xF46C16, radioIdFromEfuseMac(efuse(wally)));
}

void test_line_names_id_mac_and_bt_address() {
  const uint8_t mac[6] = {0x28, 0x56, 0x2F, 0x8D, 0xBC, 0xF0};
  const uint8_t bt[6] = {0x28, 0x56, 0x2F, 0x8D, 0xBC, 0xF2};
  char line[96];
  int n = formatRadioIdLine(line, sizeof(line), efuse(mac), bt);
  TEST_ASSERT_EQUAL_STRING("Radio id: kv4p-F0BC8D mac=28:56:2F:8D:BC:F0 bt=28:56:2F:8D:BC:F2", line);
  TEST_ASSERT_EQUAL_INT((int)strlen(line), n);
}

void test_small_ids_keep_six_digits() {
  const uint8_t mac[6] = {0x00, 0x01, 0x02, 0x03, 0x00, 0x00};
  const uint8_t bt[6] = {0, 1, 2, 3, 0, 2};
  char line[96];
  formatRadioIdLine(line, sizeof(line), efuse(mac), bt);
  TEST_ASSERT_EQUAL_STRING("Radio id: kv4p-000003 mac=00:01:02:03:00:00 bt=00:01:02:03:00:02", line);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_id_is_the_advertised_name_suffix);
  RUN_TEST(test_line_names_id_mac_and_bt_address);
  RUN_TEST(test_small_ids_keep_six_digits);
  return UNITY_END();
}
