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

// Who this radio is, the same on every transport. Over Bluetooth a host
// already knows it from the advertised name ("kv4p-XXXXXX"); over USB it
// otherwise has no way to tell one radio from another, so a host that
// remembers a Bluetooth radio would take whatever is on the cable for it.
//
// The line, sent as COMMAND_DEBUG_INFO (in release builds too):
//
//   Radio id: kv4p-F0BC8D mac=28:56:2F:8D:BC:F0 bt=28:56:2F:8D:BC:F2
//
// The id is exactly the advertised name. mac= is the factory (efuse) MAC the
// id is derived from, bt= the Bluetooth MAC, the address an Android host
// sees on the radio's advertisements. Pure, so the native tests cover it.

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

// The low three bytes of the factory MAC, last byte first: the hex suffix of
// the advertised name. ESP.getEfuseMac() holds MAC byte i in bits 8i..8i+7.
inline uint32_t radioIdFromEfuseMac(uint64_t efuseMac) {
  return (uint32_t)((efuseMac >> 24) & 0xFFFFFF);
}

inline int formatRadioIdLine(char *out, size_t outSize, uint64_t efuseMac, const uint8_t bt[6]) {
  uint8_t m[6];
  for (int i = 0; i < 6; i++) {
    m[i] = (uint8_t)((efuseMac >> (8 * i)) & 0xFF);
  }
  return snprintf(out, outSize,
                  "Radio id: kv4p-%06X mac=%02X:%02X:%02X:%02X:%02X:%02X bt=%02X:%02X:%02X:%02X:%02X:%02X",
                  (unsigned)radioIdFromEfuseMac(efuseMac), m[0], m[1], m[2], m[3], m[4], m[5],
                  bt[0], bt[1], bt[2], bt[3], bt[4], bt[5]);
}
