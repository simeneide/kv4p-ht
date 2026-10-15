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

// FIRMWARE_VER is the number sent in HELLO; hosts compare it numerically.
// FIRMWARE_LABEL is for people: it names the build in the USB boot banner and
// in the "Reset reason" debug line, so a log says which firmware produced it.
// This is the pgpilot fork (branch pgpilot/ble-robust). Keep the label's
// leading number equal to FIRMWARE_VER. tools/set_app_version.py reads the
// label from this file and writes it into the ESP-IDF app descriptor, which
// `esptool.py image_info` and flashers read.
const uint16_t FIRMWARE_VER = 20;
#define FIRMWARE_LABEL "20 pgpilot"
