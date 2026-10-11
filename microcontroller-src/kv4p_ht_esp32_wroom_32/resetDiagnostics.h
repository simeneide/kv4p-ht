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

// Why did the radio restart?
//
// Field reports of "the radio rebooted while transmitting" can be a panic
// (software reset after a crash), a watchdog, or a brownout. These look the
// same to the user but have different fixes, so this records and reports:
//  - esp_reset_reason() for this boot,
//  - a breadcrumb kept in RTC memory across software resets: whether the
//    radio was keyed (and at which power) when the previous boot ended, and
//    how long it had been up,
//  - a summary of the core dump in flash, if one exists (task, PC, cause,
//    faulting address, backtrace), so a crash can be symbolicated without
//    first reading the flash over USB.
//
// The report is printed as plain text on the USB serial port at boot (before
// any kv4p framing, next to the existing banner), and sent as ordinary
// COMMAND_DEBUG_INFO text frames to each host session after HELLO. Debug frames
// already exist in the protocol, so hosts that do not care simply log or drop
// them; nothing in HELLO or DeviceState changes.

#include <Arduino.h>
#include <esp_system.h>
#include <esp_attr.h>
#include <esp_core_dump.h>
#include "protocol.h"
#include "firmwareVersion.h"

struct ResetBreadcrumb {
  uint32_t magic;
  uint32_t bootCount;
  uint32_t uptimeMs;     // Last recorded uptime of the previous boot.
  uint8_t keyed;         // 1 if PTT was keyed at the last update.
  uint8_t highPower;     // 1 if high power was selected at the last update.
  uint16_t reserved;
  uint32_t keyedForMs;   // How long the current transmission had lasted.
  uint32_t check;        // Detects a breadcrumb torn by a reset mid-update.
};

static constexpr uint32_t RESET_BREADCRUMB_MAGIC = 0x4B563450; // "KV4P"
static constexpr uint32_t RESET_BREADCRUMB_UPDATE_MS = 100;

RTC_NOINIT_ATTR ResetBreadcrumb resetBreadcrumb;

// Snapshot of the previous boot, captured before this boot overwrites it.
ResetBreadcrumb previousBreadcrumb = {};
bool previousBreadcrumbValid = false;
esp_reset_reason_t bootResetReason = ESP_RST_UNKNOWN;
// Formatted once at boot: reading and checking the dump scans up to 64 KB of
// flash, which must not happen on every host connect.
char *coreDumpLine = nullptr;

uint32_t resetBreadcrumbCheck(const ResetBreadcrumb &b) {
  return (b.magic ^ b.bootCount ^ (b.uptimeMs * 31) ^ ((uint32_t)b.keyed << 8) ^ ((uint32_t)b.highPower << 16)
          ^ (b.keyedForMs * 7)) + 0x9E3779B9;
}

int formatCoreDumpLine(char *out, size_t outSize);

const char *resetReasonName(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT";
    case ESP_RST_SW:        return "SW";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "UNKNOWN";
  }
}

const char *resetReasonDescription(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON:   return "Power On Reset";
    case ESP_RST_EXT:       return "External Pin Reset";
    case ESP_RST_SW:        return "Software Reset";
    case ESP_RST_PANIC:     return "Exception/Panic Reset";
    case ESP_RST_INT_WDT:   return "Interrupt Watchdog Reset";
    case ESP_RST_TASK_WDT:  return "Task Watchdog Reset";
    case ESP_RST_WDT:       return "Other Watchdog Reset";
    case ESP_RST_DEEPSLEEP: return "Deep Sleep Reset";
    case ESP_RST_BROWNOUT:  return "Brownout Reset";
    case ESP_RST_SDIO:      return "SDIO Reset";
    default:                return "Unknown";
  }
}

// Call first thing in setup().
void resetDiagnosticsBegin() {
  bootResetReason = esp_reset_reason();
  // RTC slow memory survives software resets, panics and watchdogs, and
  // usually a brownout (the RTC domain keeps its contents unless the supply
  // collapses), but not a power-on. The magic and check word reject garbage.
  previousBreadcrumbValid = bootResetReason != ESP_RST_POWERON
    && resetBreadcrumb.magic == RESET_BREADCRUMB_MAGIC
    && resetBreadcrumb.check == resetBreadcrumbCheck(resetBreadcrumb);
  if (previousBreadcrumbValid) {
    previousBreadcrumb = resetBreadcrumb;
  }
  uint32_t bootCount = previousBreadcrumbValid ? previousBreadcrumb.bootCount + 1 : 1;
  resetBreadcrumb = {};
  resetBreadcrumb.magic = RESET_BREADCRUMB_MAGIC;
  resetBreadcrumb.bootCount = bootCount;
  resetBreadcrumb.check = resetBreadcrumbCheck(resetBreadcrumb);

  char line[400];
  int n = formatCoreDumpLine(line, sizeof(line));
  if (n > 0) {
    coreDumpLine = strdup(line);
  }
}

// Call from loop() and on every key/unkey; cheap, writes RTC memory at most
// every 100 ms unless the keyed state changed.
void resetDiagnosticsUpdate(bool keyed, bool highPower, uint32_t keyedForMs) {
  static uint32_t lastUpdateMs = 0;
  uint32_t now = millis();
  bool keyedChanged = resetBreadcrumb.keyed != (keyed ? 1 : 0);
  if (!keyedChanged && (uint32_t)(now - lastUpdateMs) < RESET_BREADCRUMB_UPDATE_MS) {
    return;
  }
  lastUpdateMs = now;
  resetBreadcrumb.uptimeMs = now;
  resetBreadcrumb.keyed = keyed ? 1 : 0;
  resetBreadcrumb.highPower = highPower ? 1 : 0;
  resetBreadcrumb.keyedForMs = keyed ? keyedForMs : 0;
  resetBreadcrumb.check = resetBreadcrumbCheck(resetBreadcrumb);
}

// Formats "Reset reason: ..." (the wording hosts already look for). The most
// useful fields come first: some hosts keep only the first 120 characters, so
// the firmware label goes before the long description, which only restates
// the reason name.
int formatResetReasonLine(char *out, size_t outSize) {
  int n;
  if (previousBreadcrumbValid) {
    n = snprintf(out, outSize, "Reset reason: %s; prev keyed=%u high=%u keyed_for=%lums up=%lums; boot #%lu; fw " FIRMWARE_LABEL "; %s",
                 resetReasonName(bootResetReason), previousBreadcrumb.keyed, previousBreadcrumb.highPower,
                 (unsigned long)previousBreadcrumb.keyedForMs, (unsigned long)previousBreadcrumb.uptimeMs,
                 (unsigned long)resetBreadcrumb.bootCount, resetReasonDescription(bootResetReason));
  } else {
    n = snprintf(out, outSize, "Reset reason: %s; boot #%lu; fw " FIRMWARE_LABEL "; %s", resetReasonName(bootResetReason),
                 (unsigned long)resetBreadcrumb.bootCount, resetReasonDescription(bootResetReason));
  }
  return n;
}

// Formats a one-line summary of the core dump stored in flash, or returns 0 if
// there is none. A dump stays in flash until the next crash overwrites it, so
// it may be older than this boot; the reset reason tells whether it is fresh.
int formatCoreDumpLine(char *out, size_t outSize) {
#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH && CONFIG_ESP_COREDUMP_DATA_FORMAT_ELF
  size_t addr = 0;
  size_t size = 0;
  if (esp_core_dump_image_get(&addr, &size) != ESP_OK || size == 0) {
    return 0;
  }
  esp_err_t check = esp_core_dump_image_check();
  if (check != ESP_OK) {
    return snprintf(out, outSize, "Core dump: %u bytes at 0x%x, invalid (%s)", (unsigned)size, (unsigned)addr,
                    esp_err_to_name(check));
  }
  esp_core_dump_summary_t *summary = (esp_core_dump_summary_t *)malloc(sizeof(esp_core_dump_summary_t));
  if (summary == nullptr) {
    return snprintf(out, outSize, "Core dump: %u bytes at 0x%x (no memory for summary)", (unsigned)size, (unsigned)addr);
  }
  int n;
  if (esp_core_dump_get_summary(summary) == ESP_OK) {
    n = snprintf(out, outSize,
                 "Core dump: %u bytes at 0x%x, task=%.16s pc=0x%08lx exccause=%lu excvaddr=0x%08lx elf=%.16s bt=",
                 (unsigned)size, (unsigned)addr, summary->exc_task, (unsigned long)summary->exc_pc,
                 (unsigned long)summary->ex_info.exc_cause, (unsigned long)summary->ex_info.exc_vaddr,
                 (const char *)summary->app_elf_sha256);
    for (uint32_t i = 0; i < summary->exc_bt_info.depth && i < 16 && n > 0 && (size_t)n < outSize; i++) {
      n += snprintf(out + n, outSize - n, "%s0x%08lx", i == 0 ? "" : ",", (unsigned long)summary->exc_bt_info.bt[i]);
    }
    if (n > 0 && (size_t)n < outSize && summary->exc_bt_info.corrupted) {
      n += snprintf(out + n, outSize - n, " (corrupted)");
    }
  } else {
    n = snprintf(out, outSize, "Core dump: %u bytes at 0x%x (summary unavailable)", (unsigned)size, (unsigned)addr);
  }
  free(summary);
  return n;
#else
  (void)out;
  (void)outSize;
  return 0;
#endif
}

// Plain text on the USB serial port, printed before any kv4p framing so it is
// readable with any serial monitor.
void printResetDiagnostics(Print &out) {
  char line[200];
  if (formatResetReasonLine(line, sizeof(line)) > 0) {
    out.println(line);
  }
  if (coreDumpLine != nullptr) {
    out.println(coreDumpLine);
  }
  out.printf("Heap free=%u min=%u largest=%u\n", (unsigned)ESP.getFreeHeap(),
             (unsigned)ESP.getMinFreeHeap(), (unsigned)ESP.getMaxAllocHeap());
}

// The same report as COMMAND_DEBUG_INFO frames to one host session. Sent on
// every connect, so the same boot can be reported more than once; the boot
// number tells repeats apart.
void sendResetDiagnostics(Stream &out) {
  char line[200];
  if (formatResetReasonLine(line, sizeof(line)) > 0) {
    sendKv4pVendorFrame(out, COMMAND_DEBUG_INFO, (const uint8_t *)line, strnlen(line, sizeof(line)));
  }
  if (coreDumpLine != nullptr) {
    sendKv4pVendorFrame(out, COMMAND_DEBUG_INFO, (const uint8_t *)coreDumpLine, strlen(coreDumpLine));
  }
}
