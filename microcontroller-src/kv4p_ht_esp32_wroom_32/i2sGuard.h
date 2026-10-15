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

// Keep a failed i2s_driver_install() from becoming a crash.
//
// Every key-down uninstalls the RX (built-in ADC) driver on I2S0 and installs
// the TX (PDM) driver, which allocates its DMA ring (6 x 1024 bytes of
// DMA-capable RAM with the mono ring in txAudio.h, 6 x 2048 with the old
// stereo one) right then. When that allocation fails, IDF 4.4 frees the
// half-built driver and leaves p_i2s[0] NULL. arduino-audio-tools'
// I2SDriverESP32::begin() logs the failure and carries on into i2s_set_pin(),
// which dereferences p_i2s[0] (LoadProhibited, EXCVADDR 0x3c): the radio
// reboots at the moment the pilot presses PTT. The same holds for every other
// call that takes a port (zero_dma_buffer, write, read, ...).
//
// So every port call the firmware and its libraries make goes through a
// wrapper (linked with -Wl,--wrap, see platformio.ini) that returns
// ESP_ERR_INVALID_STATE for a port whose driver is not installed, and the
// callers check i2sGuardInstalled() to refuse the key cleanly. Calls inside
// the IDF driver itself are not wrapped (the linker only redirects undefined
// references), so the driver's own internal calls are untouched.

#include <Arduino.h>
#include <driver/i2s.h>
#include <esp_heap_caps.h>

static volatile bool i2sGuardPortInstalled[I2S_NUM_MAX] = {};
static volatile uint32_t i2sGuardBlockedCalls = 0;
static volatile uint32_t i2sGuardInstallFailures = 0;

static inline bool i2sGuardInstalled(i2s_port_t port) {
  return port < I2S_NUM_MAX && i2sGuardPortInstalled[port];
}

// DMA-capable heap, which is what the I2S DMA ring is allocated from.
struct I2sGuardHeap {
  uint32_t dmaFree;
  uint32_t dmaLargest;
  uint32_t internalFree;
};

static inline I2sGuardHeap i2sGuardHeapNow() {
  return {
    (uint32_t)heap_caps_get_free_size(MALLOC_CAP_DMA),
    (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_DMA),
    (uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
  };
}

#define I2S_GUARD_PORT_OR_FAIL(port)        \
  do {                                      \
    if (!i2sGuardInstalled(port)) {         \
      i2sGuardBlockedCalls = i2sGuardBlockedCalls + 1; \
      return ESP_ERR_INVALID_STATE;         \
    }                                       \
  } while (0)

extern "C" {
esp_err_t __real_i2s_driver_install(i2s_port_t, const i2s_config_t *, int, void *);
esp_err_t __real_i2s_driver_uninstall(i2s_port_t);
esp_err_t __real_i2s_set_pin(i2s_port_t, const i2s_pin_config_t *);
esp_err_t __real_i2s_zero_dma_buffer(i2s_port_t);
esp_err_t __real_i2s_write(i2s_port_t, const void *, size_t, size_t *, TickType_t);
esp_err_t __real_i2s_write_expand(i2s_port_t, const void *, size_t, size_t, size_t, size_t *, TickType_t);
esp_err_t __real_i2s_read(i2s_port_t, void *, size_t, size_t *, TickType_t);
esp_err_t __real_i2s_set_sample_rates(i2s_port_t, uint32_t);
esp_err_t __real_i2s_set_clk(i2s_port_t, uint32_t, uint32_t, i2s_channel_t);
esp_err_t __real_i2s_start(i2s_port_t);
esp_err_t __real_i2s_stop(i2s_port_t);
esp_err_t __real_i2s_adc_enable(i2s_port_t);
esp_err_t __real_i2s_adc_disable(i2s_port_t);

esp_err_t __wrap_i2s_driver_install(i2s_port_t port, const i2s_config_t *config, int queueSize, void *queue) {
  esp_err_t r = __real_i2s_driver_install(port, config, queueSize, queue);
  // A refusal because the port is already installed leaves it installed.
  if (r == ESP_OK && port < I2S_NUM_MAX) {
    i2sGuardPortInstalled[port] = true;
  } else if (r != ESP_OK && r != ESP_ERR_INVALID_STATE) {
    i2sGuardInstallFailures = i2sGuardInstallFailures + 1;
  }
  return r;
}

esp_err_t __wrap_i2s_driver_uninstall(i2s_port_t port) {
  I2S_GUARD_PORT_OR_FAIL(port);
  esp_err_t r = __real_i2s_driver_uninstall(port);
  if (r == ESP_OK) {
    i2sGuardPortInstalled[port] = false;
  }
  return r;
}

esp_err_t __wrap_i2s_set_pin(i2s_port_t port, const i2s_pin_config_t *pin) {
  I2S_GUARD_PORT_OR_FAIL(port);
  return __real_i2s_set_pin(port, pin);
}

esp_err_t __wrap_i2s_zero_dma_buffer(i2s_port_t port) {
  I2S_GUARD_PORT_OR_FAIL(port);
  return __real_i2s_zero_dma_buffer(port);
}

esp_err_t __wrap_i2s_write(i2s_port_t port, const void *src, size_t size, size_t *written, TickType_t wait) {
  if (!i2sGuardInstalled(port)) {
    if (written != nullptr) *written = 0;
  }
  I2S_GUARD_PORT_OR_FAIL(port);
  return __real_i2s_write(port, src, size, written, wait);
}

esp_err_t __wrap_i2s_write_expand(i2s_port_t port, const void *src, size_t size, size_t srcBits, size_t aimBits,
                                  size_t *written, TickType_t wait) {
  if (!i2sGuardInstalled(port)) {
    if (written != nullptr) *written = 0;
  }
  I2S_GUARD_PORT_OR_FAIL(port);
  return __real_i2s_write_expand(port, src, size, srcBits, aimBits, written, wait);
}

esp_err_t __wrap_i2s_read(i2s_port_t port, void *dest, size_t size, size_t *read, TickType_t wait) {
  if (!i2sGuardInstalled(port)) {
    if (read != nullptr) *read = 0;
  }
  I2S_GUARD_PORT_OR_FAIL(port);
  return __real_i2s_read(port, dest, size, read, wait);
}

esp_err_t __wrap_i2s_set_sample_rates(i2s_port_t port, uint32_t rate) {
  I2S_GUARD_PORT_OR_FAIL(port);
  return __real_i2s_set_sample_rates(port, rate);
}

esp_err_t __wrap_i2s_set_clk(i2s_port_t port, uint32_t rate, uint32_t bits, i2s_channel_t ch) {
  I2S_GUARD_PORT_OR_FAIL(port);
  return __real_i2s_set_clk(port, rate, bits, ch);
}

esp_err_t __wrap_i2s_start(i2s_port_t port) {
  I2S_GUARD_PORT_OR_FAIL(port);
  return __real_i2s_start(port);
}

esp_err_t __wrap_i2s_stop(i2s_port_t port) {
  I2S_GUARD_PORT_OR_FAIL(port);
  return __real_i2s_stop(port);
}

esp_err_t __wrap_i2s_adc_enable(i2s_port_t port) {
  I2S_GUARD_PORT_OR_FAIL(port);
  return __real_i2s_adc_enable(port);
}

esp_err_t __wrap_i2s_adc_disable(i2s_port_t port) {
  I2S_GUARD_PORT_OR_FAIL(port);
  return __real_i2s_adc_disable(port);
}
}  // extern "C"
