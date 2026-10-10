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

#include <stddef.h>
#include <stdint.h>
#include <string.h>

// Encoded TX audio jitter buffer.
//
// Host TX audio frames (ADPCM) are queued here instead of being decoded the
// moment they arrive. Playout starts once a pre-roll has built up, so a
// wireless link that delivers audio in bursts (for example BLE while the phone
// also runs a Bluetooth headset) does not stutter on the air. When the buffer
// runs dry mid-transmission the caller plays silence and the buffer goes back
// to pre-rolling instead of playing each late frame as soon as it trickles in.
//
// Each entry remembers the flow-control credit (encoded KISS frame length) of
// the frame that carried it, so credits go back to the host only as audio is
// actually consumed. With the host honouring the window, the buffer can never
// hold more than one window of audio, which bounds both memory and latency.
//
// The class is transport and hardware agnostic so it can be unit tested
// natively; the caller owns timing, decoding and the I2S output.
template <size_t CAPACITY_BYTES, size_t CAPACITY_ENTRIES>
class TxJitterBuffer {
public:
  static_assert(CAPACITY_BYTES > 0, "CAPACITY_BYTES must be > 0");
  static_assert(CAPACITY_ENTRIES > 0, "CAPACITY_ENTRIES must be > 0");

  struct Entry {
    const void *ackTag;  // Who to return the credit to (a session), may be null.
    uint16_t len;        // Payload bytes.
    uint16_t ackBytes;   // Flow-control credit to return when consumed.
  };

  struct Stats {
    uint32_t framesIn = 0;
    uint32_t bytesIn = 0;
    uint32_t framesOut = 0;
    uint32_t underruns = 0;           // Buffer ran dry while playing.
    uint32_t overflows = 0;           // Push did not fit; caller played the oldest early.
    uint32_t prerollReachedMs = 0;    // First playout start, relative to reset().
    bool prerollReached = false;
    uint32_t peakBytes = 0;
    uint32_t maxArrivalGapMs = 0;     // Longest gap between two pushes.
  };

  // samplesPerBlock / bytesPerBlock convert buffered bytes to audio time.
  // For IMA WAV ADPCM mono that is 249 samples per 128-byte block.
  TxJitterBuffer(uint32_t sampleRate, uint32_t samplesPerBlock, uint32_t bytesPerBlock)
    : _sampleRate(sampleRate), _samplesPerBlock(samplesPerBlock), _bytesPerBlock(bytesPerBlock) {}

  // Start a new transmission. prerollMs is the target buffered audio before
  // playout starts (and restarts after an underrun).
  void reset(uint32_t nowMs, uint32_t prerollMs) {
    _head = _tail = _count = 0;
    _byteHead = _byteTail = _byteCount = 0;
    _heldAckBytes = 0;
    _playing = false;
    _episodeStartMs = 0;
    _episodeStarted = false;
    _lastPushMs = 0;
    _hasLastPush = false;
    _startMs = nowMs;
    _prerollMs = prerollMs;
    _stats = Stats{};
  }

  void setPrerollMs(uint32_t prerollMs) { _prerollMs = prerollMs; }
  uint32_t prerollMs() const { return _prerollMs; }

  bool canAccept(size_t len) const {
    return len > 0 && len <= CAPACITY_BYTES && _count < CAPACITY_ENTRIES
      && len <= (CAPACITY_BYTES - _byteCount) && len <= 0xFFFF;
  }

  // Queue one frame. Returns false when it does not fit; the caller should
  // then pop and play the oldest entry and retry, so order is preserved and
  // nothing is dropped.
  bool push(const uint8_t *data, size_t len, const void *ackTag, uint16_t ackBytes, uint32_t nowMs) {
    if (data == nullptr || !canAccept(len)) {
      if (data != nullptr && len > 0) {
        ++_stats.overflows;
      }
      return false;
    }
    size_t first = CAPACITY_BYTES - _byteTail;
    if (first > len) first = len;
    memcpy(&_bytes[_byteTail], data, first);
    if (len > first) {
      memcpy(&_bytes[0], data + first, len - first);
    }
    _byteTail = (_byteTail + len) % CAPACITY_BYTES;
    _byteCount += len;
    Entry &e = _entries[_tail];
    e.ackTag = ackTag;
    e.len = (uint16_t)len;
    e.ackBytes = ackBytes;
    _tail = (_tail + 1) % CAPACITY_ENTRIES;
    ++_count;
    _heldAckBytes += ackBytes;

    if (_hasLastPush) {
      uint32_t gap = nowMs - _lastPushMs;
      if (gap > _stats.maxArrivalGapMs) _stats.maxArrivalGapMs = gap;
    }
    _lastPushMs = nowMs;
    _hasLastPush = true;
    if (!_episodeStarted) {
      _episodeStartMs = nowMs;
      _episodeStarted = true;
    }
    ++_stats.framesIn;
    _stats.bytesIn += len;
    if (_byteCount > _stats.peakBytes) _stats.peakBytes = _byteCount;
    return true;
  }

  // Decide whether audio should be played now. Call once per playout slot.
  // Playout (re)starts when any of these holds:
  //  - the buffered audio reaches the pre-roll target,
  //  - the first frame of this pre-roll episode is pre-roll old (a host that
  //    cannot fill the target, e.g. a small window, is never stalled forever),
  //  - creditPressure: the host is nearly out of window credit and cannot
  //    send more until we consume something,
  //  - drain: PTT was released, play out whatever is left.
  bool shouldPlay(uint32_t nowMs, bool creditPressure, bool drain) {
    if (_count == 0) {
      return false;
    }
    if (!_playing) {
      bool levelReached = bufferedMs() >= _prerollMs;
      bool aged = _episodeStarted && (uint32_t)(nowMs - _episodeStartMs) >= _prerollMs;
      if (levelReached || aged || creditPressure || drain) {
        _playing = true;
        if (!_stats.prerollReached) {
          _stats.prerollReached = true;
          _stats.prerollReachedMs = nowMs - _startMs;
        }
      }
    }
    return _playing;
  }

  // Report that a playout slot found nothing to play. If we were playing, this
  // is an underrun: go back to pre-rolling.
  void noteEmptySlot() {
    if (_playing && _count == 0) {
      _playing = false;
      _episodeStarted = false;
      ++_stats.underruns;
    }
  }

  // Remove the oldest frame, handing its payload to sink(const uint8_t *, size_t)
  // in at most two contiguous pieces (the ring may wrap), so no copy buffer is
  // needed. meta receives its ack information. Returns the payload length.
  template <typename Sink>
  size_t pop(Sink &&sink, Entry &meta) {
    if (_count == 0) {
      return 0;
    }
    meta = _entries[_head];
    _head = (_head + 1) % CAPACITY_ENTRIES;
    --_count;
    _heldAckBytes -= meta.ackBytes;
    size_t len = meta.len;
    size_t first = CAPACITY_BYTES - _byteHead;
    if (first > len) first = len;
    sink(&_bytes[_byteHead], first);
    if (len > first) {
      sink(&_bytes[0], len - first);
    }
    _byteHead = (_byteHead + len) % CAPACITY_BYTES;
    _byteCount -= len;
    ++_stats.framesOut;
    return len;
  }

  // Remove the oldest frame without using its payload.
  size_t discard(Entry &meta) {
    return pop([](const uint8_t *, size_t) {}, meta);
  }

  // A session went away: keep its audio but never return its credits, since a
  // reconnected host starts with a fresh window.
  void forgetAckTag(const void *ackTag) {
    size_t idx = _head;
    for (size_t i = 0; i < _count; ++i) {
      Entry &e = _entries[idx];
      if (e.ackTag == ackTag) {
        _heldAckBytes -= e.ackBytes;
        e.ackBytes = 0;
        e.ackTag = nullptr;
      }
      idx = (idx + 1) % CAPACITY_ENTRIES;
    }
  }

  // Credit currently held for one session (the host's window is per session).
  size_t heldAckBytesFor(const void *ackTag) const {
    size_t held = 0;
    size_t idx = _head;
    for (size_t i = 0; i < _count; ++i) {
      if (_entries[idx].ackTag == ackTag) {
        held += _entries[idx].ackBytes;
      }
      idx = (idx + 1) % CAPACITY_ENTRIES;
    }
    return held;
  }

  uint32_t bufferedMs() const {
    return bytesToMs(_byteCount);
  }

  uint32_t bytesToMs(size_t bytes) const {
    if (_bytesPerBlock == 0 || _sampleRate == 0) return 0;
    uint64_t samples = (uint64_t)bytes * _samplesPerBlock / _bytesPerBlock;
    return (uint32_t)(samples * 1000 / _sampleRate);
  }

  bool playing() const { return _playing; }
  bool empty() const { return _count == 0; }
  size_t entries() const { return _count; }
  size_t bufferedBytes() const { return _byteCount; }
  size_t heldAckBytes() const { return _heldAckBytes; }
  const Stats &stats() const { return _stats; }

private:
  uint32_t _sampleRate;
  uint32_t _samplesPerBlock;
  uint32_t _bytesPerBlock;

  uint8_t _bytes[CAPACITY_BYTES];
  size_t _byteHead = 0;
  size_t _byteTail = 0;
  size_t _byteCount = 0;

  Entry _entries[CAPACITY_ENTRIES];
  size_t _head = 0;
  size_t _tail = 0;
  size_t _count = 0;
  size_t _heldAckBytes = 0;

  bool _playing = false;
  bool _episodeStarted = false;
  uint32_t _episodeStartMs = 0;
  uint32_t _lastPushMs = 0;
  bool _hasLastPush = false;
  uint32_t _startMs = 0;
  uint32_t _prerollMs = 0;
  Stats _stats{};
};
