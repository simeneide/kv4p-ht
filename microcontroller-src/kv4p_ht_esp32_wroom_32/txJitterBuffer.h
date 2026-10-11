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

#include <memory>

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
// the frame that carried it. releaseCredit() hands credit back as soon as a
// frame is queued, as long as the host could not then send more than the
// buffer has room for: the credit still held for a session never drops below
// its window minus the free space. So the host sees credit flow in real time
// from the first frame (including during the pre-roll), while a faster than
// real-time burst is held back by free space, not by the window. Whatever
// credit is still held for a frame goes back when the frame is consumed.
//
// Frames are consumed in chunks (popChunk) so a large host frame (several
// ADPCM blocks) can be played out a block at a time, at the DAC's pace.
//
// The audio storage is allocated on the heap once, at construction, to keep
// the static DRAM segment (tight with Bluetooth enabled) free.
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
    uint32_t peakHeldAckBytes = 0;    // Most credit held back at once (after releaseCredit).
  };

  // samplesPerBlock / bytesPerBlock convert buffered bytes to audio time.
  // For IMA WAV ADPCM mono that is 249 samples per 128-byte block.
  TxJitterBuffer(uint32_t sampleRate, uint32_t samplesPerBlock, uint32_t bytesPerBlock)
    : _sampleRate(sampleRate), _samplesPerBlock(samplesPerBlock), _bytesPerBlock(bytesPerBlock),
      _bytes(new uint8_t[CAPACITY_BYTES]) {}

  // Start a new transmission. prerollMs is the target buffered audio before
  // playout starts (and restarts after an underrun). leadMs is the minimum
  // time from reset() (the moment the transmitter is keyed) to the first
  // playout: receivers need carrier for a while before they unmute, however
  // fast the host fills the buffer.
  void reset(uint32_t nowMs, uint32_t prerollMs, uint32_t leadMs = 0) {
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
    _leadMs = leadMs;
    _stats = Stats{};
  }

  void setPrerollMs(uint32_t prerollMs) { _prerollMs = prerollMs; }
  uint32_t prerollMs() const { return _prerollMs; }
  uint32_t leadMs() const { return _leadMs; }

  size_t freeBytes() const { return CAPACITY_BYTES - _byteCount; }
  static constexpr size_t capacityBytes() { return CAPACITY_BYTES; }

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
  // Nothing is played before the carrier lead time since reset() has passed;
  // until then the buffer keeps filling (bounded by free space and credit).
  // After that, playout (re)starts when any of these holds:
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
      if (!_stats.prerollReached && (uint32_t)(nowMs - _startMs) < _leadMs) {
        return false;
      }
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

  // Consume up to maxBytes of the oldest frame, handing them to
  // sink(const uint8_t *, size_t) in at most two contiguous pieces (the ring
  // may wrap), so no copy buffer is needed. Returns the bytes consumed. When
  // this finishes the frame, meta receives its ack information (the credit
  // still held for it); otherwise meta.ackBytes is 0. meta.len is the number
  // of payload bytes the frame had left before this call.
  template <typename Sink>
  size_t popChunk(Sink &&sink, size_t maxBytes, Entry &meta) {
    meta = Entry{nullptr, 0, 0};
    if (_count == 0 || maxBytes == 0) {
      return 0;
    }
    Entry &head = _entries[_head];
    meta.len = head.len;
    size_t len = head.len < maxBytes ? head.len : maxBytes;
    size_t first = CAPACITY_BYTES - _byteHead;
    if (first > len) first = len;
    sink(&_bytes[_byteHead], first);
    if (len > first) {
      sink(&_bytes[0], len - first);
    }
    _byteHead = (_byteHead + len) % CAPACITY_BYTES;
    _byteCount -= len;
    head.len = (uint16_t)(head.len - len);
    if (head.len == 0) {
      meta.ackTag = head.ackTag;
      meta.ackBytes = head.ackBytes;
      _heldAckBytes -= head.ackBytes;
      _head = (_head + 1) % CAPACITY_ENTRIES;
      --_count;
      ++_stats.framesOut;
    }
    return len;
  }

  // Remove what is left of the oldest frame, as popChunk.
  template <typename Sink>
  size_t pop(Sink &&sink, Entry &meta) {
    return popChunk(sink, (size_t)0xFFFF, meta);
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

  // Hand back credit for ackTag early, keeping only what bounds the host by
  // free space: a host with `window` bytes of credit can have at most
  // window - held bytes on the way, so holding at least window - freeBytes()
  // means whatever it sends next always fits. Credit counts encoded KISS bytes,
  // which are never fewer than the payload they carry, so the bound is
  // conservative. window 0 means unknown: hold everything until played.
  // Returns the credit released (to send to that session now).
  size_t releaseCredit(const void *ackTag, size_t window) {
    size_t released = 0;
    if (ackTag != nullptr && window > 0) {
      size_t held = heldAckBytesFor(ackTag);
      size_t free = freeBytes();
      size_t mustHold = window > free ? window - free : 0;
      size_t releasable = held > mustHold ? held - mustHold : 0;
      size_t idx = _head;
      for (size_t i = 0; i < _count && releasable > 0; ++i) {
        Entry &e = _entries[idx];
        if (e.ackTag == ackTag && e.ackBytes > 0) {
          uint16_t part = e.ackBytes < releasable ? e.ackBytes : (uint16_t)releasable;
          e.ackBytes = (uint16_t)(e.ackBytes - part);
          _heldAckBytes -= part;
          releasable -= part;
          released += part;
        }
        idx = (idx + 1) % CAPACITY_ENTRIES;
      }
    }
    if (_heldAckBytes > _stats.peakHeldAckBytes) _stats.peakHeldAckBytes = (uint32_t)_heldAckBytes;
    return released;
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

  std::unique_ptr<uint8_t[]> _bytes;
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
  uint32_t _leadMs = 0;
  Stats _stats{};
};
