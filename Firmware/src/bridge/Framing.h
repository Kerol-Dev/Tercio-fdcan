#pragma once
// -----------------------------------------------------------------------------
// USB serial framing between host and adapter (both directions), version 2.
//
//   packet  = [id u16 LE][opcode u8][payload 0..63 bytes][crc16 u16 LE]
//   on wire = COBS(packet) followed by a 0x00 delimiter
//
// COBS removes every zero byte from the packet, so 0x00 only ever marks a
// frame boundary: after any corruption (a reopened port starting mid-frame,
// lost bytes) the next delimiter resynchronises deterministically. The CRC
// (CRC-16/CCITT-FALSE over id..payload) rejects whatever arrives damaged.
//
// A CAN frame maps to a packet as id = CAN id, opcode = data[0], payload =
// data[1..]. id 0x800 (outside the 11-bit range) addresses the adapter itself;
// see AdapterProtocol.h. Hardware-independent (unit tested on the host).
// -----------------------------------------------------------------------------
#include <cstdint>
#include <cstring>

namespace framing {

inline constexpr uint8_t kVersion = 2;
inline constexpr uint8_t kMaxPayload = 63;
inline constexpr uint16_t kMaxCanId = 0x7FF;
inline constexpr uint16_t kAdapterId = 0x800;
inline constexpr uint8_t kOverhead = 2 + 1 + 2;                               // id, opcode, crc
inline constexpr uint8_t kMaxPacket = kOverhead + kMaxPayload;                // 68
inline constexpr uint8_t kMaxEncoded = kMaxPacket + kMaxPacket / 254 + 1 + 1;  // COBS + delimiter

struct Frame {
  uint16_t id = 0;
  uint8_t opcode = 0;
  uint8_t length = 0;
  uint8_t payload[kMaxPayload] = {};
};

inline bool validId(uint16_t id) { return id <= kMaxCanId || id == kAdapterId; }

// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF (check "123456789" = 0x29B1).
inline uint16_t crc16(const uint8_t* data, uint8_t length) {
  uint16_t crc = 0xFFFF;
  while (length--) {
    crc ^= static_cast<uint16_t>(*data++) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
  }
  return crc;
}

// COBS: `out` holds length + length/254 + 1 bytes. Returns the encoded size.
inline uint8_t cobsEncode(const uint8_t* in, uint8_t length, uint8_t* out) {
  uint8_t codeIndex = 0, outIndex = 1, code = 1;
  for (uint8_t i = 0; i < length; ++i) {
    if (in[i] == 0) {
      out[codeIndex] = code;
      codeIndex = outIndex++;
      code = 1;
    } else {
      out[outIndex++] = in[i];
      if (++code == 0xFF) {
        out[codeIndex] = code;
        codeIndex = outIndex++;
        code = 1;
      }
    }
  }
  out[codeIndex] = code;
  return outIndex;
}

// Returns the decoded size, or -1 if `in` is not valid COBS.
inline int cobsDecode(const uint8_t* in, uint8_t length, uint8_t* out) {
  uint8_t i = 0;
  int o = 0;
  while (i < length) {
    const uint8_t code = in[i++];
    if (code == 0) return -1;
    for (uint8_t j = 1; j < code; ++j) {
      if (i >= length || in[i] == 0) return -1;
      out[o++] = in[i++];
    }
    if (code != 0xFF && i < length) out[o++] = 0;
  }
  return o;
}

// Writes the delimited wire form of a frame into `out` (>= kMaxEncoded bytes).
inline uint8_t encode(uint16_t id, uint8_t opcode, const uint8_t* payload, uint8_t length, uint8_t* out) {
  if (length > kMaxPayload) length = kMaxPayload;
  uint8_t packet[kMaxPacket];
  packet[0] = static_cast<uint8_t>(id);
  packet[1] = static_cast<uint8_t>(id >> 8);
  packet[2] = opcode;
  if (length) std::memcpy(packet + 3, payload, length);
  const uint16_t crc = crc16(packet, static_cast<uint8_t>(3 + length));
  packet[3 + length] = static_cast<uint8_t>(crc);
  packet[4 + length] = static_cast<uint8_t>(crc >> 8);
  const uint8_t size = cobsEncode(packet, static_cast<uint8_t>(kOverhead + length), out);
  out[size] = 0;
  return static_cast<uint8_t>(size + 1);
}

// Decodes one delimited packet (without its 0x00). False if damaged.
inline bool decode(const uint8_t* encoded, uint8_t length, Frame& frame) {
  uint8_t packet[kMaxEncoded];  // decoding never grows the data
  if (length > kMaxEncoded) return false;
  const int size = cobsDecode(encoded, length, packet);
  if (size < kOverhead || size > kMaxPacket) return false;
  const uint8_t payloadLength = static_cast<uint8_t>(size - kOverhead);
  const uint16_t crc = static_cast<uint16_t>(packet[size - 2] | (packet[size - 1] << 8));
  if (crc != crc16(packet, static_cast<uint8_t>(size - 2))) return false;
  const uint16_t id = static_cast<uint16_t>(packet[0] | (packet[1] << 8));
  if (!validId(id)) return false;
  frame.id = id;
  frame.opcode = packet[2];
  frame.length = payloadLength;
  std::memcpy(frame.payload, packet + 3, payloadLength);
  return true;
}

// Byte-stream parser: collects bytes up to each 0x00 delimiter and decodes.
class Parser {
 public:
  // Returns true when `byte` completes a valid frame, which is then in `frame`.
  bool push(uint8_t byte, Frame& frame) {
    if (byte != 0) {
      if (length_ < sizeof buffer_)
        buffer_[length_++] = byte;
      else
        overflow_ = true;  // too long to be a frame: drop it at the delimiter
      return false;
    }
    const bool ok = !overflow_ && length_ > 0 && decode(buffer_, length_, frame);
    if (!ok && (length_ > 0 || overflow_)) ++rejected_;
    length_ = 0;
    overflow_ = false;
    return ok;
  }

  uint32_t rejectedFrames() const { return rejected_; }

 private:
  uint8_t buffer_[kMaxEncoded] = {};
  uint8_t length_ = 0;
  bool overflow_ = false;
  uint32_t rejected_ = 0;
};

// CAN-FD data lengths.
inline constexpr uint8_t kFdLengths[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

// Smallest DLC code whose length holds `bytes` (<= 64).
inline uint8_t dlcFor(uint8_t bytes) {
  uint8_t dlc = 0;
  while (dlc < 15 && kFdLengths[dlc] < bytes) ++dlc;
  return dlc;
}

}  // namespace framing
