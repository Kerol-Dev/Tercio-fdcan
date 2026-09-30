#pragma once
// -----------------------------------------------------------------------------
// The adapter's own channel: serial frames with id = framing::kAdapterId.
// Mirrored by the host libraries (Tercio-s1 Firmware/Libraries); change together.
//
// Host -> adapter            Adapter -> host (same opcode)
//   GetInfo                    InfoLayout
//   GetStatus                  StatusLayout (also sent every 250 ms while the
//                                            host holds the port open)
//   ResetCounters              StatusLayout (after the reset)
//   EnterBootloader            empty; then the adapter restarts into ST's USB
//                              DFU bootloader for a firmware update
// -----------------------------------------------------------------------------
#include <cstdint>
#include <cstring>

namespace adapter {

inline constexpr uint8_t kProtocolVersion = 1;

enum class Op : uint8_t { GetInfo = 0x00, GetStatus = 0x01, ResetCounters = 0x02, EnterBootloader = 0x03 };

inline constexpr uint8_t kInfoCrystalClock = 1u << 0;   // core clock from the crystal
inline constexpr uint8_t kInfoBootOptionsOk = 1u << 1;  // BOOT0 pin ignored (see BootOptions.cpp)

enum class BusState : uint8_t { ErrorActive = 0, Warning = 1, ErrorPassive = 2, BusOff = 3 };

// InfoLayout, 20 bytes:
//   u8 protocol  u8 fw major  u8 fw minor  u8 fw patch  u8 hw revision
//   u8 flags     u16 reserved u8 uid[12]
inline constexpr uint8_t kInfoSize = 20;

// StatusLayout, 28 bytes:
//   u8 bus state   u8 TEC   u8 REC   u8 last protocol error (PSR.LEC)
//   u32 frames to CAN     u32 frames from CAN
//   u32 dropped to CAN    (queue full or undeliverable for 50 ms)
//   u32 dropped from CAN  (host not reading fast enough)
//   u32 rejected host frames (bad COBS/CRC/id: serial framing errors)
//   u16 bus-off events    u16 reserved
inline constexpr uint8_t kStatusSize = 28;

struct Status {
  BusState busState = BusState::ErrorActive;
  uint8_t tec = 0;
  uint8_t rec = 0;
  uint8_t lastError = 0;
  uint32_t toCan = 0;
  uint32_t fromCan = 0;
  uint32_t droppedToCan = 0;
  uint32_t droppedFromCan = 0;
  uint32_t framingErrors = 0;
  uint16_t busOffEvents = 0;
};

inline uint8_t encodeStatus(const Status& s, uint8_t* out) {
  uint8_t* p = out;
  auto put = [&p](const void* v, uint8_t n) {
    std::memcpy(p, v, n);
    p += n;
  };
  const uint8_t head[4] = {static_cast<uint8_t>(s.busState), s.tec, s.rec, s.lastError};
  put(head, 4);
  put(&s.toCan, 4);
  put(&s.fromCan, 4);
  put(&s.droppedToCan, 4);
  put(&s.droppedFromCan, 4);
  put(&s.framingErrors, 4);
  put(&s.busOffEvents, 2);
  const uint16_t reserved = 0;
  put(&reserved, 2);
  return static_cast<uint8_t>(p - out);
}

struct Info {
  uint8_t fwMajor = 0, fwMinor = 0, fwPatch = 0;
  uint8_t hardwareRevision = 0;
  uint8_t flags = 0;
  uint8_t uid[12] = {};
};

inline uint8_t encodeInfo(const Info& info, uint8_t* out) {
  const uint8_t head[8] = {kProtocolVersion, info.fwMajor, info.fwMinor, info.fwPatch,
                           info.hardwareRevision, info.flags, 0, 0};
  std::memcpy(out, head, sizeof head);
  std::memcpy(out + sizeof head, info.uid, sizeof info.uid);
  return kInfoSize;
}

}  // namespace adapter
