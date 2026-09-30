#include "bridge/Bridge.h"

#include <Arduino.h>

#include <cstring>

#include "board/Board.h"
#include "board/BootOptions.h"
#include "board/Version.h"
#include "drivers/CanBus.h"

namespace bridge {
namespace {

constexpr uint32_t kStatusPeriodMs = 250;
constexpr uint32_t kBootloaderDelayMs = 50;  // let the acknowledgement reach the host

framing::Parser g_parser;
uint32_t g_nextStatusMs = 0;
bool g_wasConnected = false;
uint32_t g_bootloaderAtMs = 0;
bool g_bootloaderPending = false;
bool g_bootOptionsOk = false;

bool hostConnected() { return SerialUSB.dtr(); }

// Writes a whole frame or nothing.
bool writeToHost(uint16_t id, uint8_t opcode, const uint8_t* payload, uint8_t length) {
  if (!hostConnected()) return false;
  uint8_t buffer[framing::kMaxEncoded];
  const uint8_t size = framing::encode(id, opcode, payload, length, buffer);
  if (SerialUSB.availableForWrite() < size) return false;
  return SerialUSB.write(buffer, size) == size;
}

adapter::Status currentStatus() {
  const can_bus::Stats& stats = can_bus::stats();
  const can_bus::BusStatus bus = can_bus::status();
  adapter::Status s;
  s.busState = static_cast<adapter::BusState>(bus.state);
  s.tec = bus.tec;
  s.rec = bus.rec;
  s.lastError = bus.lastError;
  s.toCan = stats.delivered;
  s.fromCan = stats.received;
  s.droppedToCan = stats.txDropped;
  s.droppedFromCan = stats.rxDropped;
  s.framingErrors = g_parser.rejectedFrames();
  s.busOffEvents = stats.busOffEvents;
  return s;
}

void sendStatus(adapter::Op op) {
  uint8_t payload[adapter::kStatusSize];
  const uint8_t size = adapter::encodeStatus(currentStatus(), payload);
  (void)writeToHost(framing::kAdapterId, static_cast<uint8_t>(op), payload, size);
}

void sendInfo() {
  adapter::Info info;
  info.fwMajor = version::kMajor;
  info.fwMinor = version::kMinor;
  info.fwPatch = version::kPatch;
  info.hardwareRevision = board::kHardwareRevision;
  info.flags = static_cast<uint8_t>((board::g_runningOnCrystal ? adapter::kInfoCrystalClock : 0) |
                                    (g_bootOptionsOk ? adapter::kInfoBootOptionsOk : 0));
  const uint32_t uid[3] = {HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2()};
  std::memcpy(info.uid, uid, sizeof info.uid);
  uint8_t payload[adapter::kInfoSize];
  (void)writeToHost(framing::kAdapterId, static_cast<uint8_t>(adapter::Op::GetInfo), payload,
                    adapter::encodeInfo(info, payload));
}

void handleAdapterFrame(const framing::Frame& frame, uint32_t nowMs) {
  switch (static_cast<adapter::Op>(frame.opcode)) {
    case adapter::Op::GetInfo:
      sendInfo();
      break;
    case adapter::Op::GetStatus:
      sendStatus(adapter::Op::GetStatus);
      break;
    case adapter::Op::ResetCounters:
      can_bus::resetStats();
      sendStatus(adapter::Op::ResetCounters);
      break;
    case adapter::Op::EnterBootloader:
      (void)writeToHost(framing::kAdapterId, frame.opcode, nullptr, 0);
      g_bootloaderPending = true;
      g_bootloaderAtMs = nowMs + kBootloaderDelayMs;
      break;
  }
}

void hostToBus(uint32_t nowMs) {
  // Flow control: while the transmit queue is full, bytes stay in the USB
  // receive queue and the USB stack holds the host off (NAK) — nothing is lost.
  // One byte completes at most one frame, so one free slot is enough.
  framing::Frame frame;
  while (can_bus::txFree() > 0 && SerialUSB.available() > 0) {
    if (!g_parser.push(static_cast<uint8_t>(SerialUSB.read()), frame)) continue;
    if (frame.id == framing::kAdapterId) {
      handleAdapterFrame(frame, nowMs);
      continue;
    }
    CanFrame can;
    can.id = frame.id;
    can.length = static_cast<uint8_t>(1 + frame.length);
    can.data[0] = frame.opcode;
    std::memcpy(can.data + 1, frame.payload, frame.length);
    (void)can_bus::send(can, nowMs);
  }
}

void busToHost() {
  const bool connected = hostConnected();
  while (const CanFrame* can = can_bus::peek()) {
    // Nobody listening: discard. Host busy: keep the frame for the next pass.
    if (connected && can->length > 0) {
      if (SerialUSB.availableForWrite() < framing::kMaxEncoded) return;
      if (!writeToHost(can->id, can->data[0], can->data + 1, static_cast<uint8_t>(can->length - 1))) return;
    }
    can_bus::pop();  // a frame without an opcode cannot be framed and is dropped
  }
}

}  // namespace

void begin() {
  g_bootOptionsOk = boot_options::ensureFlashBoot();  // resets once if they needed fixing
  SerialUSB.begin();
}

void service(uint32_t nowMs) {
  // A host just opened the port: introduce the adapter, so the host knows where
  // this session starts (bytes queued before it are stale).
  const bool connected = hostConnected();
  if (connected && !g_wasConnected) sendInfo();
  g_wasConnected = connected;

  hostToBus(nowMs);
  can_bus::service(nowMs);
  busToHost();

  if (connected && static_cast<int32_t>(nowMs - g_nextStatusMs) >= 0) {
    g_nextStatusMs = nowMs + kStatusPeriodMs;
    sendStatus(adapter::Op::GetStatus);
  }
  if (g_bootloaderPending && static_cast<int32_t>(nowMs - g_bootloaderAtMs) >= 0) {
    SerialUSB.end();  // detach from the host before handing over
    boot_options::restartIntoBootloader();
  }
}

}  // namespace bridge
