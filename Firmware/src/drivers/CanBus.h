#pragma once
// -----------------------------------------------------------------------------
// FDCAN1 for a bridge: CAN-FD 500 kbit/s arbitration / 2.5 Mbit/s data with
// bit-rate switching — the same timing as the Tercio S1.
//
//  * Receives every standard-ID frame (extended and remote frames are rejected:
//    the serial framing cannot carry them) into a 64-frame ring, from the
//    interrupt.
//  * Transmits in order from a 64-frame queue, one frame in the controller at a
//    time, in single-shot mode: a lost arbitration or missing acknowledge is
//    retried by software until kTxLifetimeMs after the frame arrived, then the
//    frame is dropped. Nothing is ever cancelled inside the controller, and a
//    command sent while a motor is unplugged never executes when it is plugged
//    in later.
//  * Recovers from bus-off by itself and keeps delivery statistics.
// -----------------------------------------------------------------------------
#include <cstdint>

struct CanFrame {
  uint16_t id = 0;
  uint8_t length = 0;  // bytes, 0..64
  uint8_t data[64] = {};
};

namespace can_bus {

inline constexpr uint32_t kTxLifetimeMs = 50;

struct Stats {
  uint32_t delivered = 0;       // transmitted and acknowledged
  uint32_t received = 0;
  uint32_t txDropped = 0;       // queue full, or not delivered within kTxLifetimeMs
  uint32_t rxDropped = 0;       // ring full or hardware FIFO overrun
  uint16_t busOffEvents = 0;
};

struct BusStatus {
  uint8_t state = 0;      // adapter::BusState
  uint8_t tec = 0;
  uint8_t rec = 0;
  uint8_t lastError = 0;  // most recent protocol error code (PSR.LEC), latched
};

bool begin(uint32_t irqPriority);
// Queues a frame for transmission (FD with BRS, padded to a valid FD length).
bool send(const CanFrame& frame, uint32_t nowMs);
// Free transmit queue slots (the bridge stops reading the host at zero).
uint8_t txFree();

// Oldest received frame, left in place until pop() — so a frame is only
// consumed once it has been forwarded.
const CanFrame* peek();
void pop();

// Main loop: drives transmission, recovers bus-off, samples the error state.
void service(uint32_t nowMs);

const Stats& stats();
void resetStats();
BusStatus status();

}  // namespace can_bus
