#pragma once
// -----------------------------------------------------------------------------
// USB <-> CAN-FD bridge. Host frames are validated (COBS + CRC, Framing.h) and
// queued for the bus; bus frames are forwarded whole or not at all. Frames addressed to framing::kAdapterId are
// handled here (AdapterProtocol.h).
// -----------------------------------------------------------------------------
#include <cstdint>

#include "bridge/AdapterProtocol.h"
#include "bridge/Framing.h"

namespace bridge {

void begin();
// Main loop, every iteration.
void service(uint32_t nowMs);

}  // namespace bridge
