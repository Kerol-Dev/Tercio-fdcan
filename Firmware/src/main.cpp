// -----------------------------------------------------------------------------
// Tercio FD — USB to CAN-FD adapter firmware entry point.
//
// A single cooperative loop: the bridge moves frames between USB and CAN-FD,
// the CAN driver fills its receive ring from the FDCAN interrupt. Nothing in
// the loop blocks.
// -----------------------------------------------------------------------------
#include <Arduino.h>
#include <IWatchdog.h>

#include "board/BootOptions.h"
#include "bridge/Bridge.h"
#include "drivers/CanBus.h"

namespace {
constexpr uint32_t kCanIrqPriority = 2;
constexpr uint32_t kWatchdogTimeoutUs = 500000;
constexpr uint32_t kHealthyAfterMs = 2000;  // then a later reset is not a failed start
bool g_healthy = false;
}  // namespace

void setup() {
  bridge::begin();
  (void)can_bus::begin(kCanIrqPriority);  // bus health is reported in the adapter status
  IWatchdog.begin(kWatchdogTimeoutUs);
}

void loop() {
  const uint32_t now = millis();
  bridge::service(now);
  if (!g_healthy && now > kHealthyAfterMs) {
    g_healthy = true;
    boot_options::markHealthy();
  }
  IWatchdog.reload();
}
