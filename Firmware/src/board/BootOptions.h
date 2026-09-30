#pragma once
// -----------------------------------------------------------------------------
// Boot configuration and the way into ST's ROM bootloader (USB DFU).
//
// FDCAN1_RX shares its pin with BOOT0 (PB8), and the transceiver holds RX high
// whenever the bus is idle — including at power-up. With factory option bytes
// the chip samples that pin and would start the ROM bootloader instead of this
// firmware. The fix is to take BOOT0 from the option bytes (nSWBOOT0 = 0,
// nBOOT0 = 1: always boot from flash); the firmware checks and corrects that.
//
// The ROM bootloader stays reachable on purpose:
//  * on request from the host (adapter EnterBootloader), to update over USB;
//  * automatically after 3 failed starts in a row (resets before the firmware
//    ran healthily for 2 s), so an interrupted update can be repaired without
//    an ST-Link.
// -----------------------------------------------------------------------------

namespace boot_options {

// First thing at start-up (from SystemClock_Config): enters the ROM bootloader
// if requested or if this firmware keeps failing to start.
void earlyBoot();
// Call once the firmware has run healthily for a while.
void markHealthy();

// True when BOOT0 is already taken from the option bytes. Otherwise programs
// them and resets (at most once per power cycle).
bool ensureFlashBoot();

// Restarts into the ROM bootloader (USB DFU). Does not return.
[[noreturn]] void restartIntoBootloader();

}  // namespace boot_options
