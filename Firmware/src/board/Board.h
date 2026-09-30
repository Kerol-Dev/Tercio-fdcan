#pragma once
// -----------------------------------------------------------------------------
// Tercio FD hardware description (STM32G431CBT6, LQFP48, 170 MHz).
// Source: USB_CAN schematic.
// -----------------------------------------------------------------------------
#include <Arduino.h>

namespace board {

// ---- CAN-FD (FDCAN1, AF9) ------------------------------------------------------
// RX shares its pin with BOOT0: see BootOptions.cpp.
inline constexpr uint32_t kCanRxPin = PB8;
inline constexpr uint32_t kCanTxPin = PB9;
inline constexpr uint32_t kCanStandbyPin = PA6;  // MCP2562FD STBY, high = standby

// USB FS on PA11/PA12 (handled by the core). The RX/TX LEDs are driven by the
// CAN RXD/TXD lines in hardware; there are no LED GPIOs.

// ---- Clock ------------------------------------------------------------------------
// Crystal Y2: 16 MHz. Verified against HSI at boot, HSI fallback otherwise.
// USB runs from HSI48 disciplined by the CRS to the host's start-of-frame.
inline constexpr uint32_t kHseHz = HSE_VALUE;
static_assert(kHseHz == 16000000, "Tercio FD uses a 16 MHz crystal");
extern bool g_runningOnCrystal;  // set by SystemClock_Config()

inline constexpr uint8_t kHardwareRevision = 1;

}  // namespace board
