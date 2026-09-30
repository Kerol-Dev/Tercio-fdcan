#include "board/BootOptions.h"

#include <Arduino.h>

namespace boot_options {
namespace {

constexpr uint32_t kSystemMemory = 0x1FFF0000;  // STM32G4 ROM bootloader
constexpr uint32_t kBootloaderMagic = 0xB007'1D12;
constexpr uint32_t kOptionFixMagic = 0x0B7E'F1C5;  // an option-byte fix was attempted this power cycle
constexpr uint32_t kMaxFailedStarts = 3;

// TAMP backup registers survive a system reset. VBAT is tied to VDD on this
// board, so they clear on a power cycle.
volatile uint32_t& bootloaderRequest() { return TAMP->BKP0R; }
volatile uint32_t& optionFixAttempt() { return TAMP->BKP1R; }
volatile uint32_t& failedStarts() { return TAMP->BKP2R; }

void enableBackupAccess() {
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_RCC_RTCAPB_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
}

[[noreturn]] void jumpToRom() {
  // Hand over a chip as close to reset state as possible.
  __disable_irq();
  HAL_RCC_DeInit();  // note: re-arms SysTick through HAL_InitTick
  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL = 0;
  SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;
  for (uint32_t i = 0; i < sizeof(NVIC->ICER) / sizeof(NVIC->ICER[0]); ++i) {
    NVIC->ICER[i] = 0xFFFFFFFF;
    NVIC->ICPR[i] = 0xFFFFFFFF;
  }
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
  SCB->VTOR = kSystemMemory;
  __DSB();
  __ISB();
  __enable_irq();

  const uint32_t stack = *reinterpret_cast<volatile uint32_t*>(kSystemMemory);
  const auto entry = reinterpret_cast<void (*)()>(*reinterpret_cast<volatile uint32_t*>(kSystemMemory + 4));
  __set_MSP(stack);
  entry();
  while (true) {
  }
}

}  // namespace

void earlyBoot() {
  // Arriving with the PLL still running (e.g. the ROM jumped here without a
  // reset) would make the clock setup fail: start from a clean reset instead.
  if ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) NVIC_SystemReset();

  enableBackupAccess();
  if (bootloaderRequest() == kBootloaderMagic) {
    bootloaderRequest() = 0;
    jumpToRom();
  }
  if (failedStarts() >= kMaxFailedStarts) {
    failedStarts() = 0;
    jumpToRom();
  }
  failedStarts() = failedStarts() + 1;
}

void markHealthy() { failedStarts() = 0; }

bool ensureFlashBoot() {
  const uint32_t optr = FLASH->OPTR;
  const bool ok = !(optr & FLASH_OPTR_nSWBOOT0) && (optr & FLASH_OPTR_nBOOT0);
  const bool attempted = optionFixAttempt() == kOptionFixMagic;
  optionFixAttempt() = 0;
  // One attempt per power cycle: if the write does not stick (e.g. protected
  // option bytes), carry on rather than reset forever.
  if (ok || attempted) return ok;
  optionFixAttempt() = kOptionFixMagic;

  HAL_FLASH_Unlock();
  HAL_FLASH_OB_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
  FLASH_OBProgramInitTypeDef ob{};
  ob.OptionType = OPTIONBYTE_USER;
  ob.USERType = OB_USER_nSWBOOT0 | OB_USER_nBOOT0;  // only these two bits change
  ob.USERConfig = OB_BOOT0_FROM_OB | OB_nBOOT0_SET;
  if (HAL_FLASHEx_OBProgram(&ob) == HAL_OK) HAL_FLASH_OB_Launch();  // reloads the option bytes: resets
  HAL_FLASH_OB_Lock();
  HAL_FLASH_Lock();
  return false;
}

void restartIntoBootloader() {
  bootloaderRequest() = kBootloaderMagic;
  NVIC_SystemReset();
}

}  // namespace boot_options
