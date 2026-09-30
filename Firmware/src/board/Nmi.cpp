// Non-maskable interrupt: crystal failure (clock security system) restarts the
// adapter; the boot-time check then finds no crystal and runs from HSI.
#include <Arduino.h>

extern "C" void NMI_Handler(void) {
  if (RCC->CIFR & RCC_CIFR_CSSF) {
    RCC->CICR = RCC_CICR_CSSC;
    NVIC_SystemReset();
  }
  while (true) {
  }
}
