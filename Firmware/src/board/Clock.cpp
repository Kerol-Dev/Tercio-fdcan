// -----------------------------------------------------------------------------
// System clock: 170 MHz from the crystal when it measures right, otherwise from
// HSI16 — the same scheme as the Tercio S1, so both ends of the bus share the
// CAN bit timing. USB runs from HSI48, disciplined by the clock recovery system
// (CRS) to the host's 1 kHz start-of-frame, which keeps it within the ±0.25 %
// USB full-speed tolerance at any temperature.
// -----------------------------------------------------------------------------
#include <Arduino.h>

#include "board/Board.h"
#include "board/BootOptions.h"

namespace {

constexpr uint32_t kPllInputHz = 4000000;  // PLL input after the M divider
constexpr uint32_t kHseTolerancePct = 3;   // HSI is ±1 %, crystals far better
constexpr uint32_t kCaptureEvents = 8;     // IC prescaler: one capture per 8 HSE/32 edges

// Measures HSE/32 with TIM16 clocked from HSI16 (the clock at this point).
bool hseMatchesExpected() {
  __HAL_RCC_TIM16_CLK_ENABLE();
  TIM16->CR1 = 0;
  TIM16->PSC = 0;
  TIM16->ARR = 0xFFFF;
  TIM16->TISEL = TIM_TISEL_TI1SEL_1 | TIM_TISEL_TI1SEL_0;       // TI1 = HSE/32
  TIM16->CCMR1 = TIM_CCMR1_CC1S_0 | (3u << TIM_CCMR1_IC1PSC_Pos);  // capture every 8 events
  TIM16->CCER = TIM_CCER_CC1E;
  TIM16->EGR = TIM_EGR_UG;
  TIM16->SR = 0;
  TIM16->CR1 = TIM_CR1_CEN;

  uint16_t captures[3] = {};
  bool ok = true;
  for (uint16_t& capture : captures) {
    uint32_t spin = 200000;  // a capture is due every ~16 µs; give up after ~50 ms
    while (!(TIM16->SR & TIM_SR_CC1IF) && --spin) {
    }
    if (spin == 0) {
      ok = false;
      break;
    }
    capture = static_cast<uint16_t>(TIM16->CCR1);  // reading clears CC1IF
  }

  TIM16->CR1 = 0;
  TIM16->TISEL = 0;
  __HAL_RCC_TIM16_CLK_DISABLE();
  if (!ok) return false;

  const uint32_t measured = static_cast<uint16_t>(captures[2] - captures[1]);
  const uint64_t expected = static_cast<uint64_t>(HSI_VALUE) * 32u * kCaptureEvents / board::kHseHz;
  const uint64_t tolerance = expected * kHseTolerancePct / 100u;
  return measured + tolerance >= expected && measured <= expected + tolerance;
}

bool configure(bool fromHse) {
  RCC_OscInitTypeDef osc{};
  osc.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI48;
  osc.HSIState = RCC_HSI_ON;
  osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  osc.HSI48State = RCC_HSI48_ON;
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = fromHse ? RCC_PLLSOURCE_HSE : RCC_PLLSOURCE_HSI;
  osc.PLL.PLLM = (fromHse ? board::kHseHz : HSI_VALUE) / kPllInputHz;  // 4 MHz into the PLL
  osc.PLL.PLLN = 85;                                                  // 340 MHz VCO
  osc.PLL.PLLP = RCC_PLLP_DIV2;
  osc.PLL.PLLQ = RCC_PLLQ_DIV2;
  osc.PLL.PLLR = RCC_PLLR_DIV2;                                       // 170 MHz SYSCLK
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) return false;

  RCC_ClkInitTypeDef clk{};
  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV1;
  clk.APB2CLKDivider = RCC_HCLK_DIV1;
  return HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_8) == HAL_OK;
}

bool startHse() {
  RCC_OscInitTypeDef osc{};
  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState = RCC_HSE_ON;
  osc.PLL.PLLState = RCC_PLL_NONE;
  return HAL_RCC_OscConfig(&osc) == HAL_OK;  // times out after HSE_STARTUP_TIMEOUT if absent
}

void stopHse() {
  RCC_OscInitTypeDef osc{};
  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState = RCC_HSE_OFF;
  osc.PLL.PLLState = RCC_PLL_NONE;
  (void)HAL_RCC_OscConfig(&osc);
}

void configureUsbClock() {
  RCC_PeriphCLKInitTypeDef usb{};
  usb.PeriphClockSelection = RCC_PERIPHCLK_USB;
  usb.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  if (HAL_RCCEx_PeriphCLKConfig(&usb) != HAL_OK) Error_Handler();

  __HAL_RCC_CRS_CLK_ENABLE();
  RCC_CRSInitTypeDef crs{};
  crs.Prescaler = RCC_CRS_SYNC_DIV1;
  crs.Source = RCC_CRS_SYNC_SOURCE_USB;
  crs.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  crs.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000, 1000);
  crs.ErrorLimitValue = RCC_CRS_ERRORLIMIT_DEFAULT;
  crs.HSI48CalibrationValue = RCC_CRS_HSI48CALIBRATION_DEFAULT;
  HAL_RCCEx_CRSConfig(&crs);
}

}  // namespace

namespace board {
bool g_runningOnCrystal = false;
}

// Replaces the variant's weak HSI-only version; called by the core before USB
// and setup(). A pending bootloader request is honoured before anything else.
extern "C" void SystemClock_Config(void) {
  boot_options::earlyBoot();
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  if (startHse() && hseMatchesExpected() && configure(true)) {
    board::g_runningOnCrystal = true;
    HAL_RCC_EnableCSS();  // a failing crystal raises an NMI (handled: reset, then HSI)
  } else {
    stopHse();
    if (!configure(false)) Error_Handler();
  }
  configureUsbClock();
}
