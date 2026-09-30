#include "drivers/CanBus.h"

#include <Arduino.h>

#include <cstring>

#include "board/Board.h"
#include "bridge/AdapterProtocol.h"
#include "bridge/Framing.h"

static_assert(board::kCanRxPin == PB8 && board::kCanTxPin == PB9, "FDCAN1 is wired to PB8/PB9 (AF9)");

namespace can_bus {
namespace {

// Bit timing for a 170 MHz FDCAN kernel clock (PCLK1), identical to the S1:
// 85 MHz time quanta; arbitration 1+126+43 tq, data 1+24+9 tq; TDC offset half a data bit.
constexpr uint32_t kKernelClock = 170000000;
constexpr uint32_t kPrescaler = 2;
constexpr uint32_t kNominalSeg1 = 126, kNominalSeg2 = 43;
constexpr uint32_t kDataSeg1 = 24, kDataSeg2 = 9;
constexpr uint32_t kTdcOffset = 34;
constexpr uint32_t kBusOffRetryMs = 50;
constexpr uint8_t kLecNoChange = 7;

constexpr uint8_t kSlots = 64;  // power of two, both rings
FDCAN_HandleTypeDef g_can{};

CanFrame g_rx[kSlots];
volatile uint8_t g_rxHead = 0;  // interrupt
volatile uint8_t g_rxTail = 0;  // main loop

struct Queued {
  CanFrame frame;
  uint32_t arrivedMs;
};
Queued g_tx[kSlots];
uint8_t g_txHead = 0, g_txTail = 0;  // main loop only

uint32_t g_inFlightBit = 0;  // hardware buffer holding the queue head, 0 = none

Stats g_stats{};
volatile uint32_t g_rxDroppedIrq = 0;
uint32_t g_lastRecoveryMs = 0;
bool g_wasBusOff = false;
uint32_t g_psr = 0;          // sampled once per service() (reading PSR resets LEC)
uint8_t g_lastError = 0;

bool submit(const CanFrame& frame) {
  uint8_t data[64] = {};
  std::memcpy(data, frame.data, frame.length);
  FDCAN_TxHeaderTypeDef header{};
  header.Identifier = frame.id & framing::kMaxCanId;
  header.IdType = FDCAN_STANDARD_ID;
  header.TxFrameType = FDCAN_DATA_FRAME;
  header.DataLength = framing::dlcFor(frame.length);
  header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  header.BitRateSwitch = FDCAN_BRS_ON;
  header.FDFormat = FDCAN_FD_CAN;
  header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  if (HAL_FDCAN_AddMessageToTxFifoQ(&g_can, &header, data) != HAL_OK) return false;
  g_inFlightBit = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&g_can);
  return true;
}

void popTx() { g_txTail = static_cast<uint8_t>((g_txTail + 1) & (kSlots - 1)); }

}  // namespace

bool begin(uint32_t irqPriority) {
  if (HAL_RCC_GetPCLK1Freq() != kKernelClock) return false;  // bit timing assumes 170 MHz

  // Transceiver out of standby.
  pinMode(board::kCanStandbyPin, OUTPUT);
  digitalWrite(board::kCanStandbyPin, LOW);

  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitTypeDef pins{};
  pins.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  pins.Mode = GPIO_MODE_AF_PP;
  pins.Pull = GPIO_NOPULL;
  pins.Speed = GPIO_SPEED_FREQ_HIGH;
  pins.Alternate = GPIO_AF9_FDCAN1;
  HAL_GPIO_Init(GPIOB, &pins);

  RCC_PeriphCLKInitTypeDef clock{};
  clock.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
  clock.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&clock) != HAL_OK) return false;
  __HAL_RCC_FDCAN_CLK_ENABLE();

  g_can.Instance = FDCAN1;
  FDCAN_InitTypeDef& init = g_can.Init;
  init.ClockDivider = FDCAN_CLOCK_DIV1;
  init.FrameFormat = FDCAN_FRAME_FD_BRS;
  init.Mode = FDCAN_MODE_NORMAL;
  init.AutoRetransmission = DISABLE;  // single-shot: service() retries within the frame's lifetime
  init.TransmitPause = DISABLE;
  init.ProtocolException = DISABLE;
  init.NominalPrescaler = kPrescaler;
  init.NominalSyncJumpWidth = kNominalSeg2;
  init.NominalTimeSeg1 = kNominalSeg1;
  init.NominalTimeSeg2 = kNominalSeg2;
  init.DataPrescaler = kPrescaler;
  init.DataSyncJumpWidth = kDataSeg2;
  init.DataTimeSeg1 = kDataSeg1;
  init.DataTimeSeg2 = kDataSeg2;
  init.StdFiltersNbr = 0;
  init.ExtFiltersNbr = 0;
  init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&g_can) != HAL_OK) return false;

  if (HAL_FDCAN_ConfigGlobalFilter(&g_can, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT, FDCAN_REJECT_REMOTE,
                                   FDCAN_REJECT_REMOTE) != HAL_OK)
    return false;
  if (HAL_FDCAN_ConfigTxDelayCompensation(&g_can, kTdcOffset, 0) != HAL_OK) return false;
  if (HAL_FDCAN_EnableTxDelayCompensation(&g_can) != HAL_OK) return false;
  if (HAL_FDCAN_ActivateNotification(&g_can, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_MESSAGE_LOST, 0) !=
      HAL_OK)
    return false;

  HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, irqPriority, 0);
  HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  return HAL_FDCAN_Start(&g_can) == HAL_OK;
}

bool send(const CanFrame& frame, uint32_t nowMs) {
  const uint8_t next = static_cast<uint8_t>((g_txHead + 1) & (kSlots - 1));
  if (next == g_txTail || frame.length > 64) {
    ++g_stats.txDropped;
    return false;
  }
  g_tx[g_txHead] = {frame, nowMs};
  g_txHead = next;
  return true;
}

uint8_t txFree() { return static_cast<uint8_t>((g_txTail - g_txHead - 1) & (kSlots - 1)); }

const CanFrame* peek() {
  const uint8_t tail = g_rxTail;
  if (tail == g_rxHead) return nullptr;
  __DMB();  // read the slot only after seeing the head that published it
  return &g_rx[tail];
}

void pop() {
  __DMB();
  g_rxTail = static_cast<uint8_t>((g_rxTail + 1) & (kSlots - 1));
  ++g_stats.received;
}

void service(uint32_t nowMs) {
  g_psr = FDCAN1->PSR;
  const uint8_t lec = static_cast<uint8_t>((g_psr & FDCAN_PSR_LEC) >> FDCAN_PSR_LEC_Pos);
  if (lec != kLecNoChange) g_lastError = lec;

  // 1. The frame in the controller: delivered, or failed (single-shot) and retried.
  const bool waiting = g_inFlightBit && (FDCAN1->TXBRP & g_inFlightBit);
  if (g_inFlightBit && !waiting) {
    const bool delivered = FDCAN1->TXBTO & g_inFlightBit;  // read after TXBRP cleared
    g_inFlightBit = 0;
    if (delivered) {
      ++g_stats.delivered;
      popTx();
    }
    // Not delivered: the head stays queued; step 2 retries it or lets it expire.
  }

  // 2. Next frame: drop what is too old, then hand the head to the controller.
  if (!waiting) {
    while (g_txTail != g_txHead && nowMs - g_tx[g_txTail].arrivedMs > kTxLifetimeMs) {
      ++g_stats.txDropped;
      popTx();
    }
    if (g_txTail != g_txHead && !(g_psr & FDCAN_PSR_BO)) (void)submit(g_tx[g_txTail].frame);
  }

  // 3. Bus-off: the FDCAN parks itself in INIT; leaving INIT starts the
  // standard recovery (128 x 11 recessive bits) in hardware.
  const bool busOff = g_psr & FDCAN_PSR_BO;
  if (busOff && !g_wasBusOff) ++g_stats.busOffEvents;
  g_wasBusOff = busOff;
  if (busOff && nowMs - g_lastRecoveryMs >= kBusOffRetryMs) {
    g_lastRecoveryMs = nowMs;
    CLEAR_BIT(FDCAN1->CCCR, FDCAN_CCCR_INIT);
  }

  g_stats.rxDropped = g_rxDroppedIrq;
}

const Stats& stats() { return g_stats; }

void resetStats() {
  g_stats = Stats{};
  g_rxDroppedIrq = 0;
  g_lastError = 0;
}

BusStatus status() {
  const uint32_t ecr = FDCAN1->ECR;
  BusStatus s;
  if (g_psr & FDCAN_PSR_BO)
    s.state = static_cast<uint8_t>(adapter::BusState::BusOff);
  else if (g_psr & FDCAN_PSR_EP)
    s.state = static_cast<uint8_t>(adapter::BusState::ErrorPassive);
  else if (g_psr & FDCAN_PSR_EW)
    s.state = static_cast<uint8_t>(adapter::BusState::Warning);
  else
    s.state = static_cast<uint8_t>(adapter::BusState::ErrorActive);
  s.tec = static_cast<uint8_t>((ecr & FDCAN_ECR_TEC) >> FDCAN_ECR_TEC_Pos);
  s.rec = static_cast<uint8_t>((ecr & FDCAN_ECR_REC) >> FDCAN_ECR_REC_Pos);
  s.lastError = g_lastError;
  return s;
}

}  // namespace can_bus

extern "C" {

void FDCAN1_IT0_IRQHandler(void) { HAL_FDCAN_IRQHandler(&can_bus::g_can); }

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* handle, uint32_t interrupts) {
  using namespace can_bus;
  if (interrupts & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) g_rxDroppedIrq = g_rxDroppedIrq + 1;

  FDCAN_RxHeaderTypeDef header;
  while (HAL_FDCAN_GetRxFifoFillLevel(handle, FDCAN_RX_FIFO0) > 0) {
    const uint8_t head = g_rxHead;
    const uint8_t next = static_cast<uint8_t>((head + 1) & (kSlots - 1));
    if (next == g_rxTail) {  // ring full: the host is not keeping up
      uint8_t scratch[64];
      HAL_FDCAN_GetRxMessage(handle, FDCAN_RX_FIFO0, &header, scratch);
      g_rxDroppedIrq = g_rxDroppedIrq + 1;
      continue;
    }
    CanFrame& frame = g_rx[head];
    if (HAL_FDCAN_GetRxMessage(handle, FDCAN_RX_FIFO0, &header, frame.data) != HAL_OK) break;
    frame.id = static_cast<uint16_t>(header.Identifier);
    frame.length = framing::kFdLengths[header.DataLength & 0x0Fu];
    // Classic CAN: DLC 9..15 still means 8 bytes.
    if (header.FDFormat == FDCAN_CLASSIC_CAN && frame.length > 8) frame.length = 8;
    __DMB();
    g_rxHead = next;
  }
}

}  // extern "C"
