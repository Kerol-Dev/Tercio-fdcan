#include <Arduino.h>
#include <ACANFD_STM32.h>
#include <ACANFD_STM32_Settings.h>

// ---------------- Hardware pins ----------------
static constexpr uint8_t PIN_CAN_RX  = PB8;  // FDCAN1_RX (AF9)
static constexpr uint8_t PIN_CAN_TX  = PB9;  // FDCAN1_TX (AF9)
static constexpr uint8_t PIN_STBY    = PA6;  // LOW = transceiver active
static constexpr uint8_t LED_TX      = PB0;  // blink on successful TX
static constexpr uint8_t LED_RX      = PB1;  // blink on successful RX

// ---------------- Bridge protocol (USB <-> CAN) ----------------
// Frame: ID_L, ID_H, CMD, LEN, PAYLOAD[LEN], where 0 <= LEN <= 64
static constexpr uint8_t  MAX_PAYLOAD  = 64;
static constexpr uint32_t CAN_ID_MASK  = 0x7FF; // standard ID (11-bit); we accept 0..0xFFFF but will mask to 11-bit by default

// ---------------- CAN-FD config ----------------
static constexpr uint32_t NOMINAL_BITRATE = 500000; // 500 kbps
static constexpr auto     DATA_FACTOR      = DataBitRateFactor::x5; // 2.5 Mbps

// ---------------- Globals ----------------

// LED pulse (non-blocking)
struct LedPulse {
  uint8_t pin;
  uint32_t offAt = 0;
  void begin(uint8_t p) { pin = p; pinMode(pin, OUTPUT); digitalWrite(pin, LOW); }
  void pulse(uint16_t ms = 75) { digitalWrite(pin, HIGH); offAt = millis() + ms; }
  void service() { if (offAt && (int32_t)(millis() - offAt) >= 0) { digitalWrite(pin, LOW); offAt = 0; } }
} ledTx, ledRx;

// --------------- USB RX state machine ---------------
enum class RxState : uint8_t { ID0, ID1, CMD, LEN, DATA };
static RxState usbState = RxState::ID0;
static uint8_t usbBuf[4 + MAX_PAYLOAD];
static uint8_t usbLen   = 0;  // expected payload length (LEN)
static uint8_t usbPos   = 0;  // how many payload bytes collected so far

// Map arbitrary length to CAN-FD DLC (0..64, but we clamp to 0..32 here)
static uint8_t lenToDLC(uint8_t n) {
  // CAN FD DLC mapping steps
  static const uint8_t steps[] = { 0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64 };
  for (uint8_t i = 0; i < sizeof(steps); ++i) {
    if (n <= steps[i]) return steps[i];
  }
  return 32; // clamp
}

// --------------- USB -> CAN sender ---------------
static void sendCanFromUsbFrame(const uint8_t *frame) {
  const uint16_t id   = (uint16_t)frame[0] | ((uint16_t)frame[1] << 8);
  const uint8_t  cmd  = frame[2];
  const uint8_t  len  = frame[3];
  const uint8_t *pay  = &frame[4];

  // Build CAN-FD payload: [CMD][PAY...]
  uint8_t canData[1 + MAX_PAYLOAD];
  canData[0] = cmd;
  memcpy(&canData[1], pay, len);
  const uint8_t canLen = (uint8_t)min<uint16_t>(1 + len, MAX_PAYLOAD + 1);

  CANFDMessage m;
  m.id  = (id & CAN_ID_MASK); // use standard 11-bit; change if you want extended IDs
  m.len = lenToDLC(canLen);

  // Zero the buffer and copy, so padding bytes are deterministic
  memset(m.data, 0, sizeof(m.data));
  memcpy(m.data, canData, canLen);

  const uint32_t st = fdcan1.tryToSendReturnStatusFD(m);
  if (st == 0) {
    ledTx.pulse();
  } else {
    // TX error
    Serial.print("CAN TX error 0x"); Serial.println(st, HEX);
  }
}

// --------------- CAN -> USB sender ---------------
static void sendUsbFromCanFrame(const CANFDMessage &m) {
  if (m.len == 0) return; // need at least CMD
  const uint8_t cmd = m.data[0];

  // Payload is rest after CMD
  uint8_t payloadLen = (m.len > 1) ? (uint8_t)(m.len - 1) : 0;
  if (payloadLen > MAX_PAYLOAD) payloadLen = MAX_PAYLOAD;

  uint8_t out[4 + MAX_PAYLOAD];
  out[0] = (uint8_t)(m.id & 0xFF);        // ID_L
  out[1] = (uint8_t)((m.id >> 8) & 0xFF); // ID_H
  out[2] = cmd;
  out[3] = payloadLen;
  memcpy(&out[4], &m.data[1], payloadLen);

  // Write to USB CDC in one shot (non-blocking in Arduino core)
  Serial.write(out, 4 + payloadLen);
  ledRx.pulse();
}

// --------------- Process incoming USB bytes ---------------
static void serviceUsbToCan() {
  while (Serial.available()) {
    const uint8_t b = (uint8_t)Serial.read();
    switch (usbState) {
      case RxState::ID0:
        usbBuf[0] = b;
        usbState = RxState::ID1;
        break;

      case RxState::ID1:
        usbBuf[1] = b;
        usbState = RxState::CMD;
        break;

      case RxState::CMD:
        usbBuf[2] = b;
        usbState = RxState::LEN;
        break;

      case RxState::LEN:
        usbLen = b;
        if (usbLen > MAX_PAYLOAD) {
          // invalid length, reset state
          usbState = RxState::ID0;
          usbLen   = 0;
          usbPos   = 0;
        } else {
          usbBuf[3] = usbLen;
          usbPos = 0;
          if (usbLen == 0) {
            // Complete frame with empty payload
            sendCanFromUsbFrame(usbBuf);
            usbState = RxState::ID0;
          } else {
            usbState = RxState::DATA;
          }
        }
        break;

      case RxState::DATA:
        usbBuf[4 + usbPos] = b;
        usbPos++;
        if (usbPos >= usbLen) {
          // Full frame ready
          sendCanFromUsbFrame(usbBuf);
          usbState = RxState::ID0;
          usbLen   = 0;
          usbPos   = 0;
        }
        break;
    }
  }
}

// --------------- Process incoming CAN frames ---------------
static void serviceCanToUsb() {
  CANFDMessage m;
  // Drain FIFO quickly
  while (fdcan1.receiveFD0(m)) {
    sendUsbFromCanFrame(m);
  }
}

// ---------------- Arduino setup/loop ----------------
void setup() {
  // Pins
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, LOW); // enable transceiver
  ledTx.begin(LED_TX);
  ledRx.begin(LED_RX);

  // USB CDC
  Serial.begin(115200);
  while (!Serial) {
    digitalWrite(LED_TX, millis() & 0x200 ? HIGH : LOW);
    digitalWrite(LED_RX, millis() & 0x200 ? LOW : HIGH);
   }

  // CAN-FD init
  ACANFD_STM32_Settings settings(NOMINAL_BITRATE, DATA_FACTOR);
  settings.mModuleMode = ACANFD_STM32_Settings::NORMAL_FD;
  settings.mRxPin = PIN_CAN_RX;
  settings.mTxPin = PIN_CAN_TX;

  uint32_t err = fdcan1.beginFD(settings);
  if (err != 0) {
    Serial.print("FDCAN init error: 0x"); Serial.println(err, HEX);
    // keep running; USB bridge will still accept input even if CAN failed
  }

  Serial.println("USB<->CAN-FD bridge ready");
  Serial.println("Frame: ID_L, ID_H, CMD, LEN (0..32), PAYLOAD[LEN]");
}

void loop() {
  // Move bytes both ways
  serviceUsbToCan();
  serviceCanToUsb();

  // Non-blocking LED housekeeping
  ledTx.service();
  ledRx.service();
}