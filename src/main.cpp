#include <Arduino.h>
#include <ACANFD_STM32.h>
#include <ACANFD_STM32_Settings.h>

/* ============================== Pins ============================== */

static constexpr uint8_t PIN_CAN_RX = PB8;   // FDCAN1_RX (AF9)
static constexpr uint8_t PIN_CAN_TX = PB9;   // FDCAN1_TX (AF9)
static constexpr uint8_t PIN_STBY   = PA6;   // LOW = transceiver active
static constexpr uint8_t LED_TX     = PB0;
static constexpr uint8_t LED_RX     = PB1;

/* ============================ Protocol ============================ */

static constexpr uint8_t  MAX_PAYLOAD = 64;     // payload bytes (USB/CAN)
static constexpr uint32_t CAN_ID_MASK = 0x7FF;  // 11-bit standard

/* ============================ CAN-FD cfg =========================== */

static constexpr uint32_t NOMINAL_BITRATE = 500000;                     // 500 kbps
static constexpr auto     DATA_FACTOR      = DataBitRateFactor::x5;     // 2.5 Mbps

/* ============================ Utilities ============================ */

struct LedPulse {
  uint8_t pin{0};
  uint32_t offAt{0};
  void begin(uint8_t p) { pin = p; pinMode(pin, OUTPUT); digitalWrite(pin, LOW); }
  void pulse(uint16_t ms = 75) { digitalWrite(pin, HIGH); offAt = millis() + ms; }
  void service() { if (offAt && (int32_t)(millis() - offAt) >= 0) { digitalWrite(pin, LOW); offAt = 0; } }
};

static LedPulse ledTx, ledRx;

/* ========================= USB RX state =========================== */

enum class RxState : uint8_t { ID0, ID1, CMD, LEN, DATA };
static RxState usbState = RxState::ID0;
static uint8_t usbBuf[4 + MAX_PAYLOAD];
static uint8_t usbLen = 0;
static uint8_t usbPos = 0;

/* Map arbitrary length to the next valid CAN-FD payload size (0..64). */
static uint8_t toValidFdLen(uint8_t n) {
  static constexpr uint8_t steps[] = { 0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64 };
  for (uint8_t i = 0; i < sizeof(steps); ++i) {
    if (n <= steps[i]) return steps[i];
  }
  return 64;
}

/* ========================== USB -> CAN ============================ */

static void sendCanFromUsbFrame(const uint8_t* frame) {
  const uint16_t id  = (uint16_t)frame[0] | ((uint16_t)frame[1] << 8);
  const uint8_t  cmd = frame[2];
  const uint8_t  len = frame[3];
  const uint8_t* pay = &frame[4];

  uint8_t canData[1 + MAX_PAYLOAD] = {};
  canData[0] = cmd;
  memcpy(&canData[1], pay, len);

  const uint8_t canLen = (uint8_t)min<uint16_t>(1 + len, MAX_PAYLOAD + 1);
  const uint8_t fdLen  = toValidFdLen(canLen);

  CANFDMessage m;
  m.id  = (id & CAN_ID_MASK);
  m.len = fdLen;
  memset(m.data, 0, sizeof(m.data));
  memcpy(m.data, canData, canLen);

  if (fdcan1.tryToSendReturnStatusFD(m) == 0) {
    ledTx.pulse();
  }
}

/* ========================== CAN -> USB ============================ */

static void sendUsbFromCanFrame(const CANFDMessage& m) {
  if (m.len == 0) return;
  const uint8_t cmd = m.data[0];

  uint8_t payloadLen = (m.len > 1) ? (uint8_t)(m.len - 1) : 0;
  if (payloadLen > MAX_PAYLOAD) payloadLen = MAX_PAYLOAD;

  uint8_t out[4 + MAX_PAYLOAD];
  out[0] = (uint8_t)(m.id & 0xFF);
  out[1] = (uint8_t)((m.id >> 8) & 0xFF);
  out[2] = cmd;
  out[3] = payloadLen;
  memcpy(&out[4], &m.data[1], payloadLen);

  Serial.write(out, 4 + payloadLen);
  ledRx.pulse();
}

/* ======================== Service functions ======================= */

static void serviceUsbToCan() {
  while (Serial.available()) {
    const uint8_t b = (uint8_t)Serial.read();
    switch (usbState) {
      case RxState::ID0:
        usbBuf[0] = b; usbState = RxState::ID1; break;
      case RxState::ID1:
        usbBuf[1] = b; usbState = RxState::CMD; break;
      case RxState::CMD:
        usbBuf[2] = b; usbState = RxState::LEN; break;
      case RxState::LEN:
        usbLen = b;
        if (usbLen > MAX_PAYLOAD) { usbState = RxState::ID0; usbLen = 0; usbPos = 0; }
        else {
          usbBuf[3] = usbLen; usbPos = 0;
          if (usbLen == 0) { sendCanFromUsbFrame(usbBuf); usbState = RxState::ID0; }
          else { usbState = RxState::DATA; }
        }
        break;
      case RxState::DATA:
        usbBuf[4 + usbPos] = b;
        if (++usbPos >= usbLen) {
          sendCanFromUsbFrame(usbBuf);
          usbState = RxState::ID0; usbLen = 0; usbPos = 0;
        }
        break;
    }
  }
}

static void serviceCanToUsb() {
  CANFDMessage m;
  while (fdcan1.receiveFD0(m)) {
    sendUsbFromCanFrame(m);
  }
}

/* ============================ Arduino ============================= */

void setup() {
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, LOW);

  ledTx.begin(LED_TX);
  ledRx.begin(LED_RX);

  Serial.begin(115200);
  while (!Serial) {
    digitalWrite(LED_TX, (millis() & 0x200) ? HIGH : LOW);
    digitalWrite(LED_RX, (millis() & 0x200) ? LOW  : HIGH);
  }

  ACANFD_STM32_Settings settings(NOMINAL_BITRATE, DATA_FACTOR);
  settings.mModuleMode = ACANFD_STM32_Settings::NORMAL_FD;
  settings.mRxPin = PIN_CAN_RX;
  settings.mTxPin = PIN_CAN_TX;

  const uint32_t err = fdcan1.beginFD(settings);
  if (err != 0) {
    Serial.print("FDCAN init error: 0x"); Serial.println(err, HEX);
  } else {
    Serial.println("FDCAN initialized");
  }
}

void loop() {
  serviceUsbToCan();
  serviceCanToUsb();
  ledTx.service();
  ledRx.service();
}