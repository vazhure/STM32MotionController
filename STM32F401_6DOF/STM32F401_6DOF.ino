// !!!!!! W.I.P. !!!!!
//
// 6DOF by Andrey Zhuravlev
// STM32F401_6DOF.ino
// 6DOF Stewart platform synchronized STEP/DIR controller
// STM32F401 + 6 stepper axes + USB serial protocol

#include "dma_stepper_hal.h"
#include <string.h>

// =============================================================================
// PIN MAP FOR STM32F401 BLACK PILL
// Adjust pins for your exact board/wiring.
// Avoid PA11/PA12 because they are USB on F401.
// =============================================================================

#define AXIS0_STEP   PA0
#define AXIS0_DIR    PA1
#define AXIS0_LIMIT  PA8

#define AXIS1_STEP   PA2
#define AXIS1_DIR    PA3
#define AXIS1_LIMIT  PA9

#define AXIS2_STEP   PA4
#define AXIS2_DIR    PA5
#define AXIS2_LIMIT  PA10

#define AXIS3_STEP   PA6
#define AXIS3_DIR    PA7
#define AXIS3_LIMIT  PB10

#define AXIS4_STEP   PB0
#define AXIS4_DIR    PB1
#define AXIS4_LIMIT  PB12

#define AXIS5_STEP   PB5
#define AXIS5_DIR    PB6
#define AXIS5_LIMIT  PB13

#define LED_PIN      PC13

// =============================================================================
// PROTOCOL CONSTANTS
// =============================================================================

#define SERIAL_BAUD_RATE 115200 

#define PKT_SYNC0 0xA5
#define PKT_SYNC1 0x5A

#define PKT_TARGET_FRAME 0x01
#define PKT_COMMAND      0x02
#define PKT_STATE        0x81

// Host commands, sent in data[0] of PKT_COMMAND
#define HOST_CMD_HOME         0
#define HOST_CMD_ENABLE       1
#define HOST_CMD_DISABLE      2
#define HOST_CMD_CLEAR_ALARM  3
#define HOST_CMD_SET_HOME     4
#define HOST_CMD_GET_STATE    5

// Optional automatic state reporting.
// Set to 0 if you only want on-demand GET_STATE.
#define AUTO_STATE_PERIOD_MS 20

// =============================================================================
// PACKETS
// =============================================================================

struct HostPacket {
  uint8_t sync0;
  uint8_t sync1;
  uint8_t type;
  uint8_t seq;
  uint32_t frame_us;
  int32_t data[NUM_AXES];
  uint16_t crc;
} __attribute__((packed));

#define HOST_PACKET_SIZE sizeof(HostPacket)

struct StatePacket {
  uint8_t sync0;
  uint8_t sync1;
  uint8_t type;
  uint8_t seq;

  uint8_t mode;
  uint8_t error;
  uint8_t queueCount;
  uint8_t flags;

  int32_t pos[NUM_AXES];
  int32_t target[NUM_AXES];

  uint16_t crc;
} __attribute__((packed));

#define STATE_PACKET_SIZE sizeof(StatePacket)

// flags bits
#define FLAG_HOMED          0x01
#define FLAG_SEGMENT_ACTIVE 0x02

// =============================================================================
// SERIAL RX STATE
// =============================================================================

static uint8_t rxBuf[HOST_PACKET_SIZE];
static size_t rxLen = 0;

static uint8_t txSeq = 0;
static uint32_t droppedFrames = 0;

// =============================================================================
// CRC16 CCITT
// =============================================================================

static uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;

  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;

    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x8000) {
        crc = (uint16_t)((crc << 1) ^ 0x1021);
      } else {
        crc = (uint16_t)(crc << 1);
      }
    }
  }

  return crc;
}

// =============================================================================
// STATE SENDING
// =============================================================================

void sendState() {
  StatePacket s;

  s.sync0 = PKT_SYNC0;
  s.sync1 = PKT_SYNC1;
  s.type = PKT_STATE;
  s.seq = txSeq++;

  s.mode = DMAStepper_GetMode();
  s.error = DMAStepper_GetError();
  s.queueCount = DMAStepper_GetQueueCount();

  s.flags = 0;
  if (DMAStepper_IsHomed()) s.flags |= FLAG_HOMED;
  if (DMAStepper_IsSegmentActive()) s.flags |= FLAG_SEGMENT_ACTIVE;

  DMAStepper_GetPositions(s.pos);
  DMAStepper_GetTargets(s.target);

  s.crc = crc16_ccitt((const uint8_t*)&s, STATE_PACKET_SIZE - sizeof(uint16_t));

  Serial.write((const uint8_t*)&s, STATE_PACKET_SIZE);
}

// =============================================================================
// PACKET HANDLING
// =============================================================================

void handlePacket(const HostPacket& pkt) {
  DMAStepper_NotifyValidPacket();

  if (pkt.type == PKT_TARGET_FRAME) {
    if (!DMAStepper_QueueFrame(pkt.data, pkt.frame_us, pkt.seq)) {
      droppedFrames++;
    }
  } else if (pkt.type == PKT_COMMAND) {
    int32_t cmd = pkt.data[0];

    switch (cmd) {
      case HOST_CMD_HOME:
        DMAStepper_StartHoming();
        break;

      case HOST_CMD_ENABLE:
        DMAStepper_Enable(true);
        break;

      case HOST_CMD_DISABLE:
        DMAStepper_Enable(false);
        break;

      case HOST_CMD_CLEAR_ALARM:
        DMAStepper_ClearAlarm();
        break;

      case HOST_CMD_SET_HOME:
        DMAStepper_SetHome();
        break;

      case HOST_CMD_GET_STATE:
        break;

      default:
        break;
    }

    sendState();
  }
}

void readSerial() {
  while (Serial.available()) {
    uint8_t b = (uint8_t)Serial.read();

    if (rxLen == 0) {
      if (b == PKT_SYNC0) {
        rxBuf[0] = b;
        rxLen = 1;
      }
    } else if (rxLen == 1) {
      if (b == PKT_SYNC1) {
        rxBuf[1] = b;
        rxLen = 2;
      } else if (b == PKT_SYNC0) {
        rxBuf[0] = b;
        rxLen = 1;
      } else {
        rxLen = 0;
      }
    } else {
      rxBuf[rxLen++] = b;

      if (rxLen == HOST_PACKET_SIZE) {
        rxLen = 0;

        HostPacket pkt;
        memcpy(&pkt, rxBuf, HOST_PACKET_SIZE);

        uint16_t expected = pkt.crc;
        uint16_t computed = crc16_ccitt((const uint8_t*)&pkt, HOST_PACKET_SIZE - sizeof(uint16_t));

        if (expected == computed) {
          handlePacket(pkt);
        }
      }
    }
  }
}

// =============================================================================
// SETUP / LOOP
// =============================================================================

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);

  uint32_t start = millis();
  while (!Serial && (millis() - start) < 3000) {
    // Wait up to 3 seconds for USB CDC.
  }

  pinMode(LED_PIN, OUTPUT);

  DMAStepper_Init();

  DMAStepper_InitAxis(0, AXIS0_STEP, AXIS0_DIR, AXIS0_LIMIT);
  DMAStepper_InitAxis(1, AXIS1_STEP, AXIS1_DIR, AXIS1_LIMIT);
  DMAStepper_InitAxis(2, AXIS2_STEP, AXIS2_DIR, AXIS2_LIMIT);
  DMAStepper_InitAxis(3, AXIS3_STEP, AXIS3_DIR, AXIS3_LIMIT);
  DMAStepper_InitAxis(4, AXIS4_STEP, AXIS4_DIR, AXIS4_LIMIT);
  DMAStepper_InitAxis(5, AXIS5_STEP, AXIS5_DIR, AXIS5_LIMIT);

  // Start in enabled but not homed state.
  DMAStepper_Enable(true);

  digitalWrite(LED_PIN, HIGH);
}

void loop() {
  readSerial();
  DMAStepper_Process();

  static uint32_t lastStateMs = 0;
  static uint32_t lastLedMs = 0;

  uint32_t now = millis();

#if AUTO_STATE_PERIOD_MS > 0
  if (now - lastStateMs >= AUTO_STATE_PERIOD_MS) {
    lastStateMs = now;
    sendState();
  }
#endif

  uint8_t mode = DMAStepper_GetMode();
  uint32_t ledPeriod = 1000;

  if (mode == MODE_ALARM) ledPeriod = 80;
  else if (mode == MODE_HOMING) ledPeriod = 250;
  else if (mode == MODE_RUNNING) ledPeriod = 500;

  if (now - lastLedMs >= ledPeriod) {
    lastLedMs = now;
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}