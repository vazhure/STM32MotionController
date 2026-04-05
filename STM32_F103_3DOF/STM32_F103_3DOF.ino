/*
 * =============================================================================
 * 🛠️ COMPILATION & SETUP INSTRUCTIONS
 * =============================================================================
 * 1. Arduino IDE Configuration:
 *    - Open Preferences -> Additional Boards Manager URLs:
 *      https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json
 *    - Board Manager -> Install "STM32 MCU based boards" by STMicroelectronics (v2.5.0+).
 *    - Select Board: "Generic STM32F1 series"
 *    - Board part number: "Blue Pill F103C8" (or F103CB)
 *    - Upload method: "STLink" (recommended) or "Serial"
 *    - CPU Speed: "72 MHz (Normal)"
 *    - Note: This code uses legacy libmaple headers. If compilation fails, 
 *      switch Board variant to "STM32F1xx/Maple (Roger's core)" or replace 
 *      #include <libmaple/gpio.h> with #include <Arduino.h> in dma_stepper_hal.cpp.
 *
 * 2. Required Libraries:
 *    - No external libraries required. Uses built-in STM32 Arduino core.
 *
 * 3. ST-Link Flashing Utilities & Drivers:
 *    - Windows GUI: STM32CubeProgrammer
 *      https://www.st.com/en/development-tools/stm32cubeprog.html
 *    - Legacy ST-Link Utility:
 *      https://www.st.com/en/development-tools/stsw-link004.html
 *    - USB Driver (Windows libusb/winusb fix): Zadig
 *      https://zadig.akeo.ie/
 *    - Linux/macOS: Install openocd or stlink-tools
 *      sudo apt install openocd stlink-tools  (Debian/Ubuntu)
 *      brew install open-ocd stlink-tools     (macOS)
 *
 * 4. Hardware Wiring Notes:
 *    - STEP+, DIR+ connect to AXIS(N)_STEP / AXIS(N)_DIR pins.
 *    - STEP-, DIR- connect to GND.
 *    - Limit switches: Use Normally Closed (NC) switches between AXIS(N)_LIMIT and GND.
 *      (Logic: LOW = idle, HIGH = triggered via internal pull-up)
 * =============================================================================
 */

// 3DOF by Andrey Zhuravlev
// v.azhure@gmail.com
// Discord: https://discord.gg/ynHCkrsmMA
#include "dma_stepper_hal.h"
#include <EEPROM.h>
#include <string.h>
#define SERIAL_BAUD_RATE 115200

// =============================================================================
// PIN ASSIGNMENTS (AXIS3_LIMIT moved to PB12 to avoid USB conflict on PA11)
// =============================================================================
#define AXIS0_STEP PA0
#define AXIS0_DIR PA1
#define AXIS0_LIMIT PB12 //PA8
#define AXIS1_STEP PA2
#define AXIS1_DIR PA3
#define AXIS1_LIMIT PA8 //PA9
#define AXIS2_STEP PA4
#define AXIS2_DIR PA5
#define AXIS2_LIMIT PA9 // PA10
#define AXIS3_STEP PA6
#define AXIS3_DIR PA7
#define AXIS3_LIMIT PA10// PB12
#define LED_PIN PB2  //PC13

// Limit Pins -> GND, NC (use normally closed switches)
// STEP-, DIR- -> GND
// STEP+, DIR+ -> AXIS(N)_STEP, AXIS(N)_DIR
#define CMD_ID 0
#define PCCMD_SIZE 20
#define STATE_SIZE 20
#define PID_STATE_SIZE 20

enum COMMAND : uint8_t {
  CMD_HOME = 0,
  CMD_MOVE = 1,
  CMD_SET_SPEED = 2,
  CMD_DISABLE = 3,
  CMD_ENABLE = 4,
  CMD_GET_STATE = 5,
  CMD_CLEAR_ALARM = 6,
  CMD_PARK = 7,
  SET_ALARM = 8,
  CMD_SET_LOW_SPEED = 9,
  CMD_SET_ACCEL = 10,
  CMD_MOVE_SH = 11,
  CMD_SET_PID_KP = 0x0C,
  CMD_SET_PID_KI = 0x0D,
  CMD_SET_PID_KD = 0x0E,
  CMD_SET_PID_KS = 0x0F,
  CMD_SET_PID_ENABLE = 0x10,
  CMD_SET_PID_BLEND = 0x11,
  CMD_GET_PID_STATE = 0x12,
  CMD_STORE_PID = 0x13,
  CMD_RESTORE_PID = 0x14
};

struct PCCMD {
  uint8_t header;
  uint8_t len;
  COMMAND cmd;
  uint8_t reserved;
  int32_t data[NUM_AXES];
} __attribute__((packed));


// FOR SimHub
struct PCCMD_SH {
  uint8_t header = 0;
  uint8_t len;  // len
  COMMAND cmd;
  uint8_t reserved;
  uint16_t data[NUM_AXES];
  uint16_t data2[NUM_AXES];  // EMPTY
} __attribute__((packed));

struct STATE {
  uint8_t mode;
  uint8_t flags;
  uint16_t speedMMperSEC;
  int32_t currentpos;
  int32_t targetpos;
  int32_t min;
  int32_t max;
} __attribute__((packed));

struct PID_STATE {
  uint8_t version;
  uint8_t flags;
  uint16_t blend;
  float Kp, Ki, Kd, Ks;
} __attribute__((packed));

enum PID_FLAGS : uint8_t {
  PID_NONE = 0,
  PID_ENABLED = 1,
  PID_MASTER_SYNC = 1 << 1,
  PID_DIAG_ENABLED = 1 << 2,  // Diagnostics: set false after validation if SimHub parsing is affected.
};

PCCMD pccmd;
PCCMD_SH& pccmd_sh = *(PCCMD_SH*)&pccmd;

const int RAW_DATA_LEN = sizeof(PCCMD);

volatile bool dataReceived = false;
uint8_t rxBuffer[RAW_DATA_LEN * 2];
int rxOffset = 0;

PID_STATE pidGlobal = { .version = 1, .flags = PID_FLAGS::PID_ENABLED, .blend = 100, .Kp = 15.0f, .Ki = 0.0f, .Kd = 0.02f, .Ks = 0.30f };

const int PID_STATE_LEN = sizeof(PID_STATE);
#define PID_EEPROM_MAGIC 0x5A
#define PID_EEPROM_ADDR 0

void savePidToEEPROM() {
  EEPROM.update(PID_EEPROM_ADDR, PID_EEPROM_MAGIC);
  uint8_t* p = (uint8_t*)&pidGlobal;
  for (int i = 0; i < PID_STATE_LEN; i++) EEPROM.update(PID_EEPROM_ADDR + 1 + i, p[i]);
}

bool loadPidFromEEPROM() {
  if (EEPROM.read(PID_EEPROM_ADDR) != PID_EEPROM_MAGIC) return false;
  uint8_t* p = (uint8_t*)&pidGlobal;
  for (int i = 0; i < PID_STATE_LEN; i++) p[i] = EEPROM.read(PID_EEPROM_ADDR + 1 + i);
  return true;
}

void sendAllStates() {
  for (int i = 0; i < NUM_AXES; i++) {
    AxisState* ax = DMAStepper_GetAxis(i);
    if (!ax) continue;
    STATE s = { ax->mode, (uint8_t)(ax->homed ? 0x02 : 0), (uint16_t)ax->maxSpeedMM, ax->currentPosition, ax->targetPosition, ax->minPos, ax->maxPos };
    Serial.write(10 + i);
    Serial.write(STATE_SIZE);
    Serial.write((uint8_t*)&s, STATE_SIZE);
  }
}

void sendPIDState() {
  Serial.write(255);
  Serial.write(PID_STATE_SIZE);
  Serial.write((uint8_t*)&pidGlobal, PID_STATE_SIZE);
}

void processCommand() {
  switch (pccmd.cmd) {
    case CMD_MOVE:
      for (int i = 0; i < NUM_AXES; i++) {
        AxisState* ax = DMAStepper_GetAxis(i);
        if (ax && ax->homed && ax->mode == MODE_READY) DMAStepper_SetTarget(i, pccmd.data[i]);
      }
      break;
    case CMD_MOVE_SH:
      for (int i = 0; i < NUM_AXES; i++) {
        uint16_t val = pccmd_sh.data[i];
        val = (val >> 8) | (val << 8);
        AxisState* ax = DMAStepper_GetAxis(i);
        if (ax && ax->homed && ax->mode == MODE_READY) {
          int32_t target = map(val, 0, 65535, ax[i].minPos, ax[i].maxPos);
          DMAStepper_SetTarget(i, target);
        }
      }
      break;
    case CMD_SET_SPEED:
      for (int i = 0; i < NUM_AXES; i++) DMAStepper_SetMaxSpeed(i, (uint32_t)pccmd.data[0]);
      break;
    case CMD_SET_ACCEL:
      for (int i = 0; i < NUM_AXES; i++) {
        AxisState* ax = DMAStepper_GetAxis(i);
        if (ax) ax->maxAccel = pccmd.data[0];
      }
      break;
    case CMD_ENABLE:
      for (int i = 0; i < NUM_AXES; i++) {
        AxisState* ax = DMAStepper_GetAxis(i);
        if (ax) ax->mode = ax->homed ? MODE_READY : MODE_CONNECTED;
      }
      break;
    case CMD_DISABLE:
      DMAStepper_StopAll();
      for (int i = 0; i < NUM_AXES; i++) {
        AxisState* ax = DMAStepper_GetAxis(i);
        if (ax) ax->mode = MODE_DISABLED;
      }
      break;
    case CMD_CLEAR_ALARM: DMAStepper_ClearAlarm(); break;
    case CMD_HOME:
      // Start homing sequence
      for (int i = 0; i < NUM_AXES; i++) {
        AxisState* ax = DMAStepper_GetAxis(i);
        if (!ax) continue;
        if (ax->homed && ax->mode != MODE_ALARM) {
          continue;  // Skip if already homed and not in alarm
        }
        ax->mode = MODE_HOMING;
      }
      break;
    case CMD_PARK:
      for (int i = 0; i < NUM_AXES; i++) {
        AxisState* ax = DMAStepper_GetAxis(i);
        if (ax && ax->homed) { ax->mode = MODE_PARKING; }
      }
      break;
    case SET_ALARM:
      DMAStepper_StopAll();
      for (int i = 0; i < NUM_AXES; i++) {
        AxisState* ax = DMAStepper_GetAxis(i);
        if (ax) {
          ax->mode = MODE_ALARM;
          ax->homed = false;
        }
      }
      break;
    case CMD_GET_STATE: sendAllStates(); break;
    case CMD_SET_PID_KP:
      pidGlobal.Kp = constrain((float)pccmd.data[0] / 10.0f, 0.0f, 200.0f);
      for (int i = 0; i < NUM_AXES; i++) DMAStepper_SetPID(i, pidGlobal.Kp, pidGlobal.Ki, pidGlobal.Kd, pidGlobal.Ks);
      break;
    case CMD_SET_PID_KI:
      pidGlobal.Ki = constrain((float)pccmd.data[0] / 10.0f, 0.0f, 50.0f);
      for (int i = 0; i < NUM_AXES; i++) DMAStepper_SetPID(i, pidGlobal.Kp, pidGlobal.Ki, pidGlobal.Kd, pidGlobal.Ks);
      break;
    case CMD_SET_PID_KD:
      pidGlobal.Kd = constrain((float)pccmd.data[0] / 100.0f, 0.0f, 50.0f);
      for (int i = 0; i < NUM_AXES; i++) DMAStepper_SetPID(i, pidGlobal.Kp, pidGlobal.Ki, pidGlobal.Kd, pidGlobal.Ks);
      break;
    case CMD_SET_PID_KS:
      pidGlobal.Ks = constrain((float)pccmd.data[0] / 100.0f, 0.0f, 1.0f);
      for (int i = 0; i < NUM_AXES; i++) DMAStepper_SetPID(i, pidGlobal.Kp, pidGlobal.Ki, pidGlobal.Kd, pidGlobal.Ks);
      break;
    case CMD_SET_PID_ENABLE:
      for (int i = 0; i < NUM_AXES; i++) DMAStepper_SetPIDEnable(i, pccmd.data[0] != 0);
      if (pccmd.data[0]) pidGlobal.flags |= PID_FLAGS::PID_ENABLED;
      else pidGlobal.flags &= ~PID_FLAGS::PID_ENABLED;
      break;
    case CMD_SET_PID_BLEND:
      pidGlobal.blend = constrain((uint16_t)pccmd.data[0], 0, 100);
      for (int i = 0; i < NUM_AXES; i++) DMAStepper_SetPIDBlend(i, (float)pidGlobal.blend / 100.0f);
      break;
    case CMD_GET_PID_STATE: sendPIDState(); break;
    case CMD_STORE_PID:
      savePidToEEPROM();
      break;
    case CMD_RESTORE_PID:
      if (loadPidFromEEPROM()) {
        bool pidWasEnabled = (pidGlobal.flags & PID_FLAGS::PID_ENABLED) != 0;
        // Stop PID on all axes, apply new coefficients
        for (int i = 0; i < NUM_AXES; i++) {
          DMAStepper_SetPIDEnable(i, false);  // Stop PID
          DMAStepper_SetPID(i, pidGlobal.Kp, pidGlobal.Ki, pidGlobal.Kd, pidGlobal.Ks);
          DMAStepper_SetPIDBlend(i, (float)pidGlobal.blend / 100.0f);
          // If PID was globally enabled, restore operation (integral reset happens inside HAL)
          if (pidWasEnabled) {
            DMAStepper_SetPIDEnable(i, true);
          }
        }
      }
      break;
    default: break;
  }
}

void setup() {
  for (int pin = PA0; pin <= PC15; pin++) pinMode(pin, INPUT_PULLDOWN);
  pinMode(LED_PIN, OUTPUT);
  loadPidFromEEPROM();
  DMAStepper_Init();
  DMAStepper_InitAxis(0, AXIS0_STEP, AXIS0_DIR, AXIS0_LIMIT);
  DMAStepper_InitAxis(1, AXIS1_STEP, AXIS1_DIR, AXIS1_LIMIT);
  DMAStepper_InitAxis(2, AXIS2_STEP, AXIS2_DIR, AXIS2_LIMIT);
  DMAStepper_InitAxis(3, AXIS3_STEP, AXIS3_DIR, AXIS3_LIMIT);
  for (int i = 0; i < NUM_AXES; i++) {
    DMAStepper_SetPID(i, pidGlobal.Kp, pidGlobal.Ki, pidGlobal.Kd, pidGlobal.Ks);
    DMAStepper_SetPIDBlend(i, (float)pidGlobal.blend / 100.0f);
    bool pidEnabled = (pidGlobal.flags & PID_FLAGS::PID_ENABLED) != 0;
    DMAStepper_SetPIDEnable(i, pidEnabled);
  }
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial)
    ;
  digitalWrite(LED_PIN, HIGH);
}

inline void serialEvent() {
  int data_cnt = min(Serial.available(), PCCMD_SIZE);
  if (data_cnt < 2)
    return;
  for (int t = 0; t < data_cnt; ++t) {
    int byte = Serial.read();
    if (rxOffset > 0) {
      rxBuffer[rxOffset++] = byte;
      if (rxOffset == PCCMD_SIZE) {
        memcpy(&pccmd, rxBuffer, PCCMD_SIZE);
        dataReceived = true;
        rxOffset = 0;
      }
    } else {
      if (byte == CMD_ID) {
        if (Serial.peek() == PCCMD_SIZE) {
          rxBuffer[rxOffset++] = CMD_ID;
        }
      }
    }
  }
}


void loop() {
  serialEvent();

  if (dataReceived) {
    dataReceived = false;
    digitalWrite(LED_PIN, LOW);
    processCommand();
    digitalWrite(LED_PIN, HIGH);
  }
  DMAStepper_Process();
}