/*
🛠️ COMPILATION & SETUP INSTRUCTIONS
Board: "Generic STM32F1 series" → "Blue Pill F103C8"
Upload method: "STLink", CPU Speed: "72 MHz (Normal)"

6-AXIS SYSTEM WIRING (identical on both boards):
AXIS0: PA0 (STEP), PA1 (DIR), PB10 (LIMIT)
AXIS1: PA2 (STEP), PA3 (DIR), PA8  (LIMIT)
AXIS2: PB3 (STEP), PB4 (DIR), PB5  (LIMIT)

SPI interconnect (Master ↔ Slave):
PA5 (SCK), PA6 (MISO), PA7 (MOSI), PA4 (NSS)
PB0 (SYNC), PB1 (ALARM), GND (common ground required!)

MASTER board: connect USB to PC/SimHub
SLAVE board:  powered separately, no USB needed

⚠️ IMPORTANT: 
To compile for Master, ensure dma_stepper_hal.h has: #define CONTROLLER_MODE CONTROLLER_MODE_MASTER
To compile for Slave, ensure dma_stepper_hal.h has:  #define CONTROLLER_MODE CONTROLLER_MODE_SLAVE
Do NOT define CONTROLLER_MODE in this .ino file.
*/

// 2026-08-19: Merged 6-axis SPI Master/Slave with advanced limit switch config.
// 3DOF by Andrey Zhuravlev
// v.azhure@gmail.com
// Discord: https://discord.gg/ynHCkrsmMA

#include "dma_stepper_hal.h"
#include <EEPROM.h>
#include <string.h>

#define SERIAL_BAUD_RATE 115200
#define PID_KP_SCALE 10.0f
#define PID_KI_SCALE 10.0f
#define PID_KD_SCALE 100.0f
#define PID_KS_SCALE 100.0f
#define SH_DATA_MAX_VALUE 65535

#define AXIS0_STEP PA0
#define AXIS0_DIR PA1
#define AXIS0_LIMIT PB10
#define AXIS1_STEP PA2
#define AXIS1_DIR PA3
#define AXIS1_LIMIT PA8
#define AXIS2_STEP PB3
#define AXIS2_DIR PB4
#define AXIS2_LIMIT PB5
#define LED_PIN PB2

#define CMD_ID 0
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

// 2026-08-19 FIX: Double underscore for packed attribute
struct __attribute__((packed)) PCCMD {
  uint8_t header;
  uint8_t len;
  COMMAND cmd;
  uint8_t reserved;
  int32_t data[AXES_TOTAL];
};

struct __attribute__((packed)) PCCMD_SH {
  uint8_t header = 0;
  uint8_t len;
  COMMAND cmd;
  uint8_t reserved;
  uint16_t data[AXES_TOTAL];
  uint16_t data2[AXES_TOTAL];
};

struct __attribute__((packed)) STATE {
  uint8_t mode;
  uint8_t flags;
  uint16_t speedMMperSEC;
  int32_t currentpos;
  int32_t targetpos;
  int32_t min;
  int32_t max;
};

struct __attribute__((packed)) PID_STATE {
  uint8_t version;
  uint8_t flags;
  uint16_t blend;
  float Kp, Ki, Kd, Ks;
};

enum PID_FLAGS : uint8_t {
  PID_NONE = 0,
  PID_ENABLED = 1,
  PID_MASTER_SYNC = 1 << 1,
  PID_DIAG_ENABLED = 1 << 2
};

PCCMD pccmd;
PCCMD_SH& pccmd_sh = (PCCMD_SH&)pccmd;
const int RAW_DATA_LEN = sizeof(PCCMD);

volatile bool dataReceived = false;
uint8_t rxBuffer[RAW_DATA_LEN * 2];
int rxOffset = 0;

PID_STATE pidGlobal = {
  .version = 1, .flags = PID_FLAGS::PID_ENABLED, .blend = 100, .Kp = 15.0f, .Ki = 0.0f, .Kd = 0.02f, .Ks = 0.30f
};
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
  for (int logicalAxis = 0; logicalAxis < AXES_TOTAL; logicalAxis++) {
    STATE s;
    memset(&s, 0, sizeof(s));  // 2026-08-19 FIX: Zero-init to avoid garbage

#if (CONTROLLER_MODE == CONTROLLER_MODE_MASTER)
    if (logicalAxis < AXES_PER_BOARD) {
      AxisState* ax = DMAStepper_GetAxis(logicalAxis);
      if (!ax) continue;
      s.mode = ax->mode;
      s.flags = ax->homed ? 0x02 : 0;
      s.speedMMperSEC = (uint16_t)ax->maxSpeedMM;
      s.currentpos = ax->currentPosition;
      s.targetpos = ax->targetPosition;
      s.min = ax->minPos;
      s.max = ax->maxPos;
    } else {
      int slaveIdx = logicalAxis - AXES_PER_BOARD;
      SPI_FRAME* frame = SPI_Controller_GetLastFrame();

      // 2026-08-19 FIX: Never use 'continue'. Send explicit "no data" state.
      if (!frame) {
        s.mode = 0;      // MODE_UNKNOWN
        s.flags = 0x08;  // Special flag for C#: "no data from slave"
        s.speedMMperSEC = 0;
        s.currentpos = 0;
        s.targetpos = 0;
        s.min = 0;
        s.max = 0;
      } else {
        s.mode = frame->modes[slaveIdx];
        s.flags = (frame->slaveStatus & (1 << slaveIdx)) ? 0x02 : 0;
        if (frame->slaveStatus & 0x08) s.flags |= 0x04;
        s.speedMMperSEC = 0;
        s.currentpos = frame->positions[slaveIdx];
        s.targetpos = frame->targets[slaveIdx];
        s.min = 0;
        s.max = 0;
      }
    }
#else
    if (logicalAxis < LOGICAL_AXIS_OFFSET || logicalAxis >= LOGICAL_AXIS_OFFSET + AXES_PER_BOARD) continue;
    int localIdx = logicalAxis - LOGICAL_AXIS_OFFSET;
    AxisState* ax = DMAStepper_GetAxis(localIdx);
    if (!ax) continue;
    s.mode = ax->mode;
    s.flags = ax->homed ? 0x02 : 0;
    s.speedMMperSEC = (uint16_t)ax->maxSpeedMM;
    s.currentpos = ax->currentPosition;
    s.targetpos = ax->targetPosition;
    s.min = ax->minPos;
    s.max = ax->maxPos;
#endif

    Serial.write(10 + logicalAxis);
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
      {
#if (CONTROLLER_MODE == CONTROLLER_MODE_MASTER)
        for (int i = 0; i < AXES_PER_BOARD; i++) {
          AxisState* ax = DMAStepper_GetAxis(i);
          if (!ax || !ax->homed) continue;
          if (ax->mode == MODE_READY) DMAStepper_SetTarget(i, pccmd.data[i]);
          else if (ax->mode == MODE_PARKED || ax->mode == MODE_UNPARKING) {
            ax->pendingTarget = pccmd.data[i];
            if (ax->mode == MODE_PARKED) ax->mode = MODE_UNPARKING;
          }
        }
        int32_t slaveData[3] = { pccmd.data[3], pccmd.data[4], pccmd.data[5] };
        SPI_Controller_SendCommand(CMD_MOVE, slaveData, false);
#endif
        break;
      }
    case CMD_MOVE_SH:
      {
#if (CONTROLLER_MODE == CONTROLLER_MODE_MASTER)
        for (int i = 0; i < AXES_PER_BOARD; i++) {
          uint16_t val = pccmd_sh.data[i];
          val = (val >> 8) | (val << 8);
          AxisState* ax = DMAStepper_GetAxis(i);
          if (!ax || !ax->homed) continue;
          int32_t target = map(val, 0, SH_DATA_MAX_VALUE, ax->minPos, ax->maxPos);
          if (ax->mode == MODE_READY) DMAStepper_SetTarget(i, target);
          else if (ax->mode == MODE_PARKED || ax->mode == MODE_UNPARKING) {
            ax->pendingTarget = target;
            if (ax->mode == MODE_PARKED) ax->mode = MODE_UNPARKING;
          }
        }
        int32_t slaveData[3];
        for (int i = 0; i < 3; i++) {
          uint16_t val = pccmd_sh.data[AXES_PER_BOARD + i];
          val = (val >> 8) | (val << 8);
          AxisState* refAx = DMAStepper_GetAxis(0);
          if (refAx) slaveData[i] = map(val, 0, SH_DATA_MAX_VALUE, refAx->minPos, refAx->maxPos);
        }
        SPI_Controller_SendCommand(CMD_MOVE, slaveData, false);
#endif
        break;
      }
    case CMD_SET_SPEED:
      {
#if (CONTROLLER_MODE == CONTROLLER_MODE_MASTER)
        for (int i = 0; i < AXES_PER_BOARD; i++) DMAStepper_SetMaxSpeed(i, (uint32_t)pccmd.data[0]);
        int32_t spdData[3] = { pccmd.data[0], 0, 0 };
        SPI_Controller_SendCommand(CMD_SET_SPEED, spdData, false);
#endif
        break;
      }
    case CMD_SET_ACCEL:
      {
#if (CONTROLLER_MODE == CONTROLLER_MODE_MASTER)
        for (int i = 0; i < AXES_PER_BOARD; i++) {
          AxisState* ax = DMAStepper_GetAxis(i);
          if (ax) ax->maxAccel = pccmd.data[0];
        }
        int32_t accData[3] = { pccmd.data[0], 0, 0 };
        SPI_Controller_SendCommand(CMD_SET_ACCEL, accData, false);
#endif
        break;
      }
    case CMD_ENABLE:
      {
#if (CONTROLLER_MODE == CONTROLLER_MODE_MASTER)
        for (int i = 0; i < AXES_PER_BOARD; i++) {
          AxisState* ax = DMAStepper_GetAxis(i);
          if (ax) ax->mode = ax->homed ? MODE_READY : MODE_CONNECTED;
        }
        SPI_Controller_SendCommand(CMD_ENABLE, nullptr, false);
#endif
        break;
      }
    case CMD_DISABLE:
      {
#if (CONTROLLER_MODE == CONTROLLER_MODE_MASTER)
        DMAStepper_StopAll();
        for (int i = 0; i < AXES_PER_BOARD; i++) {
          AxisState* ax = DMAStepper_GetAxis(i);
          if (ax) ax->mode = MODE_DISABLED;
        }
        SPI_Controller_SendCommand(CMD_DISABLE, nullptr, false);
#endif
        break;
      }
    case CMD_CLEAR_ALARM:
      {
#if (CONTROLLER_MODE == CONTROLLER_MODE_MASTER)
        DMAStepper_ClearAlarm();
        SPI_Controller_SendCommand(CMD_CLEAR_ALARM, nullptr, false);
#endif
        break;
      }
    case CMD_HOME:
      {
#if (CONTROLLER_MODE == CONTROLLER_MODE_MASTER)
        for (int i = 0; i < AXES_PER_BOARD; i++) {
          AxisState* ax = DMAStepper_GetAxis(i);
          if (!ax) continue;
          if (ax->homed && (ax->mode == MODE_READY || ax->mode == MODE_PARKED)) continue;
          DMAStepper_StartHoming(i);
        }
        SPI_Controller_SendCommand(CMD_HOME, nullptr, false);
#endif
        break;
      }
    case CMD_PARK:
      {
#if (CONTROLLER_MODE == CONTROLLER_MODE_MASTER)
        for (int i = 0; i < AXES_PER_BOARD; i++) {
          AxisState* ax = DMAStepper_GetAxis(i);
          if (ax && ax->homed) ax->mode = MODE_PARKING;
        }
        SPI_Controller_SendCommand(CMD_PARK, nullptr, false);
#endif
        break;
      }
    case SET_ALARM:
      {
#if (CONTROLLER_MODE == CONTROLLER_MODE_MASTER)
        DMAStepper_StopAll();
        for (int i = 0; i < AXES_PER_BOARD; i++) {
          AxisState* ax = DMAStepper_GetAxis(i);
          if (ax) {
            ax->mode = MODE_ALARM;
            ax->homed = false;
          }
        }
#endif
        break;
      }
    case CMD_GET_STATE:
      {
        sendAllStates();
        break;
      }
    case CMD_SET_PID_KP:
      {
        pidGlobal.Kp = constrain((float)pccmd.data[0] / PID_KP_SCALE, 0.0f, 200.0f);
        for (int i = 0; i < AXES_PER_BOARD; i++) DMAStepper_SetPID(i, pidGlobal.Kp, pidGlobal.Ki, pidGlobal.Kd, pidGlobal.Ks);
        break;
      }
    case CMD_SET_PID_KI:
      {
        pidGlobal.Ki = constrain((float)pccmd.data[0] / PID_KI_SCALE, 0.0f, 50.0f);
        for (int i = 0; i < AXES_PER_BOARD; i++) DMAStepper_SetPID(i, pidGlobal.Kp, pidGlobal.Ki, pidGlobal.Kd, pidGlobal.Ks);
        break;
      }
    case CMD_SET_PID_KD:
      {
        pidGlobal.Kd = constrain((float)pccmd.data[0] / PID_KD_SCALE, 0.0f, 50.0f);
        for (int i = 0; i < AXES_PER_BOARD; i++) DMAStepper_SetPID(i, pidGlobal.Kp, pidGlobal.Ki, pidGlobal.Kd, pidGlobal.Ks);
        break;
      }
    case CMD_SET_PID_KS:
      {
        pidGlobal.Ks = constrain((float)pccmd.data[0] / PID_KS_SCALE, 0.0f, 1.0f);
        for (int i = 0; i < AXES_PER_BOARD; i++) DMAStepper_SetPID(i, pidGlobal.Kp, pidGlobal.Ki, pidGlobal.Kd, pidGlobal.Ks);
        break;
      }
    case CMD_SET_PID_ENABLE:
      {
        for (int i = 0; i < AXES_PER_BOARD; i++) DMAStepper_SetPIDEnable(i, pccmd.data[0] != 0);
        if (pccmd.data[0]) pidGlobal.flags |= PID_FLAGS::PID_ENABLED;
        else pidGlobal.flags &= ~PID_FLAGS::PID_ENABLED;
        break;
      }
    case CMD_SET_PID_BLEND:
      {
        pidGlobal.blend = constrain((uint16_t)pccmd.data[0], 0, 100);
        for (int i = 0; i < AXES_PER_BOARD; i++) DMAStepper_SetPIDBlend(i, (float)pidGlobal.blend / 100.0f);
        break;
      }
    case CMD_GET_PID_STATE:
      {
        sendPIDState();
        break;
      }
    case CMD_STORE_PID:
      {
        savePidToEEPROM();
        break;
      }
    case CMD_RESTORE_PID:
      {
        if (loadPidFromEEPROM()) {
          bool pidWasEnabled = (pidGlobal.flags & PID_FLAGS::PID_ENABLED) != 0;
          for (int i = 0; i < AXES_PER_BOARD; i++) {
            DMAStepper_SetPIDEnable(i, false);
            DMAStepper_SetPID(i, pidGlobal.Kp, pidGlobal.Ki, pidGlobal.Kd, pidGlobal.Ks);
            DMAStepper_SetPIDBlend(i, (float)pidGlobal.blend / 100.0f);
            if (pidWasEnabled) DMAStepper_SetPIDEnable(i, true);
          }
        }
        break;
      }
    default:
      {
        break;
      }
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

  for (int i = 0; i < AXES_PER_BOARD; i++) {
    DMAStepper_SetPID(i, pidGlobal.Kp, pidGlobal.Ki, pidGlobal.Kd, pidGlobal.Ks);
    DMAStepper_SetPIDBlend(i, (float)pidGlobal.blend / 100.0f);
    bool pidEnabled = (pidGlobal.flags & PID_FLAGS::PID_ENABLED) != 0;
    DMAStepper_SetPIDEnable(i, pidEnabled);
  }

  SPI_Controller_Init();

#if (CONTROLLER_MODE == CONTROLLER_MODE_MASTER)
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial)
    ;
#else
  Serial.begin(SERIAL_BAUD_RATE);
#endif
  digitalWrite(LED_PIN, HIGH);
}

inline void serialEvent() {
#if (CONTROLLER_MODE == CONTROLLER_MODE_MASTER)
  int data_cnt = min(Serial.available(), RAW_DATA_LEN);
  if (data_cnt < 2) return;

  for (int t = 0; t < data_cnt; ++t) {
    int byte = Serial.read();
    if (rxOffset > 0) {
      rxBuffer[rxOffset++] = byte;
      if (rxOffset == RAW_DATA_LEN) {
        memcpy(&pccmd, rxBuffer, RAW_DATA_LEN);
        dataReceived = true;
        rxOffset = 0;
      }
    } else {
      if (byte == CMD_ID) {
        // 2026-08-19 FIX: Compare with RAW_DATA_LEN constant, not uninitialized pccmd.len
        int nextByte = Serial.peek();
        if (nextByte == RAW_DATA_LEN) {
          rxBuffer[rxOffset++] = CMD_ID;
        }
      }
    }
  }
#endif
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

#if (CONTROLLER_MODE == CONTROLLER_MODE_SLAVE)
  SPI_Controller_Process();
#endif
}