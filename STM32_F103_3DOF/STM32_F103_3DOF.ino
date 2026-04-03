#include "dma_stepper_hal.h"
#include <EEPROM.h>
#include <string.h>

#define SERIAL_BAUD_RATE 115200

// =============================================================================
// ПИНЫ (AXIS3_LIMIT перенесён на PB12 для избежания конфликта с USB PA11)
// =============================================================================
#define AXIS0_STEP PA0
#define AXIS0_DIR PA1
#define AXIS0_LIMIT PA8
#define AXIS1_STEP PA2
#define AXIS1_DIR PA3
#define AXIS1_LIMIT PA9
#define AXIS2_STEP PA4
#define AXIS2_DIR PA5
#define AXIS2_LIMIT PA10
#define AXIS3_STEP PA6
#define AXIS3_DIR PA7
#define AXIS3_LIMIT PB12
#define LED_PIN PC13

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
  int32_t data[4];
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

PCCMD pccmd;
volatile bool dataReceived = false;
uint8_t rxBuffer[32];
int rxOffset = 0;
PID_STATE pidGlobal = { .version = 1, .flags = 0, .blend = 35, .Kp = 15.0f, .Ki = 0.0f, .Kd = 0.02f, .Ks = 0.30f };
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
      // Запуск калибровки
      for (int i = 0; i < NUM_AXES; i++) {
        AxisState* ax = DMAStepper_GetAxis(i);
        if (!ax) continue;

        if (ax->homed && ax->mode != MODE_ALARM) {
          continue;
        }

        ax->mode = MODE_HOMING;
        ax->homed = false;  // Сбрасываем флаг калибровки

        // Подготовка к движению
        DMAStepper_StopAxis(i);
        DMAStepper_SetFrequency(i, 1000);  // Медленная скорость для поиска (10 мм/с)
        DMAStepper_SetPosition(i, 0);      // Сброс счетчика для корректного отсчета
        // Цель за пределами максимума, чтобы гарантированно упереться в концевик
        DMAStepper_SetTarget(i, (int32_t)(MAX_REVOLUTIONS * STEPS_PER_REV * 1.5));

        DMAStepper_StartAxis(i, true);  // Старт в сторону концевика
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
      if (pccmd.data[0]) pidGlobal.flags |= 0x01;
      else pidGlobal.flags &= ~0x01;
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
        bool pidWasEnabled = (pidGlobal.flags & 0x01) != 0;

        // Останавливаем PID на всех осях, применяем новые коэффициенты
        for (int i = 0; i < NUM_AXES; i++) {
          DMAStepper_SetPIDEnable(i, false);  // Остановка PID
          DMAStepper_SetPID(i, pidGlobal.Kp, pidGlobal.Ki, pidGlobal.Kd, pidGlobal.Ks);
          DMAStepper_SetPIDBlend(i, (float)pidGlobal.blend / 100.0f);

          // Если PID был включён глобально, восстанавливаем работу (сброс интеграла произойдёт внутри HAL)
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
  loadPidFromEEPROM();
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial)
    ;

  DMAStepper_Init();
  DMAStepper_InitAxis(0, AXIS0_STEP, AXIS0_DIR, AXIS0_LIMIT);
  DMAStepper_InitAxis(1, AXIS1_STEP, AXIS1_DIR, AXIS1_LIMIT);
  DMAStepper_InitAxis(2, AXIS2_STEP, AXIS2_DIR, AXIS2_LIMIT);
  DMAStepper_InitAxis(3, AXIS3_STEP, AXIS3_DIR, AXIS3_LIMIT);

  for (int i = 0; i < NUM_AXES; i++) {
    DMAStepper_SetPID(i, pidGlobal.Kp, pidGlobal.Ki, pidGlobal.Kd, pidGlobal.Ks);
    DMAStepper_SetPIDBlend(i, (float)pidGlobal.blend / 100.0f);

    bool pidEnabled = (pidGlobal.flags & 0x01) != 0;
    DMAStepper_SetPIDEnable(i, pidEnabled);
  }

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
}

void loop() {
  while (Serial.available()) {
    uint8_t b = Serial.read();
    if (rxOffset == 0 && b != CMD_ID) continue;
    rxBuffer[rxOffset++] = b;
    if (rxOffset >= PCCMD_SIZE) {
      memcpy(&pccmd, rxBuffer, PCCMD_SIZE);
      dataReceived = true;
      rxOffset = 0;
    }
  }
  if (dataReceived) {
    dataReceived = false;
    digitalWrite(LED_PIN, LOW);
    processCommand();
    digitalWrite(LED_PIN, HIGH);
  }

  DMAStepper_Process();
}