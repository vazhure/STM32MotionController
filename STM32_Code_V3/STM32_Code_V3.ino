// 3DOF by Andrey Zhuravlev
// Timer-based-stepping and PID controls added by Laith Wattar (https://github.com/lawattar)

// V3 Changes:
// - Motion Smoothing/Buffer removed
// - Toggleable PID motion shaping added:
//   - STM32 PID is NOT true closed-loop; it uses estimated step-count position (no encoder connection)
//   - Servo drives run their own PID loops internally with encoder feedback (true closed-loop)
//   - STM32 shapes step/dir commands before servo; servo corrects any position errors
//   - Ki MUST stay at 0 (servo speed loop already has integral - avoid double integration. Adjust it at your own risk.)
//   - V3 Code requires usage of an updated shmotioncontroller file. This will unlock custom SimHub PID sliders in the Protocol Control Panel (click on output settings).
//   - These sliders can change the PID parameters via I2C, but motion must first be disabled in SimHub. Once edited, re-enable motion and test.

// e-mail: v.azhure@gmail.com
// original version from 2025-06-21
// modified version from 2026-03-01
// modified version from 2026-03-10 : struct STATE speedMMperSEC uint8_t -> uint16_t. Overal length of struct not changed because of alignment
// modified version from 2026-03-19 : added enum PID_FLAGS, struct PID_STATE, updated command set enum COMMAND
// stm32duino version

//!!!!!!!!!!!!!!!!!!!!!!!!!!!! WARNING !!!!!!!!!!!!!!!!!!!!!!!!!!!!
// NOTE: MAKE SURE TO EDIT LINES 813-827 IF YOU CHANGE THE BALLSCREW LENGTH!
// Also please double check:
// -LED_PIN
// -MM_PER_REV - distance in mm per revolution
// -MAX_REVOLUTIONS - total travel distance devided by MM_PER_REV
// -STEPS_PER_REVOLUTIONS - Steps per revolution, decreased from 2000 to 1000 to accomadate slower STM32s (verify ratio on your specific servo controller)
//!!!!!!!!!!!!!!!!!!!!!!!!!!!! WARNING !!!!!!!!!!!!!!!!!!!!!!!!!!!!

// NOTE: select fake STM32F103C8 or Generik STM32F103C series, CPU Speed 74Mhz, upload method STLink, Optimize Fast (-O1)
// https://www.stm32duino.com/
//
// append to Board Manager URLs:
// http://dan.drown.org/stm32duino/package_STM32duino_index.json
// https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json
//
// Download and isntall ST-Link Utility https://www.st.com/en/development-tools/stsw-link004.html

//#define DEBUG
// Board: STM32F103C8T6 5 pcs (master + 4 slave)

#define I2C_SDA PB7
#define I2C_SCL PB6
#include <Wire_slave.h>
#include <stdio.h>
#include <bitset>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>

// Uncomment this line if you using SF1610
#define SFU1610  // Used only in SLAVE devices

// Uncomment this line to flash MASTER device
// Comment this line to flash SLAVE devices
//#define I2CMASTER

// Maximal number of linear actuators
#define MAX_LINEAR_ACTUATORS 4
// Number of linear actuators
#define LINEAR_ACTUATORS MAX_LINEAR_ACTUATORS

#define SLAVE_ADDR_FL 10  // Front / Front Left
#define SLAVE_ADDR_RL 11  // Rear Left
#define SLAVE_ADDR_RR 12  // Rear Right
#define SLAVE_ADDR_FR 13  // Front RIGHT

#define SLAVE_FIRST SLAVE_ADDR_FL
#define SLAVE_LAST (SLAVE_FIRST + LINEAR_ACTUATORS - 1)

void allPinsPulldown() {
  for (int pin = PA0; pin <= PC15; pin++) pinMode(pin, INPUT_PULLDOWN);
}

inline int WireRead(uint8_t* ptr, uint8_t len) {
  int cnt = Wire.available();
  len = len > cnt ? cnt : len;
  for (int t = 0; t < len; t++) {
    ptr[t] = Wire.read();
  }
  return len;
}

template<class T>
const T& clamp(const T& x, const T& a, const T& b) {
  if (x < a) {
    return a;
  } else if (b < x) {
    return b;
  } else
    return x;
}

#ifndef I2CMASTER
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Uncomment a single line with desired Address to flash SLAVE device
//#define SLAVE_ADDR SLAVE_ADDR_FL
//#define SLAVE_ADDR SLAVE_ADDR_RL
//#define SLAVE_ADDR SLAVE_ADDR_RR
#define SLAVE_ADDR SLAVE_ADDR_FR
#endif

#define SERIAL_BAUD_RATE 115200

// override Serial Port buffer size
#define SERIAL_TX_BUFFER_SIZE 512
#define SERIAL_RX_BUFFER_SIZE 512

#define LED_PIN PB2  // PC13  // * Check your board. Some have PB2 or another (label near LED).

enum MODE : uint8_t { UNKNOWN,
                      CONNECTED,
                      DISABLED,
                      HOMEING,
                      PARKING,
                      READY,
                      ALARM
};

enum COMMAND : uint8_t { CMD_HOME,
                         CMD_MOVE,
                         CMD_SET_SPEED,
                         CMD_DISABLE,
                         CMD_ENABLE,
                         CMD_GET_STATE,
                         CMD_CLEAR_ALARM,
                         CMD_PARK,
                         SET_ALARM,
                         CMD_SET_SLOW_SPEED,
                         CMD_SET_ACCEL,
                         CMD_MOVE_SH,
                         CMD_SET_PID_KP,      // 0x0C
                         CMD_SET_PID_KI,      // 0x0D
                         CMD_SET_PID_KD,      // 0x0E
                         CMD_SET_PID_KS,      // 0x0F
                         CMD_SET_PID_ENABLE,  // 0x10
                         CMD_SET_PID_BLEND,   // 0x11
                         CMD_GET_PID_STATE,   // 0x12
                         CMD_STORE_PID,       // 0x13
                         CMD_RESTORE_PID      // 0x14
};

enum FLAGS : uint8_t { NONE = 0,
                       STATE_ON_LIMIT_SWITCH = 1,
                       STATE_HOMED = 1 << 1,
};

enum PID_FLAGS : uint8_t {
  PID_NONE = 0,
  PID_ENABLED = 1,
  PID_MASTER_SYNC = 1 << 1,
  PID_DIAG_ENABLED = 1 << 2,  // Diagnostics: set false after validation if SimHub parsing is affected.
};

#define PID_STATE_ID 255

#define HAS_FLAG(x, flag) (((x) & (flag)) == (flag))
#define SET_FLAG(x, flag) ((x) = static_cast<decltype(x)>((x) | (flag)))
#define CLEAR_FLAG(x, flag) ((x) = static_cast<decltype(x)>((x) & ~(flag)))
#define TOGGLE_FLAG(x, flag) ((x) = static_cast<decltype(x)>((x) ^ (flag)))

struct STATE {
  MODE mode;
  FLAGS flags;
  uint16_t speedMMperSEC;
  int32_t currentpos;
  int32_t targetpos;
  int32_t min;
  int32_t max;
};

const int STATE_LEN = sizeof(STATE);

struct PCCMD {
  uint8_t header = 0;
  uint8_t len;  // len
  COMMAND cmd;
  uint8_t reserved;
  int32_t data[MAX_LINEAR_ACTUATORS];
} pccmd;

// FOR SimHub
struct PCCMD_SH {
  uint8_t header = 0;
  uint8_t len;  // len
  COMMAND cmd;
  uint8_t reserved;
  uint16_t data[MAX_LINEAR_ACTUATORS];
  uint16_t data2[MAX_LINEAR_ACTUATORS];  // EMPTY
};

PCCMD_SH& pccmd_sh = *(PCCMD_SH*)&pccmd;

const int RAW_DATA_LEN = sizeof(PCCMD);

#ifdef I2CMASTER  // MASTER CODE below

#include <EEPROM.h>

#define ALARM_PIN PA4
#define SIGNAL_PIN PA5

struct PID_STATE {
  uint8_t version = 1;
  PID_FLAGS flags = PID_FLAGS::PID_NONE;
  uint16_t masterPidBlend = 100;  // [0..100]
  float masterPidKp = 15.0f;
  float masterPidKi = 0.0f;
  float masterPidKd = 0.02f;
  float masterPidKs = 0.30f;
} pid_state;

const int PID_STATE_LEN = sizeof(PID_STATE);

#define PID_EEPROM_MAGIC 0x5A  // Magic number for EEPROM
#define PID_EEPROM_ADDR 0      // EEPROM addr for PID

void savePidToEEPROM() {
  EEPROM.update(PID_EEPROM_ADDR, PID_EEPROM_MAGIC);

  uint8_t* ptr = (uint8_t*)&pid_state;
  for (int i = 0; i < PID_STATE_LEN; i++) {
    EEPROM.update(PID_EEPROM_ADDR + 1 + i, ptr[i]);
  }
}

bool loadPidFromEEPROM() {
  if (EEPROM.read(PID_EEPROM_ADDR) != PID_EEPROM_MAGIC) {
    return false;
  }

  uint8_t* ptr = (uint8_t*)&pid_state;
  for (int i = 0; i < PID_STATE_LEN; i++) {
    ptr[i] = EEPROM.read(PID_EEPROM_ADDR + 1 + i);
  }
  return true;
}

void QueuePidTextCommand(COMMAND cmd, uint32_t data);
void UpdateMasterPidMirror(COMMAND cmd, uint32_t data);
void QueueSynchronizedTargets(const int32_t* targets);
void DispatchSynchronizedTargetsIfDue(bool force);
bool TransmitCMD(uint8_t addr, uint8_t cmd, uint32_t data);

// Device states
STATE st[MAX_LINEAR_ACTUATORS];
bool arr_slaves[MAX_LINEAR_ACTUATORS];

#define SET_SLAVE_STATE(idx, x) arr_slaves[idx] = x
#define HAS_SLAVE(idx) arr_slaves[idx]

volatile bool bAlarm = false;
volatile bool estopLatched = false;
uint32_t lastAlarmBroadcastMs = 0;

// Alarm input event
void OnAlarm() {
  bAlarm = true;
  estopLatched = true;
}

void ActualizePID() {
  for (int t = 0; t < LINEAR_ACTUATORS; t++) {
    TransmitCMD(SLAVE_FIRST + t, COMMAND::CMD_SET_PID_ENABLE, 0);
    TransmitCMD(SLAVE_FIRST + t, COMMAND::CMD_SET_PID_KI, pid_state.masterPidKi * 10.0f);
    TransmitCMD(SLAVE_FIRST + t, COMMAND::CMD_SET_PID_KP, pid_state.masterPidKp * 10.0f);
    TransmitCMD(SLAVE_FIRST + t, COMMAND::CMD_SET_PID_KD, pid_state.masterPidKd * 100.0f);
    TransmitCMD(SLAVE_FIRST + t, COMMAND::CMD_SET_PID_KS, pid_state.masterPidKs * 100.0f);
    TransmitCMD(SLAVE_FIRST + t, COMMAND::CMD_SET_PID_BLEND, pid_state.masterPidBlend);
    TransmitCMD(SLAVE_FIRST + t, COMMAND::CMD_SET_PID_ENABLE, HAS_FLAG(pid_state.flags, PID_FLAGS::PID_ENABLED) ? 1 : 0);
  }
}

#define RETRY_CNT 3

void setup() {
  loadPidFromEEPROM();
  allPinsPulldown();

  pinMode(LED_PIN, OUTPUT);
  pinMode(ALARM_PIN, INPUT_PULLDOWN);
  pinMode(SIGNAL_PIN, INPUT_PULLDOWN);
  digitalWrite(LED_PIN, HIGH);

  attachInterrupt(ALARM_PIN, OnAlarm, RISING);
  estopLatched = (digitalRead(ALARM_PIN) == HIGH);
  if (estopLatched) {
    bAlarm = true;
  }
  Wire.begin();
  // Wire.setClock(400000);  // Reverted due to connectivity issues: use default I2C clock
  delay(1000);
  int nFound = 0;
  for (int t = 0; t < RETRY_CNT; t++) {
    nFound = 0;
    // scanning slave devices
    for (int addr = SLAVE_FIRST; addr <= SLAVE_LAST; addr++) {
      Wire.beginTransmission(addr);
      uint8_t err = Wire.endTransmission();
      SET_SLAVE_STATE(addr - SLAVE_FIRST, err == 0);
      if (err == 0) nFound++;
    }
    if (nFound == MAX_LINEAR_ACTUATORS)
      break;
    delay(100);
  }

  digitalWrite(LED_PIN, nFound == 0 ? HIGH : LOW);

  ActualizePID();

  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial)
    ;  // whait connected

#ifdef DEBUG
  // I2C Devices scan
  Serial.println("Seeking slave devices");
  for (int addr = SLAVE_FIRST; addr <= SLAVE_LAST; addr++)
    for (int t = 0; t < RETRY_CNT; t++) {
      Wire.beginTransmission(addr);
      uint8_t err = Wire.endTransmission();
      if (err == 0) {
        Serial.print("Found Device: ");
        Serial.println(addr);
        break;
      } else if (err == 4) {
        Serial.print("Unknown error at address ");
        Serial.println(addr);
      }
      delay(500);
    }
  Serial.println("Seeking finished");
#endif
}

#define CMD_ID 0
// serial input buffer
uint8_t buf[RAW_DATA_LEN * 2];
int offset = 0;
volatile bool _bDataPresent = false;
unsigned long _lasttime;
char txtbuf[48];
uint8_t txtoffset = 0;
bool textCmdPending = false;
COMMAND textPendingCmd = COMMAND::CMD_SET_PID_KP;
uint32_t textPendingData = 0;
bool syncTrajectoryEnabled = false;
const uint16_t syncTrajectoryPeriodMs = 10;
int32_t syncTargets[MAX_LINEAR_ACTUATORS] = { 0 };
bool syncTargetsPending = false;
uint32_t syncLastDispatchMs = 0;
//
//bool pidDiagEnabled = false;
const uint32_t PID_DIAG_INTERVAL_MS = 5000;
uint32_t pidDiagLastMs = 0;

bool EqualsIgnoreCase(const char* a, const char* b) {
  while (*a && *b) {
    if (toupper((unsigned char)*a) != toupper((unsigned char)*b)) {
      return false;
    }
    ++a;
    ++b;
  }
  return *a == '\0' && *b == '\0';
}

void QueuePidTextCommand(COMMAND cmd, uint32_t data) {
  textPendingCmd = cmd;
  textPendingData = data;
  textCmdPending = true;
}

void UpdateMasterPidMirror(COMMAND cmd, uint32_t data) {
  switch (cmd) {
    case COMMAND::CMD_SET_PID_KP:
      pid_state.masterPidKp = clamp((float)data / 10.0f, 0.0f, 200.0f);
      break;
    case COMMAND::CMD_SET_PID_KI:
      pid_state.masterPidKi = clamp((float)data / 10.0f, 0.0f, 50.0f);
      break;
    case COMMAND::CMD_SET_PID_KD:
      pid_state.masterPidKd = clamp((float)data / 100.0f, 0.0f, 50.0f);
      break;
    case COMMAND::CMD_SET_PID_KS:
      pid_state.masterPidKs = clamp((float)data / 100.0f, 0.0f, 1.0f);
      break;
    case COMMAND::CMD_SET_PID_ENABLE:
      if (data != 0)
        SET_FLAG(pid_state.flags, PID_FLAGS::PID_ENABLED);
      else
        CLEAR_FLAG(pid_state.flags, PID_FLAGS::PID_ENABLED);
      break;
    case COMMAND::CMD_SET_PID_BLEND:
      pid_state.masterPidBlend = (uint16_t)data;
      break;
    default:
      break;
  }
}

void PrintPidDiagnosticsIfDue() {
  if (!HAS_FLAG(pid_state.flags, PID_DIAG_ENABLED)) {
    return;
  }
  uint32_t now = millis();
  if ((now - pidDiagLastMs) < PID_DIAG_INTERVAL_MS) {
    return;
  }
  pidDiagLastMs = now;

  Serial.print("PID EN=");
  Serial.print(HAS_FLAG(pid_state.flags, PID_FLAGS::PID_ENABLED) ? 1 : 0);
  Serial.print(" BLEND=");
  Serial.print(pid_state.masterPidBlend / 100.0f, 2);
  Serial.print(" KP=");
  Serial.print(pid_state.masterPidKp, 2);
  Serial.print(" KI=");
  Serial.print(pid_state.masterPidKi, 2);
  Serial.print(" KD=");
  Serial.print(pid_state.masterPidKd, 3);
  Serial.print(" KS=");
  Serial.print(pid_state.masterPidKs, 2);
  Serial.print(" SYNC=");
  Serial.println(HAS_FLAG(pid_state.flags, PID_FLAGS::PID_MASTER_SYNC) ? 1 : 0);
}

void QueueSynchronizedTargets(const int32_t* targets) {
  memcpy(syncTargets, targets, sizeof(syncTargets));
  syncTargetsPending = true;
}

void DispatchSynchronizedTargetsIfDue(bool force) {
  if (!syncTrajectoryEnabled || !syncTargetsPending || estopLatched) {
    return;
  }
  uint32_t now = millis();
  if (!force && syncLastDispatchMs != 0 && (now - syncLastDispatchMs) < syncTrajectoryPeriodMs) {
    return;
  }

  for (int t = 0; t < LINEAR_ACTUATORS; t++) {
    TransmitCMD(SLAVE_FIRST + t, COMMAND::CMD_MOVE, syncTargets[t]);
  }
  syncTargetsPending = false;
  syncLastDispatchMs = now;
}

void ParseTextCommandLine() {
  static int32_t lastKp10 = -1;
  static int32_t lastKi10 = -1;
  static int32_t lastKd100 = -1;
  static int32_t lastKs100 = -1;
  static int32_t lastPidEnable = -1;
  static int32_t lastPidBlend = -1;
  static int32_t lastSyncExec = -1;

  txtbuf[txtoffset] = '\0';
  char* eq = strchr(txtbuf, '=');
  if (!eq) {
    return;
  }
  *eq = '\0';
  const char* key = txtbuf;
  const char* rawValue = eq + 1;
  float value = (float)atof(rawValue);

  if (EqualsIgnoreCase(key, "KP")) {
    value = clamp(value, 0.0f, 200.0f);
    int32_t v = (int32_t)(value * 10.0f + 0.5f);
    if (v != lastKp10) {
      lastKp10 = v;
      QueuePidTextCommand(COMMAND::CMD_SET_PID_KP, (uint32_t)v);
    }
  } else if (EqualsIgnoreCase(key, "KI")) {
    value = clamp(value, 0.0f, 50.0f);
    int32_t v = (int32_t)(value * 10.0f + 0.5f);
    if (v != lastKi10) {
      lastKi10 = v;
      QueuePidTextCommand(COMMAND::CMD_SET_PID_KI, (uint32_t)v);
    }
  } else if (EqualsIgnoreCase(key, "KD")) {
    // KD accepts scaled integer (0..100 means 0.00..1.00) for SimHub sliders.
    // If decimal text is provided, interpret it as direct KD and scale by 100.
    int32_t v = 0;
    if (strchr(rawValue, '.') != nullptr) {
      value = clamp(value, 0.0f, 50.0f);
      v = (int32_t)(value * 100.0f + 0.5f);
    } else {
      int32_t scaled = atoi(rawValue);
      v = clamp<int32_t>(scaled, 0, 100);
    }
    if (v != lastKd100) {
      lastKd100 = v;
      QueuePidTextCommand(COMMAND::CMD_SET_PID_KD, (uint32_t)v);
    }
  } else if (EqualsIgnoreCase(key, "KS")) {
    value = clamp(value, 0.0f, 100.0f);
    int32_t v = (int32_t)(value + 0.5f);
    if (v != lastKs100) {
      lastKs100 = v;
      QueuePidTextCommand(COMMAND::CMD_SET_PID_KS, (uint32_t)v);
    }
  } else if (EqualsIgnoreCase(key, "PIDEN")) {
    bool enabled = false;
    if (EqualsIgnoreCase(eq + 1, "TRUE") || EqualsIgnoreCase(eq + 1, "ON")) {
      enabled = true;
    } else {
      enabled = (value >= 0.5f);
    }
    int32_t v = enabled ? 1 : 0;
    if (v != lastPidEnable) {
      lastPidEnable = v;
      QueuePidTextCommand(COMMAND::CMD_SET_PID_ENABLE, (uint32_t)v);
    }
  } else if (EqualsIgnoreCase(key, "PIDBLEND")) {
    value = clamp(value, 0.0f, 100.0f);
    int32_t v = (int32_t)(value + 0.5f);
    if (v != lastPidBlend) {
      lastPidBlend = v;
      QueuePidTextCommand(COMMAND::CMD_SET_PID_BLEND, (uint32_t)v);
    }
  } else if (EqualsIgnoreCase(key, "PIDCFG")) {
    // Accept both numeric and bool text for PID enable:
    // PIDCFG=1,35 or PIDCFG=TRUE,35
    char* comma = strchr(eq + 1, ',');
    if (!comma) {
      return;
    }
    *comma = '\0';
    const char* enToken = eq + 1;
    const char* blendToken = comma + 1;

    int en = 0;
    if (EqualsIgnoreCase(enToken, "TRUE") || EqualsIgnoreCase(enToken, "ON")) {
      en = 1;
    } else if (EqualsIgnoreCase(enToken, "FALSE") || EqualsIgnoreCase(enToken, "OFF")) {
      en = 0;
    } else {
      en = (atof(enToken) >= 0.5f) ? 1 : 0;
    }

    int blend = clamp<int>((int)atoi(blendToken), 0, 100);

    if (en != lastPidEnable) {
      lastPidEnable = en;
      QueuePidTextCommand(COMMAND::CMD_SET_PID_ENABLE, (uint32_t)en);
      return;
    }
    if (blend != lastPidBlend) {
      lastPidBlend = blend;
      QueuePidTextCommand(COMMAND::CMD_SET_PID_BLEND, (uint32_t)blend);
    }
  } else if (EqualsIgnoreCase(key, "SYNCEXEC")) {
    bool enabled = false;
    if (EqualsIgnoreCase(eq + 1, "TRUE") || EqualsIgnoreCase(eq + 1, "ON")) {
      enabled = true;
    } else {
      enabled = (value >= 0.5f);
    }
    int32_t v = enabled ? 1 : 0;
    if (v != lastSyncExec) {
      lastSyncExec = v;
      syncTrajectoryEnabled = enabled;
      if (enabled)
        SET_FLAG(pid_state.flags, PID_FLAGS::PID_MASTER_SYNC);
      else
        CLEAR_FLAG(pid_state.flags, PID_FLAGS::PID_MASTER_SYNC);
      syncTargetsPending = false;
      syncLastDispatchMs = 0;
    }
  }
}

void serialEvent() {
  int data_cnt = std::min(Serial.available(), RAW_DATA_LEN);
  if (data_cnt < 1)
    return;
  for (int t = 0; t < data_cnt; ++t) {
    int byte = Serial.read();
    if (offset > 0) {
      buf[offset++] = byte;
      if (offset == RAW_DATA_LEN) {
        memcpy(&pccmd, buf, RAW_DATA_LEN);
        _bDataPresent = true;
        _lasttime = millis();
        offset = 0;
      }
    } else {
      if (byte == CMD_ID) {
        if (Serial.peek() == RAW_DATA_LEN) {
          buf[offset++] = CMD_ID;
        }
      } else if (byte == '\r' || byte == '\n') {
        if (txtoffset > 0) {
          ParseTextCommandLine();
          txtoffset = 0;
        }
      } else if (byte >= 32 && byte <= 126) {
        if (txtoffset < (sizeof(txtbuf) - 1)) {
          txtbuf[txtoffset++] = (char)byte;
        } else {
          txtoffset = 0;
        }
      }
    }
  }
}

bool RequestSlaveState(uint8_t addr, uint8_t* ptr, uint8_t len) {
  if (HAS_SLAVE(addr - SLAVE_FIRST) && Wire.requestFrom(addr, len) == len && WireRead(ptr, len) == len) {
    return true;
  }
  return false;
}

bool TransmitCMD(uint8_t addr, uint8_t cmd, uint32_t data) {
  if (HAS_SLAVE(addr - SLAVE_FIRST)) {
    Wire.beginTransmission(addr);
    Wire.write(cmd);
    Wire.write((uint8_t*)&data, sizeof(uint32_t));
    return Wire.endTransmission() == 0;
  }
  return false;
}

void loop() {
  bool estopActive = (digitalRead(ALARM_PIN) == HIGH);
  if (estopActive) {
    estopLatched = true;
    syncTargetsPending = false;
  }

  if (bAlarm || (estopActive && (millis() - lastAlarmBroadcastMs) > 100)) {
    syncTargetsPending = false;
    for (int addr = SLAVE_FIRST; addr <= SLAVE_LAST; addr++)
      TransmitCMD(addr, COMMAND::SET_ALARM, 1);
    bAlarm = false;
    lastAlarmBroadcastMs = millis();
  }

  if (Serial.available())
    serialEvent();

  if (syncTrajectoryEnabled) {
    DispatchSynchronizedTargetsIfDue(false);
  }

  PrintPidDiagnosticsIfDue();

  if (textCmdPending) {
    if (!estopLatched) {
      for (int t = 0; t < LINEAR_ACTUATORS; t++) {
        TransmitCMD(SLAVE_FIRST + t, textPendingCmd, textPendingData);
      }
      UpdateMasterPidMirror(textPendingCmd, textPendingData);
      textCmdPending = false;
    }
  }

  if (_bDataPresent) {
    digitalWrite(LED_PIN, LOW);  // Turn LED on
    switch (pccmd.cmd) {
      case COMMAND::CMD_HOME:
      case COMMAND::CMD_ENABLE:
      case COMMAND::SET_ALARM:
      case COMMAND::CMD_DISABLE:
        {
          // While E-stop is latched, block all motion/state-changing commands.
          if (estopLatched) {
            break;
          }
          for (int t = 0; t < LINEAR_ACTUATORS; t++) {
            if (pccmd.data[t] == 1)
              TransmitCMD(SLAVE_FIRST + t, pccmd.cmd, 0);
          }
        }
        break;
      case COMMAND::CMD_CLEAR_ALARM:
        {
          // Require E-stop input release before allowing reset/clear.
          if (estopActive) {
            break;
          }
          for (int t = 0; t < LINEAR_ACTUATORS; t++) {
            if (pccmd.data[t] == 1)
              TransmitCMD(SLAVE_FIRST + t, pccmd.cmd, 0);
          }
          estopLatched = false;
        }
        break;
      case COMMAND::CMD_MOVE_SH:
        {
          if (estopLatched) {
            break;
          }
          int32_t mappedTargets[LINEAR_ACTUATORS];
          for (int t = 0; t < LINEAR_ACTUATORS; t++) {
            uint16_t val = pccmd_sh.data[t];
            val = (val >> 8) | (val << 8);
            mappedTargets[t] = map(val, 0, 65535, st[t].min, st[t].max);
          }
          if (syncTrajectoryEnabled) {
            QueueSynchronizedTargets(mappedTargets);
          } else {
            for (int t = 0; t < LINEAR_ACTUATORS; t++) {
              TransmitCMD(SLAVE_FIRST + t, COMMAND::CMD_MOVE, (uint32_t)mappedTargets[t]);
            }
          }
        }
        break;
      case COMMAND::CMD_PARK:
      case COMMAND::CMD_SET_SPEED:
      case COMMAND::CMD_SET_SLOW_SPEED:
      case COMMAND::CMD_SET_ACCEL:
        {
          if (estopLatched) {
            break;
          }
          for (int t = 0; t < LINEAR_ACTUATORS; t++)
            TransmitCMD(SLAVE_FIRST + t, pccmd.cmd, pccmd.data[t]);
        }
        break;
      case COMMAND::CMD_MOVE:
        {
          if (estopLatched) {
            break;
          }
          if (syncTrajectoryEnabled) {
            QueueSynchronizedTargets(pccmd.data);
          } else {
            for (int t = 0; t < LINEAR_ACTUATORS; t++) {
              TransmitCMD(SLAVE_FIRST + t, COMMAND::CMD_MOVE, pccmd.data[t]);
            }
          }
        }
        break;
      case COMMAND::CMD_SET_PID_KP:
      case COMMAND::CMD_SET_PID_KI:
      case COMMAND::CMD_SET_PID_KD:
      case COMMAND::CMD_SET_PID_KS:
      case COMMAND::CMD_SET_PID_ENABLE:
      case COMMAND::CMD_SET_PID_BLEND:
        {
          if (estopLatched) {
            break;
          }
          UpdateMasterPidMirror(pccmd.cmd, (uint32_t)pccmd.data[0]);
          // PID tuning values are applied globally to all actuators.
          for (int t = 0; t < LINEAR_ACTUATORS; t++) {
            TransmitCMD(SLAVE_FIRST + t, pccmd.cmd, pccmd.data[0]);
          }
        }
        break;
      case COMMAND::CMD_GET_PID_STATE:
        {
          Serial.write(PID_STATE_ID);
          Serial.write(PID_STATE_LEN);
          Serial.write((uint8_t*)&pid_state, PID_STATE_LEN);
        }
        break;
      case COMMAND::CMD_GET_STATE:
        {
          for (int t = 0; t < LINEAR_ACTUATORS; t++) {
            if (!RequestSlaveState(SLAVE_FIRST + t, (uint8_t*)&st[t], STATE_LEN))
              st[t].mode = MODE::UNKNOWN;
          }
          for (int t = 0; t < LINEAR_ACTUATORS; t++) {
            Serial.write(SLAVE_FIRST + t);
            Serial.write(STATE_LEN);
            Serial.write((uint8_t*)&st[t], STATE_LEN);
          }
        }
        break;
      case COMMAND::CMD_RESTORE_PID:
        {
          if (loadPidFromEEPROM())
            ActualizePID();
        }
        break;
      case COMMAND::CMD_STORE_PID:
        {
          savePidToEEPROM();
        }
        break;
    }
    _bDataPresent = false;
    digitalWrite(LED_PIN, HIGH);  // Turn LED off
  }
}
#else  // SLAVE CODE below

volatile MODE mode;

const uint32_t dirPin = PA4;
const uint32_t stepPin = PA5;

const uint32_t STEP_PIN_BIT = stepPin - PA0;
const uint32_t DIR_PIN_BIT = dirPin - PA0;
const uint32_t limiterPinNO = PA6;
const uint8_t limiterPinNC = PA7;

#define ANALOG_INPUT_MAX 4095

volatile uint32_t accel = 25000;  // Acceleration limit in mm/s^2 (ie. 25000 = 2.5g, 12000 would be gentler --> adjust via desktop app)
#define STEPS_CONTROL_DIST STEPS_PER_REVOLUTIONS / 4  // Distance in steps

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//ADJUST VALUES BELOW BASED ON YOUR BALLSCREW SPECIFICATIONS

#ifdef SFU1610
// If you have SFU1610 ballscrew, adjust below:
const float MM_PER_REV = 10.0f;                                // distance in mm per revolution
const float MAX_REVOLUTIONS = 9.0;                             // maximum revolutions for ballscrew (adjust depending on custom length, ie. 14 = 140mm)
const int32_t STEPS_PER_REVOLUTIONS = 1000;                    // Steps per revolution, decreased from 2000 to 1000 to accomadate slower STM32s --> verify servo drive ratio is correct as well
const int32_t SAFE_DIST_IN_STEPS = STEPS_PER_REVOLUTIONS / 4;  // Safe traveling distance in steps (do not adjust)
#define MAX_SPEED_MM_SEC 400  // maximum speed mm/sec (keep at 400 and recommended to instead adjust via desktop app)
#else
// If you have SFU1605 ballscrew, adjust below:
const float MM_PER_REV = 5.0f;                                 // distance in mm per revolution
const float MAX_REVOLUTIONS = 17.5;                            // maximum revolutions for ballscrew (adjust depending on custom length, ie. 14 = 140mm)
const int32_t STEPS_PER_REVOLUTIONS = 1000;                    // Steps per revolution --> verify servo drive ratio is correct as well
const int32_t SAFE_DIST_IN_STEPS = STEPS_PER_REVOLUTIONS / 2;  // Safe traveling distance in steps (do not adjust)
#define MAX_SPEED_MM_SEC 120  // maximum speed mm/sec (can adjust only up to 200 for 1605 ballscrews due to physical limitations)
#endif
//ADJUST VALUES ABOVE BASED ON YOUR PLATFORM SPECIFICATIONS
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

const int32_t RANGE = (int32_t)(MAX_REVOLUTIONS * STEPS_PER_REVOLUTIONS);  // Maximum traveling distance, steps
const int32_t MIN_POS = SAFE_DIST_IN_STEPS;                                // minimal controlled position, steps
const int32_t MAX_POS = RANGE - SAFE_DIST_IN_STEPS;                        // maximal controlled position, steps
const uint8_t HOME_DIRECTION = HIGH;

#define MIN_PULSE_DELAY 5  // Minimal pulse interval, us, may be limited by hardware.

#define MIN_REVERSE_DELAY 6  // Delay between DIR and STEP signal on direction change, us

#define MMPERSEC2DELAY(mmps) 1000000 / (STEPS_PER_REVOLUTIONS * mmps / MM_PER_REV)

#define MIN_SPEED_MM_SEC 10
#define SLOW_SPEED_MM_SEC 10
#define DEFAULT_SPEED_MM_SEC 90

#ifndef MAX
#define MAX(a, b) (a > b ? a : b)
#endif

#ifndef MIN
#define MIN(a, b) (a < b ? a : b)
#endif

const int HOMEING_PULSE_DELAY = MAX(MIN_PULSE_DELAY, (int)MMPERSEC2DELAY(MIN_SPEED_MM_SEC) - MIN_PULSE_DELAY);     // us
const int FAST_PULSE_DELAY = MAX(MIN_PULSE_DELAY, (int)MMPERSEC2DELAY(DEFAULT_SPEED_MM_SEC) - MIN_PULSE_DELAY);    // us
const int SLOW_PULSE_DELAY = MAX(FAST_PULSE_DELAY * 2, (int)MMPERSEC2DELAY(SLOW_SPEED_MM_SEC) - MIN_PULSE_DELAY);  // us

volatile int iFastPulseDelay = FAST_PULSE_DELAY;
volatile int iSlowPulseDelay = SLOW_PULSE_DELAY;
int iFastPulseDelayMM = MAX_SPEED_MM_SEC;

volatile int32_t targetPos = (MIN_POS + MAX_POS) / 2;
volatile uint8_t limitSwitchState = HIGH;
volatile bool bHomed = false;
volatile int32_t currentPos = 0;
volatile uint8_t currentDir = LOW;

volatile bool LimitChanged = true;

// ========== TIMER-BASED STEPPING VARIABLES ==========
HardwareTimer stepTimer(2);  // Timer 2 for Roger Clark core
volatile bool steppingEnabled = false;
volatile bool stepPinState = false;         // Track STEP pin state for pulse generation
volatile uint32_t currentFrequency = 1000;  // Current step frequency in Hz
volatile uint32_t accelStepCount = 0;       // Steps taken for acceleration tracking
// ====================================================

// ========== PID CONTROL VARIABLES ==========
struct PIDController {
  float Kp = 1.5f;
  float Ki = 0.0f;
  float Kd = 0.02f;
  float Ks = 0.50f;  // Derivative smoothing ratio (0..1)

  float integralLimit = 400.0f;
  float maxFreq = 40000.0f;
  float minFreq = 100.0f;

  float integral = 0.0f;
  float prevError = 0.0f;
  float derivativeFilter = 0.0f;

  uint32_t lastUpdateTime = 0;
} pid;

volatile int32_t maxAccelSteps = (int32_t)(25000.0f * STEPS_PER_REVOLUTIONS / MM_PER_REV);
volatile bool pidControlEnabled = false;
float pidBlend = 0.35f;  // 0..1 scales PID output intensity
float limitedFrequencyHz = 0.0f;
uint32_t frequencyLimiterLastMs = 0;

const float PID_KP_MIN = 0.0f;
const float PID_KP_MAX = 200.0f;
const float PID_KI_MIN = 0.0f;
const float PID_KI_MAX = 50.0f;
const float PID_KD_MIN = 0.0f;
const float PID_KD_MAX = 50.0f;
const float PID_KS_MIN = 0.0f;
const float PID_KS_MAX = 1.0f;

float computePID(int32_t currentPosition, int32_t targetPosition, uint32_t currentTime) {
  float dt = (currentTime - pid.lastUpdateTime) / 1000.0f;
  if (dt <= 0.0f || dt > 0.5f) {
    dt = 0.001f;
  }
  pid.lastUpdateTime = currentTime;

  float error = (float)(targetPosition - currentPosition);
  float proportional = pid.Kp * error;

  if (fabs(error) < 800.0f) {
    pid.integral += error * dt;
    pid.integral = clamp(pid.integral, -pid.integralLimit, pid.integralLimit);
  } else {
    pid.integral *= 0.9f;
  }
  float integral = pid.Ki * pid.integral;

  float derivative = (error - pid.prevError) / dt;
  pid.derivativeFilter = pid.Ks * pid.derivativeFilter + (1.0f - pid.Ks) * derivative;
  float derivativeTerm = pid.Kd * pid.derivativeFilter;
  pid.prevError = error;

  float output = proportional + integral + derivativeTerm;
  output *= pidBlend;

  float freq = fabs(output);
  if (freq < pid.minFreq && fabs(error) < 2.0f) {
    return 0.0f;
  }

  if (freq < pid.minFreq) freq = pid.minFreq;
  if (freq > pid.maxFreq) freq = pid.maxFreq;
  return freq;
}

void resetPID() {
  pid.integral = 0.0f;
  pid.prevError = (float)(targetPos - currentPos);
  pid.derivativeFilter = 0.0f;
  pid.lastUpdateTime = millis();
}

uint32_t ApplyFrequencyAccelLimit(uint32_t desiredFreqHz) {
  uint32_t nowMs = millis();
  if (frequencyLimiterLastMs == 0) {
    frequencyLimiterLastMs = nowMs;
    limitedFrequencyHz = (float)desiredFreqHz;
    return desiredFreqHz;
  }

  float dt = (nowMs - frequencyLimiterLastMs) / 1000.0f;
  if (dt <= 0.0f || dt > 0.5f) {
    dt = 0.001f;
  }
  frequencyLimiterLastMs = nowMs;

  float maxDelta = (float)maxAccelSteps * dt;
  float delta = (float)desiredFreqHz - limitedFrequencyHz;

  if (fabs(delta) > maxDelta) {
    limitedFrequencyHz += (delta > 0.0f) ? maxDelta : -maxDelta;
  } else {
    limitedFrequencyHz = (float)desiredFreqHz;
  }

  if (limitedFrequencyHz < 0.0f) limitedFrequencyHz = 0.0f;
  if (limitedFrequencyHz > 40000.0f) limitedFrequencyHz = 40000.0f;
  return (uint32_t)(limitedFrequencyHz + 0.5f);
}
// ====================================================

// Convert mm/sec to step frequency (Hz)
inline uint32_t mmPerSecToFreq(uint32_t mmPerSec) {
  return (uint32_t)((float)STEPS_PER_REVOLUTIONS * (float)mmPerSec / MM_PER_REV);
}

// Convert delay (us) to frequency (Hz)
inline uint32_t delayToFreq(uint32_t delayUs) {
  if (delayUs < MIN_PULSE_DELAY) delayUs = MIN_PULSE_DELAY;
  return 1000000 / (delayUs + MIN_PULSE_DELAY);
}

// ========== TIMER ISR - GENERATES STEP PULSES ==========
void timerISR() {
  // Check if we should stop stepping
  if (!steppingEnabled || currentPos == targetPos) {
    steppingEnabled = false;
    GPIOA->regs->BSRR = (1 << (STEP_PIN_BIT + 16));  // Ensure STEP is LOW
    stepPinState = false;
    return;
  }

  // Check limit switch - stop if on limit and moving towards it
  if (limitSwitchState == HIGH && currentDir == HOME_DIRECTION) {
    steppingEnabled = false;
    GPIOA->regs->BSRR = (1 << (STEP_PIN_BIT + 16));  // Ensure STEP is LOW
    stepPinState = false;
    return;
  }

  // Alternate between HIGH and LOW to create proper pulse width
  if (!stepPinState) {
    // Rising edge - set STEP HIGH
    GPIOA->regs->BSRR = (1 << STEP_PIN_BIT);
    stepPinState = true;
  } else {
    // Falling edge - set STEP LOW and update position
    GPIOA->regs->BSRR = (1 << (STEP_PIN_BIT + 16));
    stepPinState = false;

    // Update position on falling edge (one complete step)
    if (currentDir == HIGH) {
      currentPos++;
    } else {
      currentPos--;
    }
    accelStepCount++;
  }
}

// ========== INITIALIZE TIMER ==========
void initStepTimer() {
  stepTimer.pause();
  stepTimer.setPrescaleFactor(1);  // 72MHz / 1 = 72MHz timer clock
  stepTimer.setOverflow(36000);    // Initial value, will be updated dynamically
  stepTimer.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);
  stepTimer.attachInterrupt(TIMER_CH1, timerISR);
  stepTimer.refresh();
  stepTimer.resume();
}

// ========== SET STEP FREQUENCY ==========
void setStepFrequency(uint32_t freqHz) {
  if (freqHz < 100) freqHz = 100;      // Minimum 100 Hz
  if (freqHz > 40000) freqHz = 40000;  // Maximum 40 kHz

  currentFrequency = freqHz;

  // Timer must run at 2x frequency since we alternate HIGH/LOW
  // Each step requires 2 ISR calls: one for HIGH, one for LOW
  uint32_t timerFreq = freqHz * 2;

  // Calculate overflow value: 72,000,000 / (frequency * 2)
  uint32_t overflow = 72000000UL / timerFreq;
  stepTimer.setOverflow(overflow);
}

// ========== START STEPPING ==========
void startStepping(uint8_t dir) {
  if (currentDir != dir) {
    // Direction change - add delay
    steppingEnabled = false;
    GPIOA->regs->BSRR = (1 << (STEP_PIN_BIT + 16));  // Ensure STEP is LOW
    stepPinState = false;
    delayMicroseconds(MIN_REVERSE_DELAY);

    // Set direction pin using direct GPIO
    if (dir == HIGH) {
      GPIOA->regs->BSRR = (1 << DIR_PIN_BIT);
    } else {
      GPIOA->regs->BSRR = (1 << (DIR_PIN_BIT + 16));
    }

    currentDir = dir;
    accelStepCount = 0;
  }

  stepPinState = false;  // Always start with LOW state
  steppingEnabled = true;
}

// ========== STOP STEPPING ==========
void stopStepping() {
  steppingEnabled = false;
  GPIOA->regs->BSRR = (1 << (STEP_PIN_BIT + 16));  // Ensure STEP is LOW when stopped
  stepPinState = false;
  limitedFrequencyHz = 0.0f;
  frequencyLimiterLastMs = millis();
}

// ========== LEGACY BLOCKING STEP (UNUSED, KEPT FOR REFERENCE) ==========
// This function is no longer called - replaced by timer ISR
/*
inline void Step(uint8_t dir, int delay) {
  if (limitSwitchState == HIGH && dir == HOME_DIRECTION)
    return;  // ON LIMIT SWITCH

  digitalWrite(dirPin, dir);
  if (_oldDir != dir) {
    last_step_time = 0;
    delayMicroseconds(MIN_REVERSE_DELAY);
    _oldDir = dir;
  }

  double delta = MIN(MAX((micros() - last_step_time), 1), iSlowPulseDelay);

  double acc_pulse = pulse - (double)accel * delta / 1000000.0;
  pulse = acc_pulse > delay ? acc_pulse : delay;

  last_step_time = micros();
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(MIN_PULSE_DELAY);
  digitalWrite(stepPin, LOW);
  delayMicroseconds((uint32_t)pulse);
  currentPos += dir == HIGH ? 1 : -1;
}
*/

// Limit switch event
void OnLimitSwitch() {
  // Refresh immediately so timerISR() sees the latest switch state.
  limitSwitchState = digitalRead(limiterPinNO);
  LimitChanged = true;
}

void receiveEvent(int size) {
  uint8_t cmd;
  uint32_t data;
  if (size >= 5 && Wire.readBytes(&cmd, 1) == 1 && Wire.readBytes((uint8_t*)&data, sizeof(uint32_t)) == sizeof(uint32_t)) {
    // While in ALARM, only allow clear alarm or reassert alarm commands.
    if (mode == MODE::ALARM && cmd != COMMAND::CMD_CLEAR_ALARM && cmd != COMMAND::SET_ALARM) {
      return;
    }
    switch (cmd) {
      case COMMAND::CMD_HOME:
        // Ignore repeated HOME when already homed and ready.
        if (mode == MODE::READY && bHomed) {
          break;
        }
        if (mode != MODE::HOMEING) {
          // Immediate homing entry: stop any active stepping before mode switch.
          stopStepping();
          setStepFrequency(delayToFreq(HOMEING_PULSE_DELAY));
          accelStepCount = 0;

          // Force homing direction output now.
          if (HOME_DIRECTION == HIGH) {
            GPIOA->regs->BSRR = (1 << DIR_PIN_BIT);
          } else {
            GPIOA->regs->BSRR = (1 << (DIR_PIN_BIT + 16));
          }
          currentDir = HOME_DIRECTION;

          currentPos = 0;
          targetPos = RANGE * 1.2;
          bHomed = false;
          mode = MODE::HOMEING;
          resetPID();
        }
        break;
      case COMMAND::CMD_PARK:
        if (bHomed) {
          mode = MODE::PARKING;
          resetPID();
        }
        break;
      case COMMAND::CMD_MOVE:
        if (bHomed && mode == MODE::READY) {
          targetPos = clamp((int32_t)data, (int32_t)MIN_POS, (int32_t)MAX_POS);
        }
        break;
      case COMMAND::CMD_CLEAR_ALARM:
      case COMMAND::CMD_ENABLE:
        {
          LimitChanged = true;
          resetPID();

          if (bHomed)
            mode = MODE::READY;
          else
            mode = MODE::CONNECTED;
        }
        break;
      case COMMAND::CMD_DISABLE:
        mode = MODE::DISABLED;
        stopStepping();
        break;
      case COMMAND::CMD_SET_SPEED:
        iFastPulseDelayMM = data = clamp((int)data, MIN_SPEED_MM_SEC, MAX_SPEED_MM_SEC);
        iFastPulseDelay = MAX(MIN_PULSE_DELAY, (int)MMPERSEC2DELAY(data) - MIN_PULSE_DELAY);  // us
        pid.maxFreq = (float)mmPerSecToFreq(data);
        break;
      case COMMAND::CMD_SET_SLOW_SPEED:
        iSlowPulseDelay = MAX(iFastPulseDelay, (int)MMPERSEC2DELAY(data) - MIN_PULSE_DELAY);  // us
        break;
      case COMMAND::CMD_SET_ACCEL:
        accel = data;
        maxAccelSteps = (int32_t)((float)accel * STEPS_PER_REVOLUTIONS / MM_PER_REV);
        break;
      case COMMAND::CMD_SET_PID_KP:
        pid.Kp = clamp((float)data / 10.0f, PID_KP_MIN, PID_KP_MAX);
        resetPID();
        break;
      case COMMAND::CMD_SET_PID_KI:
        pid.Ki = clamp((float)data / 10.0f, PID_KI_MIN, PID_KI_MAX);
        resetPID();
        break;
      case COMMAND::CMD_SET_PID_KD:
        pid.Kd = clamp((float)data / 100.0f, PID_KD_MIN, PID_KD_MAX);
        resetPID();
        break;
      case COMMAND::CMD_SET_PID_KS:
        pid.Ks = clamp((float)data / 100.0f, PID_KS_MIN, PID_KS_MAX);
        resetPID();
        break;
      case COMMAND::CMD_SET_PID_ENABLE:
        pidControlEnabled = (data != 0);
        stopStepping();
        resetPID();
        break;
      case COMMAND::CMD_SET_PID_BLEND:
        pidBlend = clamp((float)data / 100.0f, 0.0f, 1.0f);
        resetPID();
        break;
      case COMMAND::SET_ALARM:
        bHomed = false;
        mode = MODE::ALARM;
        stopStepping();
        resetPID();
        break;
    }
  }
}
void requestEvent() {
  if (mode == MODE::UNKNOWN)
    mode = MODE::CONNECTED;
  STATE state = { mode, (FLAGS)((limitSwitchState == HIGH ? FLAGS::STATE_ON_LIMIT_SWITCH : 0) | (bHomed ? FLAGS::STATE_HOMED : 0)), (uint16_t)iFastPulseDelayMM, currentPos, targetPos, MIN_POS, MAX_POS };
  Wire.write((uint8_t*)&state, STATE_LEN);
}

// Initialization
void setup() {
  allPinsPulldown();

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(limiterPinNO, INPUT_PULLDOWN);
  pinMode(limiterPinNC, INPUT_PULLDOWN);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(stepPin, LOW);
  digitalWrite(dirPin, LOW);

  targetPos = (MIN_POS + MAX_POS) / 2;

  limitSwitchState = digitalRead(limiterPinNO);

  attachInterrupt(limiterPinNO, OnLimitSwitch, CHANGE);

  // Initialize timer-based stepping
  initStepTimer();
  resetPID();

  Wire.begin(SLAVE_ADDR);
  // Wire.setClock(400000);  // Reverted due to connectivity issues: use default I2C clock
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  mode = MODE::UNKNOWN;
  LimitChanged = true;
  currentPos = 0;
}

// Main function
void loop() {
  if (LimitChanged) {
    LimitChanged = false;
    limitSwitchState = digitalRead(limiterPinNO);
    digitalWrite(LED_PIN, !limitSwitchState);
  }

  switch (mode) {
    case MODE::HOMEING:
      {
        if (limitSwitchState == HIGH)  // SITTING ON SWITCH
        {
          stopStepping();

          // Move away from switch - set target correctly based on direction
          if (HOME_DIRECTION == HIGH) {
            targetPos = currentPos - SAFE_DIST_IN_STEPS;  // Moving negative (away from switch)
          } else {
            targetPos = currentPos + SAFE_DIST_IN_STEPS;  // Moving positive (away from switch)
          }

          uint32_t homeFreq = delayToFreq(HOMEING_PULSE_DELAY);
          setStepFrequency(homeFreq);
          startStepping(!HOME_DIRECTION);

          // Wait based on direction
          if (HOME_DIRECTION == HIGH) {
            while (currentPos > targetPos) {
              if (mode == MODE::ALARM || !steppingEnabled) {
                break;
              }
              delay(1);
            }
            if (mode == MODE::ALARM || currentPos > targetPos) {
              mode = MODE::ALARM;
              bHomed = false;
              stopStepping();
              break;
            }
          } else {
            while (currentPos < targetPos) {
              if (mode == MODE::ALARM || !steppingEnabled) {
                break;
              }
              delay(1);
            }
            if (mode == MODE::ALARM || currentPos < targetPos) {
              mode = MODE::ALARM;
              bHomed = false;
              stopStepping();
              break;
            }
          }

          stopStepping();

          // Re-read limit switch state after backing off
          limitSwitchState = digitalRead(limiterPinNO);

          currentPos = MAX_POS;
          targetPos = (MIN_POS + MAX_POS) / 2;

          // Move to center at homing speed
          setStepFrequency(homeFreq);
          uint8_t centerDir = (targetPos > currentPos) ? HIGH : LOW;  // Calculate correct direction
          startStepping(centerDir);

          while (targetPos != currentPos) {
            if (mode == MODE::ALARM || !steppingEnabled) {
              break;
            }
            delay(1);
          }
          if (mode == MODE::ALARM || targetPos != currentPos) {
            mode = MODE::ALARM;
            bHomed = false;
            stopStepping();
            break;
          }

          stopStepping();
          resetPID();
          mode = MODE::READY;
          bHomed = true;
          break;
        }

        digitalWrite(LED_PIN, (millis() % 500) > 250 ? HIGH : LOW);

        if (abs(currentPos) >= RANGE * 1.2)  // SWITCH NOT FOUND
        {
          mode = MODE::ALARM;
          bHomed = false;
          stopStepping();
          break;
        }

        // Continue homing - only start stepping if not already stepping
        if (!steppingEnabled) {
          uint32_t homeFreq = delayToFreq(HOMEING_PULSE_DELAY);
          setStepFrequency(homeFreq);
          startStepping(HOME_DIRECTION);
        }
      }
      break;

    case MODE::PARKING:
      {
        stopStepping();
        targetPos = MIN_POS;
        uint32_t homeFreq = delayToFreq(HOMEING_PULSE_DELAY);
        setStepFrequency(homeFreq);

        uint8_t dir = (targetPos > currentPos) ? HIGH : LOW;
        startStepping(dir);

        while (targetPos != currentPos) {
          if (mode == MODE::ALARM || !steppingEnabled) {
            break;
          }
          delay(1);
        }
        if (mode == MODE::ALARM || targetPos != currentPos) {
          mode = MODE::ALARM;
          bHomed = false;
          stopStepping();
          break;
        }

        stopStepping();
        resetPID();
        mode = MODE::READY;
      }
      break;

    case MODE::READY:
      {
        int32_t error = targetPos - currentPos;

        if (abs(error) > 2) {
          uint8_t dir = (error > 0) ? HIGH : LOW;
          uint32_t desiredFreqHz = 0;

          if (pidControlEnabled) {
            uint32_t now = millis();
            float freq = computePID(currentPos, targetPos, now);
            desiredFreqHz = (uint32_t)freq;
          } else {
            long dist = constrain(abs(error), 0, STEPS_CONTROL_DIST);
            uint32_t minFreq = delayToFreq(iSlowPulseDelay);
            uint32_t maxFreq = delayToFreq(iFastPulseDelay);
            desiredFreqHz = map(dist, 0, STEPS_CONTROL_DIST, minFreq, maxFreq);
          }

          uint32_t freqHz = ApplyFrequencyAccelLimit(desiredFreqHz);

          if (freqHz > 0) {
            setStepFrequency(freqHz);
            if (!steppingEnabled) {
              if (pidControlEnabled) resetPID();
              startStepping(dir);
            } else if (currentDir != dir) {
              stopStepping();
              delayMicroseconds(100);
              if (pidControlEnabled) resetPID();
              startStepping(dir);
            }
          } else {
            stopStepping();
          }
        } else {
          stopStepping();
          if (pidControlEnabled) {
            pid.integral *= 0.95f;
          }
        }
      }
      break;

    case MODE::ALARM:
      digitalWrite(LED_PIN, (millis() % 250) > 125 ? HIGH : LOW);
      stopStepping();
      break;

    case MODE::DISABLED:
      stopStepping();
      break;

    default:
      break;
  }
}
#endif
