// 3DOF by Andrey Zhuravlev
// v.azhure@gmail.com
// Discord: https://discord.gg/ynHCkrsmMA
// ============================================================================
// CHANGELOG:
// 2026-08-19: Added COMMAND enum to make it visible in both .cpp and .ino files.
// 2026-08-18: Merged 6-axis SPI Master/Slave support with advanced limit switch config.
// ============================================================================
#ifndef DMA_STEPPER_HAL_H
#define DMA_STEPPER_HAL_H
#include <Arduino.h>

// =============================================================================
// 🛠️ USER CONFIGURATION (Modify only these values)
// =============================================================================
#define MM_PER_REV 8.0f
#define STEPS_PER_REV 1600
#define MAX_REVOLUTIONS 20.0f

#define HOME_DIRECTION -1
#define END_CLEARANCE_MM 3.0f

#define LIMIT_TYPE_NC 0
#define LIMIT_TYPE_NO 1
#define LIMIT_PULL_UP 0
#define LIMIT_PULL_DOWN 1
#define LIMIT_PULL_AUTO 2

#ifndef LIMIT_SWITCH_TYPE
#define LIMIT_SWITCH_TYPE LIMIT_TYPE_NO
#endif
#ifndef LIMIT_PULL_MODE
#define LIMIT_PULL_MODE LIMIT_PULL_AUTO
#endif

#if (LIMIT_PULL_MODE == LIMIT_PULL_AUTO)
#if (LIMIT_SWITCH_TYPE == LIMIT_TYPE_NC)
#define _RESOLVED_PULL LIMIT_PULL_UP
#else
#define _RESOLVED_PULL LIMIT_PULL_DOWN
#endif
#else
#define _RESOLVED_PULL LIMIT_PULL_MODE
#endif

#if (LIMIT_SWITCH_TYPE == LIMIT_TYPE_NC)
#if (_RESOLVED_PULL == LIMIT_PULL_UP)
#define LIMIT_ACTIVE_HIGH 1
#else
#define LIMIT_ACTIVE_HIGH 0
#endif
#else
#if (_RESOLVED_PULL == LIMIT_PULL_UP)
#define LIMIT_ACTIVE_HIGH 0
#else
#define LIMIT_ACTIVE_HIGH 1
#endif
#endif

#define END_CLEARANCE_STEPS ((int32_t)(END_CLEARANCE_MM * (float)STEPS_PER_REV / (float)MM_PER_REV))

// =============================================================================
// MULTI-CONTROLLER CONFIGURATION
// =============================================================================
#define AXES_PER_BOARD 3
#define AXES_TOTAL 6

#define CONTROLLER_MODE_MASTER 0
#define CONTROLLER_MODE_SLAVE 1

#ifndef CONTROLLER_MODE
#define CONTROLLER_MODE CONTROLLER_MODE_SLAVE
#endif

#if (CONTROLLER_MODE == CONTROLLER_MODE_SLAVE)
#define LOGICAL_AXIS_OFFSET AXES_PER_BOARD
#else
#define LOGICAL_AXIS_OFFSET 0
#endif

#define SPI_SCK_PIN PA5
#define SPI_MISO_PIN PA6
#define SPI_MOSI_PIN PA7
#define SPI_NSS_PIN PA4
#define SYNC_PIN PB0
#define ALARM_PIN PB1

#define SPI_HEADER_MAGIC 0xAA

// 2026-08-19 FIX: Double underscore for packed attribute
typedef struct __attribute__((packed)) {
  uint8_t header;
  uint8_t frameId;
  uint8_t command;
  uint8_t flags;
  int32_t cmdData[3];
  int32_t positions[3];
  int32_t targets[3];
  uint8_t modes[3];
  uint8_t slaveStatus;
} SPI_FRAME;

#define SPI_FLAG_SYNC 0x01
#define SPI_FLAG_ALARM 0x02

// =============================================================================
// PROTOCOL COMMANDS (Moved here so .cpp and .ino can both see them)
// =============================================================================
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

// =============================================================================
// UNIT CONVERSION MACROS
// =============================================================================
#define MMPERSECTOFREQHZ(mmpers) ((uint32_t)((float)(mmpers) * (float)STEPS_PER_REV / (float)MM_PER_REV))
#define FREQHZTOMMPERSEC(freqhz) ((float)(freqhz) * (float)MM_PER_REV / (float)STEPS_PER_REV)
#define MMPERSECTOFREQHZ_SAFE(mmpers) constrain(MMPERSECTOFREQHZ(mmpers), MIN_FREQUENCY_HZ, MAX_FREQUENCY_HZ)

#define MAX_FREQUENCY_HZ 200000
#define SAFE_FREQUENCY_HZ 100000
#define DEFAULT_FREQUENCY_HZ SAFE_FREQUENCY_HZ
#define MIN_FREQUENCY_HZ 50

#define HOMING_FREQUENCY_HZ MMPERSECTOFREQHZ_SAFE(15)
#define PARKING_FREQUENCY_HZ MMPERSECTOFREQHZ_SAFE(15)
#define HOMING_RETRACT_DISTANCE_MM 3.0f
#define HOMING_RETRACT_DURATION_MS ((uint32_t)((HOMING_RETRACT_DISTANCE_MM * (float)STEPS_PER_REV * 1000.0f) / ((float)HOMING_FREQUENCY_HZ * MM_PER_REV)))
#define HOMING_SEEK_TIMEOUT_MS 30000
#define HOMING_CENTER_TIMEOUT_MS 30000
#define MAX_ACCEL 80000
#define HOMING_TRAVEL_LIMIT_MULT 1.5f
#define HOMING_OVERFLOW_LIMIT_MULT 1.4f
#define POSITION_TOLERANCE 50
#define POSITION_DEADZONE 2
#define ACCEL_RAMP_DISTANCE 50
#define DIRECTION_CHANGE_DELAY_US 10
#define HOMING_CENTER_TOLERANCE 100
#define HOMING_RETRACT_SETTLE_MS 50

// =============================================================================
// SYSTEM CONSTANTS
// =============================================================================
#define MM_PER_STEP (MM_PER_REV / STEPS_PER_REV)
#define MAX_SPEED_MM_SEC (int)(MAX_FREQUENCY_HZ * MM_PER_STEP)
#define SAFE_SPEED_MM_SEC (int)(SAFE_FREQUENCY_HZ * MM_PER_STEP)
#define DEFAULT_SPEED_MM_SEC (int)(DEFAULT_FREQUENCY_HZ * MM_PER_STEP)
#define NUM_AXES AXES_PER_BOARD
#define PENDING_TARGET_NONE INT32_MIN

// =============================================================================
// DATA STRUCTURES
// =============================================================================
typedef struct {
  uint8_t stepPin;
  uint8_t dirPin;
  uint8_t limitPin;
} AxisConfig;

typedef struct {
  volatile int32_t currentPosition;
  volatile int32_t targetPosition;
  volatile int32_t minPos;
  volatile int32_t maxPos;
  volatile uint32_t frequency;
  volatile bool direction;
  volatile bool stepping;
  volatile bool homed;
  volatile uint8_t mode;
  uint32_t maxFreqHz;
  uint32_t maxSpeedMM;
  float limitedFreq;
  uint32_t accelLastTime;
  uint32_t pidLastTime;
  uint32_t maxAccel;
  float Kp, Ki, Kd, Ks;
  float integral;
  float prevError;
  float derivativeFilter;
  float pidMinFreq;
  float pidMaxFreq;
  bool pidEnabled;
  float pidBlend;
  int32_t pendingTarget;
} AxisState;

enum AxisMode {
  MODE_UNKNOWN = 0,
  MODE_CONNECTED = 1,
  MODE_DISABLED = 2,
  MODE_HOMING = 3,
  MODE_PARKING = 4,
  MODE_READY = 5,
  MODE_ALARM = 6,
  MODE_PARKED = 7,
  MODE_UNPARKING = 8
};

enum HomingSubState {
  H_IDLE = 0,
  H_SEEKING,
  H_RETRACT,
  H_RETRACT_SETTLE,
  H_MOVING_CENTER,
  H_DONE
};

// =============================================================================
// API FUNCTIONS
// =============================================================================
void DMAStepper_Init(void);
void DMAStepper_InitAxis(uint8_t axisIdx, uint8_t stepPin, uint8_t dirPin, uint8_t limitPin);
void DMAStepper_SetFrequency(uint8_t axisIdx, uint32_t freqHz);
void DMAStepper_StartAxis(uint8_t axisIdx, bool forward);
void DMAStepper_StopAxis(uint8_t axisIdx);
void DMAStepper_StopAll(void);
void DMAStepper_SetTarget(uint8_t axisIdx, int32_t target);
int32_t DMAStepper_GetPosition(uint8_t axisIdx);
void DMAStepper_SetPosition(uint8_t axisIdx, int32_t pos);
AxisState* DMAStepper_GetAxis(uint8_t axisIdx);
void DMAStepper_SetMaxSpeed(uint8_t axisIdx, uint32_t speedMM);
void DMAStepper_SetPID(uint8_t axisIdx, float Kp, float Ki, float Kd, float Ks);
void DMAStepper_SetPIDEnable(uint8_t axisIdx, bool enable);
void DMAStepper_SetPIDBlend(uint8_t axisIdx, float blend);
bool DMAStepper_CheckLimit(uint8_t axisIdx);
void DMAStepper_ClearAlarm(void);
void DMAStepper_Process(void);
void DMAStepper_StartHoming(uint8_t axisIdx);

void SPI_Controller_Init(void);
void SPI_Controller_Process(void);
void SPI_PingSlave(void); 
bool SPI_Controller_SendCommand(uint8_t cmd, int32_t* data, bool sync);
bool SPI_Controller_IsSlaveReady(void);
void SPI_Controller_TriggerAlarm(void);
SPI_FRAME* SPI_Controller_GetLastFrame(void);

#endif