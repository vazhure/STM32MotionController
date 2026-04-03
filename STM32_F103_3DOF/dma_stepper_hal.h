// 3DOF by Andrey Zhuravlev
// v.azhure@gmail.com
// Discord: https://discord.gg/ynHCkrsmMA
#ifndef DMA_STEPPER_HAL_H
#define DMA_STEPPER_HAL_H
#include <Arduino.h>

// =============================================================================
// 🛠️ USER CONFIGURATION (Modify only these values)
// =============================================================================
// Mechanical parameters
#define MM_PER_REV 10.0f      // Millimeters per motor revolution
#define STEPS_PER_REV 1000    // Microsteps per revolution (e.g., 200 steps * 5x microstepping)
#define MAX_REVOLUTIONS 9.0f  // Maximum allowed revolutions before software limit

// =============================================================================
// UNIT CONVERSION MACROS
// =============================================================================
// Convert speed (mm/s) to step frequency (Hz)
// Formula: freq = (mm/s) × (steps/rev) / (mm/rev)
// Example: 10 mm/s × 1000 steps/rev / 10 mm/rev = 1000 Hz
#define MMPERSECTOFREQHZ(mmpers) ((uint32_t)((float)(mmpers) * (float)STEPS_PER_REV / (float)MM_PER_REV))
// Inverse conversion: frequency (Hz) → speed (mm/s)
#define FREQHZTOMMPERSEC(freqhz) ((float)(freqhz) * (float)MM_PER_REV / (float)STEPS_PER_REV)
// Safe frequency conversion with clamping
#define MMPERSECTOFREQHZ_SAFE(mmpers) constrain(MMPERSECTOFREQHZ(mmpers), MIN_FREQUENCY_HZ, MAX_FREQUENCY_HZ)

// Frequency limits (Hz)
#define MAX_FREQUENCY_HZ 200000   // Absolute maximum step frequency
#define SAFE_FREQUENCY_HZ 100000  // Recommended safe operating limit
#define DEFAULT_FREQUENCY_HZ 50000
#define MIN_FREQUENCY_HZ 50  // Minimum reliable step frequency

// Special mode speeds (Hz)
#define HOMING_FREQUENCY_HZ MMPERSECTOFREQHZ_SAFE(15)   // Homing search speed
#define PARKING_FREQUENCY_HZ MMPERSECTOFREQHZ_SAFE(15)  // Parking movement speed

// Homing timeouts (ms)
#define HOMING_SEEK_TIMEOUT_MS 30000    // Max time to find limit switch
#define HOMING_RETRACT_DURATION_MS 300  // Time to back off after hitting limit
#define HOMING_CENTER_TIMEOUT_MS 30000  // Max time to move to center position
#define MAX_ACCEL 80000                 // Max acceleration (steps/s²)

// Travel multipliers (relative to MAX_REVOLUTIONS * STEPS_PER_REV)
#define HOMING_TRAVEL_LIMIT_MULT 1.5f    // Target position during seek (safety margin)
#define HOMING_OVERFLOW_LIMIT_MULT 1.4f  // Emergency overflow limit

// Tolerances and delays
#define POSITION_TOLERANCE 50         // Position reached tolerance (steps)
#define POSITION_DEADZONE 2           // Deadzone to stop movement (steps)
#define ACCEL_RAMP_DISTANCE 50        // Distance for smooth accel/decel ramp (steps)
#define DIRECTION_CHANGE_DELAY_US 10  // Delay when changing direction (µs)
#define HOMING_CENTER_TOLERANCE 100   // Tolerance for centering (steps) - larger than POSITION_TOLERANCE

// =============================================================================
// SYSTEM CONSTANTS (Do not modify)
// =============================================================================
#define MM_PER_STEP (MM_PER_REV / STEPS_PER_REV)
#define MAX_SPEED_MM_SEC (int)(MAX_FREQUENCY_HZ * MM_PER_STEP)
#define SAFE_SPEED_MM_SEC (int)(SAFE_FREQUENCY_HZ * MM_PER_STEP)
#define DEFAULT_SPEED_MM_SEC (int)(DEFAULT_FREQUENCY_HZ * MM_PER_STEP)
#define NUM_AXES 4

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
  uint32_t maxAccel;
  float Kp, Ki, Kd, Ks;
  float integral;
  float prevError;
  float derivativeFilter;
  float pidMinFreq;
  float pidMaxFreq;
  bool pidEnabled;
  float pidBlend;
} AxisState;

enum AxisMode {
  MODE_UNKNOWN = 0,
  MODE_CONNECTED = 1,
  MODE_DISABLED = 2,
  MODE_HOMING = 3,
  MODE_PARKING = 4,
  MODE_READY = 5,
  MODE_ALARM = 6
};

enum HomingSubState {
  H_IDLE = 0,
  H_SEEKING,
  H_RETRACT,
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
#endif