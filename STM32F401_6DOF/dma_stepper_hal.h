// !!!!!! W.I.P. !!!!!
//
// 6DOF by Andrey Zhuravlev
// dma_stepper_hal.h
// 6DOF Stewart platform synchronized HAL for STM32F401
// One high-priority timer + multi-axis DDA interpolation

#ifndef DMA_STEPPER_6DOF_H
#define DMA_STEPPER_6DOF_H

#include <Arduino.h>

// =============================================================================
// USER CONFIGURATION
// =============================================================================

#define NUM_AXES 6

// Mechanical parameters
#define MM_PER_REV        10.0f     // mm per motor revolution, adjust!
#define STEPS_PER_REV     1000      // microsteps per revolution, adjust!
#define STROKE_MM         400.0f    // actuator usable stroke, adjust!

#define MM_PER_STEP       (MM_PER_REV / (float)STEPS_PER_REV)
#define STEPS_PER_MM      ((float)STEPS_PER_REV / MM_PER_REV)

// Absolute position limits in steps after homing / SET_HOME
#define MIN_POS_STEPS     0
#define MAX_POS_STEPS     ((int32_t)(STROKE_MM * STEPS_PER_MM))

// =============================================================================
// INTERPOLATION TIMER SETTINGS
// =============================================================================

// Timer tick frequency.
// Step pulse is high one tick and low one tick.
// Therefore max step frequency per axis = ISR_TICK_HZ / 2.
#define ISR_TICK_HZ       100000UL
#define MAX_PULSE_HZ      50000UL

// Frame timing limits
#define MIN_FRAME_US      1000UL        // 1 ms
#define MAX_FRAME_US      1000000UL     // 1000 ms
#define DEFAULT_FRAME_US  10000UL       // 10 ms = 100 Hz

// Direction setup delay before first step of a segment, in timer ticks
#define DIR_SETTLE_TICKS  4UL

// Frame queue length.
// Small queue gives low latency; large queue smooths USB jitter.
#define FRAME_QUEUE_LEN   8

// =============================================================================
// HOMING SETTINGS
// =============================================================================

#define HOMING_FREQUENCY_HZ       1000UL
#define HOMING_BACKOFF_STEPS      300
#define HOMING_SEEK_TIMEOUT_MS    30000UL
#define HOMING_RETRACT_TIMEOUT_MS 5000UL

// =============================================================================
// SAFETY SETTINGS
// =============================================================================

#define COMM_TIMEOUT_MS   200UL

// Limit switch polarity.
// NC switch between LIMIT and GND with INPUT_PULLUP:
// idle = LOW, triggered = HIGH.
#define LIMIT_ACTIVE_HIGH 1

// =============================================================================
// MODES / ERRORS
// =============================================================================

enum GlobalMode : uint8_t {
  MODE_INIT     = 0,
  MODE_DISABLED = 1,
  MODE_IDLE     = 2,
  MODE_RUNNING  = 3,
  MODE_HOMING   = 4,
  MODE_ALARM    = 5
};

enum StewartError : uint8_t {
  ERR_NONE            = 0,
  ERR_COMM_TIMEOUT    = 1,
  ERR_LIMIT           = 2,
  ERR_QUEUE_FULL      = 3,
  ERR_BAD_PACKET      = 4,
  ERR_HOMING_TIMEOUT  = 5,
  ERR_OVERSPEED       = 6,
  ERR_INTERNAL        = 7
};

// =============================================================================
// API
// =============================================================================

void DMAStepper_Init(void);
void DMAStepper_InitAxis(uint8_t axisIdx, uint8_t stepPin, uint8_t dirPin, uint8_t limitPin);

void DMAStepper_Process(void);

// Queue one synchronized interpolation frame.
// targets[] are absolute actuator positions in steps.
// duration_us is desired execution time of this frame.
bool DMAStepper_QueueFrame(const int32_t targets[NUM_AXES], uint32_t duration_us, uint32_t seq);

void DMAStepper_GetPositions(int32_t pos[NUM_AXES]);
void DMAStepper_GetTargets(int32_t tgt[NUM_AXES]);

uint8_t DMAStepper_GetMode(void);
uint8_t DMAStepper_GetError(void);
uint8_t DMAStepper_GetQueueCount(void);

bool DMAStepper_IsHomed(void);
bool DMAStepper_IsSegmentActive(void);

void DMAStepper_Enable(bool enable);
void DMAStepper_StartHoming(void);
void DMAStepper_SetHome(void);
void DMAStepper_ClearAlarm(void);
void DMAStepper_EmergencyStop(uint8_t err);

void DMAStepper_NotifyValidPacket(void);

#endif // DMA_STEPPER_6DOF_H