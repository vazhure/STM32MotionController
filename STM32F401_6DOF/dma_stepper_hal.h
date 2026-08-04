// =============================================================================
// 6DOF Stewart platform synchronized STEP/DIR HAL
// Target: STM32F401 / STM32F411 Black Pill boards
// Developer: Andrey Zhuravlev
// Email: v.azhure@gmail.com
// Discord: https://discord.gg/ynHCkrsmMA
//
// Changelog:
// 2026-08-04 v0.1.1
// - Added synchronized parallel homing for all axes.
// - Each axis stops individually when its own limit switch is triggered.
// - Remaining axes continue moving until all limit switches are triggered.
// - After all axes reach limit switches, all axes retract simultaneously.
// - Added DONT_USE_ENDSTOPS mode for hard-stop homing without limit switches.
// - Removed old sequential single-axis homing logic.
// - Added configurable homing direction macros.
// =============================================================================

#ifndef DMA_STEPPER_6DOF_H
#define DMA_STEPPER_6DOF_H

#include <Arduino.h>

// =============================================================================
// USER CONFIGURATION
// =============================================================================

// Number of physical actuators.
#define NUM_AXES 6

// Mechanical parameters.
#define MM_PER_REV        10.0f     // Millimeters per motor revolution, adjust.
#define STEPS_PER_REV     1000      // Microsteps per revolution, adjust.
#define STROKE_MM         400.0f    // Actuator usable stroke, adjust.

#define MM_PER_STEP       (MM_PER_REV / (float)STEPS_PER_REV)
#define STEPS_PER_MM      ((float)STEPS_PER_REV / MM_PER_REV)

// Absolute position limits in steps after homing / SET_HOME.
#define MIN_POS_STEPS     0
#define MAX_POS_STEPS     ((int32_t)(STROKE_MM * STEPS_PER_MM))

// Uncomment this line to disable limit switches.
// In this mode the controller assumes that all actuators are already at the
// mechanical lower position when homing is started. It will only move away
// by HOMING_RETRACT_DISTANCE_MM and set that point as zero reference.
//#define DONT_USE_ENDSTOPS

// Limit switch electrical polarity.
// NC switch between LIMIT pin and GND with INPUT_PULLUP:
// idle = LOW, triggered = HIGH.
// Set to 0 if your switches are inverted.
#define LIMIT_ACTIVE_HIGH 1

// Homing direction settings.
// false usually means toward minimum / lower position / zero endstop.
// true usually means away from endstop / upward / retract.
// If your platform moves in the wrong direction during homing, invert these.
#define HOMING_SEEK_DIRECTION false
#define HOMING_RETRACT_DIRECTION true

// Homing speed in steps per second.
#define HOMING_FREQUENCY_HZ 1000UL

// Distance to move away from the limit switch or mechanical hard stop.
#define HOMING_RETRACT_DISTANCE_MM 3.0f
#define HOMING_RETRACT_STEPS ((int32_t)(HOMING_RETRACT_DISTANCE_MM * STEPS_PER_MM))

// Homing timeouts.
#define HOMING_SEEK_TIMEOUT_MS    30000UL
#define HOMING_RETRACT_TIMEOUT_MS 10000UL

// Safety overflow limit while seeking endstops.
// If an axis travels farther than this without triggering a limit switch,
// homing is aborted with alarm.
#define HOMING_OVERFLOW_LIMIT_MULT 1.5f
#define HOMING_SEEK_OVERFLOW_STEPS \
    ((int32_t)((float)MAX_POS_STEPS * HOMING_OVERFLOW_LIMIT_MULT))

// =============================================================================
// INTERPOLATION TIMER SETTINGS
// =============================================================================

// Timer tick frequency.
// Step pulse is high one tick and low one tick.
// Therefore max step frequency per axis = ISR_TICK_HZ / 2.
#define ISR_TICK_HZ       100000UL
#define MAX_PULSE_HZ      50000UL

// Frame timing limits.
#define MIN_FRAME_US      1000UL        // 1 ms
#define MAX_FRAME_US      1000000UL     // 1000 ms
#define DEFAULT_FRAME_US  10000UL       // 10 ms = 100 Hz

// Direction setup delay before first step of a segment, in timer ticks.
#define DIR_SETTLE_TICKS  4UL

// Frame queue length.
// Small queue gives low latency; large queue smooths USB jitter.
#define FRAME_QUEUE_LEN   8

// =============================================================================
// SAFETY SETTINGS
// =============================================================================

#define COMM_TIMEOUT_MS   200UL

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
    ERR_INTERNAL        = 7,
    ERR_HOMING_OVERFLOW = 8
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

// Starts synchronized homing for all axes simultaneously.
void DMAStepper_StartHoming(void);

void DMAStepper_SetHome(void);
void DMAStepper_ClearAlarm(void);
void DMAStepper_EmergencyStop(uint8_t err);

void DMAStepper_NotifyValidPacket(void);

#endif // DMA_STEPPER_6DOF_H