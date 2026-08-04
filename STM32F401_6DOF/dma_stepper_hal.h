// =============================================================================
// 6DOF Stewart platform synchronized STEP/DIR HAL
// Target: STM32F401 / STM32F411 Black Pill boards
// Developer: Andrey Zhuravlev
// Email: v.azhure@gmail.com
// Discord: https://discord.gg/ynHCkrsmMA
//
// Changelog:
// 2026-08-04 v0.1.2
// - Added user speed limit MAX_SPEED_MM_S.
// - Added driver maximum step frequency limit DRIVER_MAX_PULSE_HZ.
// - Added driver minimum STEP pulse width limits.
// - Added controller ISR safety limit and pulse-width validation.
// - MAX_PULSE_HZ is now automatically limited by the safest constraint.
// - Added optional STRICT_SPEED_CHECK compile-time validation.
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

// =============================================================================
// MECHANICAL PARAMETERS (REQUIRED)
// =============================================================================

// Millimeters per motor revolution.
// Adjust to your actuator screw / gearbox.
#define MM_PER_REV        10.0f

// Microsteps per motor revolution.
// Example: 200 steps motor with 5x microstepping = 1000.
#define STEPS_PER_REV     1000

// Usable actuator stroke in millimeters.
#define STROKE_MM         400.0f

// Derived mechanical constants.
#define MM_PER_STEP       (MM_PER_REV / (float)STEPS_PER_REV)
#define STEPS_PER_MM      ((float)STEPS_PER_REV / MM_PER_REV)

// Absolute position limits in steps after homing / SET_HOME.
#define MIN_POS_STEPS     0
#define MAX_POS_STEPS     ((int32_t)(STROKE_MM * STEPS_PER_MM))

// =============================================================================
// TIMER / CONTROLLER LIMITS (REQUIRED)
// =============================================================================

// Timer tick frequency.
// Current step generation uses:
// one tick STEP HIGH + one tick STEP LOW.
// Therefore maximum full-step frequency = ISR_TICK_HZ / 2.
//
// Example:
// ISR_TICK_HZ = 100000 -> tick = 10 us -> max full step frequency = 50 kHz.
#define ISR_TICK_HZ       100000UL

// Maximum safe interrupt rate for this firmware/MCU.
// For STM32F401/F411 with 6 axes, 100 kHz is safe.
// 200 kHz may be possible but must be tested carefully.
#define CONTROLLER_MAX_SAFE_ISR_HZ 200000UL

// =============================================================================
// DRIVER LIMITS (REQUIRED)
// =============================================================================

// Maximum STEP input frequency supported by the stepper driver.
// Example: many drivers support up to 200 kHz.
// This is an absolute driver limit, regardless of MCU capabilities.
#define DRIVER_MAX_PULSE_HZ 200000UL

// Minimum STEP pulse HIGH time required by the driver, microseconds.
// Check driver datasheet.
// Common values: 1 us, 2.5 us, 5 us.
#define DRIVER_STEP_PULSE_MIN_HIGH_US 5.0f

// Minimum STEP pulse LOW time required by the driver, microseconds.
// Check driver datasheet.
// Common values: 1 us, 2.5 us, 5 us.
#define DRIVER_STEP_PULSE_MIN_LOW_US  5.0f

// =============================================================================
// SPEED LIMIT CONFIGURATION (REQUIRED)
// =============================================================================

// Desired maximum actuator linear speed in millimeters per second.
// This is the user-defined mechanical speed limit.
#define MAX_SPEED_MM_S    150.0f

// If STRICT_SPEED_CHECK = 1, compilation will fail if MAX_SPEED_MM_S
// exceeds controller, driver, or pulse-width capabilities.
//
// If STRICT_SPEED_CHECK = 0, MAX_PULSE_HZ will be silently clamped
// to the safest possible value.
#define STRICT_SPEED_CHECK 1

// =============================================================================
// DERIVED SPEED / PULSE LIMITS
// =============================================================================

// Helper macro for compile-time minimum selection.
#define MIN_U32(a, b) ((a) < (b) ? (a) : (b))

// Step frequency required for MAX_SPEED_MM_S.
// Formula:
// steps/s = mm/s * steps/mm
#define REQUESTED_SPEED_PULSE_HZ \
    ((uint32_t)(MAX_SPEED_MM_S * STEPS_PER_MM))

// Hardware limit caused by ISR step generation.
// One full step requires two ISR ticks: HIGH tick + LOW tick.
#define CONTROLLER_MAX_STEP_HZ \
    ((uint32_t)(ISR_TICK_HZ / 2UL))

// Driver pulse-width limit.
// Minimum full step period = HIGH time + LOW time.
#define DRIVER_STEP_PULSE_PERIOD_US \
    (DRIVER_STEP_PULSE_MIN_HIGH_US + DRIVER_STEP_PULSE_MIN_LOW_US)

// Maximum frequency allowed by required pulse width.
// If period is zero or invalid, return a very large value and let
// static assertions report the invalid pulse width settings.
#define DRIVER_PULSE_WIDTH_MAX_HZ \
    ((DRIVER_STEP_PULSE_PERIOD_US <= 0.0f) \
        ? 0xFFFFFFFFUL \
        : ((uint32_t)(1000000.0f / DRIVER_STEP_PULSE_PERIOD_US)))

// Absolute allowed maximum pulse frequency.
// It is the lowest of:
// - driver maximum step frequency;
// - controller ISR generation limit;
// - driver pulse-width limit.
#define MAX_ALLOWED_PULSE_HZ \
    MIN_U32( \
        MIN_U32(DRIVER_MAX_PULSE_HZ, CONTROLLER_MAX_STEP_HZ), \
        DRIVER_PULSE_WIDTH_MAX_HZ \
    )

// Final maximum pulse frequency used by motion execution.
// It cannot exceed requested user speed and absolute allowed maximum.
#define MAX_PULSE_HZ \
    MIN_U32(REQUESTED_SPEED_PULSE_HZ, MAX_ALLOWED_PULSE_HZ)

// =============================================================================
// COMPILE-TIME VALIDATION
// =============================================================================

// Helper nanosecond constants for integer static assertions.
#define ISR_TICK_PERIOD_NS \
    ((uint32_t)(1000000000.0f / (float)ISR_TICK_HZ))

#define DRIVER_STEP_PULSE_MIN_HIGH_NS \
    ((uint32_t)(DRIVER_STEP_PULSE_MIN_HIGH_US * 1000.0f))

#define DRIVER_STEP_PULSE_MIN_LOW_NS \
    ((uint32_t)(DRIVER_STEP_PULSE_MIN_LOW_US * 1000.0f))

static_assert(ISR_TICK_HZ > 0,
    "ISR_TICK_HZ must be greater than zero.");

static_assert(CONTROLLER_MAX_SAFE_ISR_HZ > 0,
    "CONTROLLER_MAX_SAFE_ISR_HZ must be greater than zero.");

static_assert(ISR_TICK_HZ <= CONTROLLER_MAX_SAFE_ISR_HZ,
    "ISR_TICK_HZ exceeds CONTROLLER_MAX_SAFE_ISR_HZ. "
    "Reduce ISR_TICK_HZ or increase CONTROLLER_MAX_SAFE_ISR_HZ if safe.");

static_assert(DRIVER_STEP_PULSE_MIN_HIGH_NS > 0,
    "DRIVER_STEP_PULSE_MIN_HIGH_US must be greater than zero.");

static_assert(DRIVER_STEP_PULSE_MIN_LOW_NS > 0,
    "DRIVER_STEP_PULSE_MIN_LOW_US must be greater than zero.");

static_assert(ISR_TICK_PERIOD_NS >= DRIVER_STEP_PULSE_MIN_HIGH_NS,
    "STEP pulse HIGH width is too short. "
    "Reduce ISR_TICK_HZ or use a driver with shorter minimum HIGH pulse.");

static_assert(ISR_TICK_PERIOD_NS >= DRIVER_STEP_PULSE_MIN_LOW_NS,
    "STEP pulse LOW width is too short. "
    "Reduce ISR_TICK_HZ or use a driver with shorter minimum LOW pulse.");

static_assert(DRIVER_MAX_PULSE_HZ > 0,
    "DRIVER_MAX_PULSE_HZ must be greater than zero.");

static_assert(MAX_ALLOWED_PULSE_HZ > 0,
    "MAX_ALLOWED_PULSE_HZ must be greater than zero.");

static_assert(MAX_PULSE_HZ > 0,
    "MAX_PULSE_HZ must be greater than zero.");

#if STRICT_SPEED_CHECK
static_assert(REQUESTED_SPEED_PULSE_HZ <= MAX_ALLOWED_PULSE_HZ,
    "MAX_SPEED_MM_S exceeds controller/driver/pulse-width capability. "
    "Reduce MAX_SPEED_MM_S, reduce microstepping, increase ISR_TICK_HZ safely, "
    "or set STRICT_SPEED_CHECK to 0 to clamp automatically.");
#endif

// =============================================================================
// FRAME / INTERPOLATION SETTINGS
// =============================================================================

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
// HOMING SETTINGS
// =============================================================================

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
// Must not exceed MAX_PULSE_HZ.
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

static_assert(HOMING_FREQUENCY_HZ <= MAX_PULSE_HZ,
    "HOMING_FREQUENCY_HZ exceeds MAX_PULSE_HZ. "
    "Reduce HOMING_FREQUENCY_HZ or increase allowed speed limits.");

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