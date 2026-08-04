// =============================================================================
// 6DOF Stewart platform synchronized execution for STM32F401 / STM32F411
// Developer: Andrey Zhuravlev
// Email: v.azhure@gmail.com
// Discord: https://discord.gg/ynHCkrsmMA
//
// Changelog:
// 2026-08-04 v0.1.1
// - Implemented synchronized parallel homing state machine.
// - All axes move to zero endstops simultaneously.
// - Each axis stops individually when its limit switch is triggered.
// - When all axes have triggered limit switches, all axes retract together.
// - Added DONT_USE_ENDSTOPS hard-stop homing mode.
// =============================================================================

#include "dma_stepper_hal.h"
#include <HardwareTimer.h>

// =============================================================================
// INTERNAL TYPES
// =============================================================================

typedef struct {
    uint8_t stepPin;
    uint8_t dirPin;
    uint8_t limitPin;

    GPIO_TypeDef* stepPort;
    uint32_t stepBit;

    GPIO_TypeDef* dirPort;
    uint32_t dirBit;

    GPIO_TypeDef* limitPort;
    uint32_t limitBit;
} AxisPins;

typedef struct {
    int32_t target[NUM_AXES];
    uint32_t duration_us;
    uint32_t seq;
} MoveFrame;

enum HomingPhase : uint8_t {
    HP_IDLE = 0,
    HP_SEEKING,
    HP_RETRACT
};

// =============================================================================
// STATIC VARIABLES
// =============================================================================

static HardwareTimer* sharedTimer = nullptr;

static AxisPins pins[NUM_AXES];

static volatile int32_t currentPosition[NUM_AXES] = {0};
static volatile int32_t targetPosition[NUM_AXES]  = {0};

static int32_t minPos[NUM_AXES];
static int32_t maxPos[NUM_AXES];

static volatile bool axisHomed[NUM_AXES] = {false};
static volatile bool allHomed = false;

static volatile uint8_t globalMode = MODE_INIT;
static uint8_t lastError = ERR_NONE;
static uint32_t lastRxMs = 0;

// Frame queue.
static MoveFrame frameQueue[FRAME_QUEUE_LEN];
static volatile uint8_t qHead = 0;
static volatile uint8_t qTail = 0;

// Last accepted/planned endpoint. Used to validate queued frames.
static int32_t plannedTarget[NUM_AXES] = {0};

// Active synchronized segment state.
static volatile bool segmentActive = false;
static uint32_t segTicks = 0;
static uint32_t segSlots = 0;
static uint32_t segTick = 0;
static uint32_t dirSettleTicks = 0;

static uint32_t segSteps[NUM_AXES]  = {0};
static int8_t   segDir[NUM_AXES]    = {1};
static int32_t  lastDir[NUM_AXES]   = {1};
static uint32_t ddaAcc[NUM_AXES]    = {0};
static uint32_t stepsDone[NUM_AXES] = {0};
static volatile bool pulseHigh[NUM_AXES] = {false};

// Synchronized homing state.
static HomingPhase homingPhase = HP_IDLE;
static uint32_t homingTime = 0;

static volatile bool homingActive[NUM_AXES] = {false};
static volatile bool homingPulseHigh[NUM_AXES] = {false};
static uint32_t homingAccum[NUM_AXES] = {0};
static bool homingDir[NUM_AXES] = {false};
static bool homingSeekDone[NUM_AXES] = {false};

// Limit debounce: 1 = released, 32768 = confirmed active.
static uint16_t limitDebounce[NUM_AXES] = {1};
static bool limitConfirmed[NUM_AXES] = {false};

// =============================================================================
// FAST GPIO
// =============================================================================

static inline void stepHigh(uint8_t i) {
    pins[i].stepPort->BSRR = pins[i].stepBit;
}

static inline void stepLow(uint8_t i) {
    pins[i].stepPort->BSRR = (pins[i].stepBit << 16);
}

static inline void setDir(uint8_t i, bool forward) {
    if (forward) {
        pins[i].dirPort->BSRR = pins[i].dirBit;
    } else {
        pins[i].dirPort->BSRR = (pins[i].dirBit << 16);
    }
}

static inline bool readLimitRaw(uint8_t i) {
    return (pins[i].limitPort->IDR & pins[i].limitBit) != 0;
}

// =============================================================================
// HOMING HELPERS
// =============================================================================

static inline int32_t homingRetractStart(void) {
    return HOMING_RETRACT_DIRECTION ? -HOMING_RETRACT_STEPS : HOMING_RETRACT_STEPS;
}

static inline bool homingRetractReached(int32_t pos) {
    return HOMING_RETRACT_DIRECTION ? (pos >= 0) : (pos <= 0);
}

static inline bool homingSeekOverflow(int32_t pos) {
    return HOMING_SEEK_DIRECTION ?
        (pos > HOMING_SEEK_OVERFLOW_STEPS) :
        (pos < -HOMING_SEEK_OVERFLOW_STEPS);
}

static inline void startHomingAxisUnlocked(uint8_t i, bool dir) {
    setDir(i, dir);
    homingDir[i] = dir;
    homingAccum[i] = 0;
    homingPulseHigh[i] = false;
    stepLow(i);
    homingActive[i] = true;
}

static inline void stopHomingAxisUnlocked(uint8_t i) {
    homingActive[i] = false;
    stepLow(i);
    homingPulseHigh[i] = false;
}

// =============================================================================
// GENERIC HELPERS
// =============================================================================

static inline uint32_t abs32(int32_t v) {
    return (v < 0) ? (uint32_t)(-(int64_t)v) : (uint32_t)v;
}

static inline uint8_t queueCountUnlocked(void) {
    return (uint8_t)((qHead + FRAME_QUEUE_LEN - qTail) % FRAME_QUEUE_LEN);
}

static void resetPlannedToCurrentUnlocked(void) {
    for (uint8_t i = 0; i < NUM_AXES; i++) {
        plannedTarget[i] = currentPosition[i];
    }
}

static void stopAllPulsesUnlocked(void) {
    for (uint8_t i = 0; i < NUM_AXES; i++) {
        stepLow(i);
        pulseHigh[i] = false;

        homingActive[i] = false;
        homingPulseHigh[i] = false;
    }

    homingPhase = HP_IDLE;
}

// =============================================================================
// FORWARD DECLARATIONS
// =============================================================================

static bool loadNextSegment(void);
static void homingISR(void);

// =============================================================================
// TIMER ISR
// =============================================================================

void stepperTimerISR(void) {
    if (globalMode == MODE_HOMING) {
        homingISR();
        return;
    }

    if (!segmentActive) {
        if (globalMode == MODE_RUNNING) {
            if (!loadNextSegment()) {
                globalMode = MODE_IDLE;
            }
        }
        return;
    }

    if (dirSettleTicks > 0) {
        dirSettleTicks--;
        return;
    }

    uint32_t t = segTick;

    if (t & 1) {
        // Odd tick: clear step pulses started on previous even tick.
        for (uint8_t i = 0; i < NUM_AXES; i++) {
            if (pulseHigh[i]) {
                stepLow(i);
                pulseHigh[i] = false;
            }
        }
    } else {
        // Even tick: DDA step slot.
        for (uint8_t i = 0; i < NUM_AXES; i++) {
            if (stepsDone[i] < segSteps[i]) {
                ddaAcc[i] += segSteps[i];

                if (ddaAcc[i] >= segSlots) {
                    ddaAcc[i] -= segSlots;

                    stepHigh(i);
                    pulseHigh[i] = true;
                    stepsDone[i]++;

                    currentPosition[i] += segDir[i];
                }
            }
        }
    }

    segTick++;

    if (segTick >= segTicks) {
        // Segment finished. Force pulse low just in case.
        for (uint8_t i = 0; i < NUM_AXES; i++) {
            if (pulseHigh[i]) {
                stepLow(i);
                pulseHigh[i] = false;
            }
        }

        segmentActive = false;

        if (globalMode == MODE_RUNNING) {
            if (!loadNextSegment()) {
                globalMode = MODE_IDLE;
            }
        }
    }
}

static void homingISR(void) {
    for (uint8_t i = 0; i < NUM_AXES; i++) {
        if (!homingActive[i]) continue;

        if (homingPulseHigh[i]) {
            stepLow(i);
            homingPulseHigh[i] = false;
            continue;
        }

        // Homing is low speed, simple toggle accumulator.
        homingAccum[i] += (HOMING_FREQUENCY_HZ * 2UL);

        if (homingAccum[i] >= ISR_TICK_HZ) {
            homingAccum[i] -= ISR_TICK_HZ;

            stepHigh(i);
            homingPulseHigh[i] = true;

            currentPosition[i] += homingDir[i] ? 1 : -1;
        }
    }
}

// =============================================================================
// SEGMENT LOADER
// =============================================================================

static bool loadNextSegment(void) {
    while (qTail != qHead) {
        MoveFrame f = frameQueue[qTail];
        qTail = (qTail + 1) % FRAME_QUEUE_LEN;

        uint32_t maxSteps = 0;
        bool dirChange = false;

        for (uint8_t i = 0; i < NUM_AXES; i++) {
            int32_t start = currentPosition[i];
            int32_t delta = f.target[i] - start;
            uint32_t steps = abs32(delta);

            segSteps[i] = steps;
            segDir[i] = (delta >= 0) ? 1 : -1;

            if (steps > 0) {
                if (segDir[i] != lastDir[i]) {
                    dirChange = true;
                }

                if (steps > maxSteps) {
                    maxSteps = steps;
                }
            }

            targetPosition[i] = f.target[i];
            ddaAcc[i] = 0;
            stepsDone[i] = 0;
            pulseHigh[i] = false;
        }

        // Empty move: skip it.
        if (maxSteps == 0) {
            continue;
        }

        uint64_t t64 = (uint64_t)f.duration_us * ISR_TICK_HZ / 1000000ULL;

        if (t64 < 2) t64 = 2;
        if (t64 > 0xFFFFFFFEULL) t64 = 0xFFFFFFFEULL;

        uint32_t ticks = (uint32_t)t64;

        // Segment length must be even because step pulses use high/low ticks.
        if (ticks & 1) ticks++;

        // Because one step needs high tick + low tick,
        // minimum segment length for maxSteps is maxSteps * 2 ticks.
        uint32_t minTicks = maxSteps * 2UL;

        if (ticks < minTicks) {
            ticks = minTicks;
            if (ticks & 1) ticks++;
        }

        segTicks = ticks;
        segSlots = ticks / 2UL;
        segTick = 0;
        dirSettleTicks = dirChange ? DIR_SETTLE_TICKS : 0;

        for (uint8_t i = 0; i < NUM_AXES; i++) {
            if (segSteps[i] > 0) {
                setDir(i, segDir[i] > 0);
                lastDir[i] = segDir[i];
            }
        }

        segmentActive = true;
        return true;
    }

    return false;
}

// =============================================================================
// INITIALIZATION
// =============================================================================

void DMAStepper_Init(void) {
    for (uint8_t i = 0; i < NUM_AXES; i++) {
        currentPosition[i] = 0;
        targetPosition[i] = 0;

        minPos[i] = MIN_POS_STEPS;
        maxPos[i] = MAX_POS_STEPS;

        axisHomed[i] = false;
        limitDebounce[i] = 1;
        lastDir[i] = 1;
        plannedTarget[i] = 0;

        pulseHigh[i] = false;

        homingActive[i] = false;
        homingPulseHigh[i] = false;
        homingAccum[i] = 0;
        homingDir[i] = false;
        homingSeekDone[i] = false;
    }

    allHomed = false;
    globalMode = MODE_DISABLED;
    lastError = ERR_NONE;
    lastRxMs = 0;

    qHead = 0;
    qTail = 0;

    segmentActive = false;
    segTicks = 0;
    segSlots = 0;
    segTick = 0;
    dirSettleTicks = 0;

    homingPhase = HP_IDLE;
    homingTime = 0;

    // Use TIM5. On STM32F401/F411 it is a 32-bit timer.
    sharedTimer = new HardwareTimer(TIM5);
    sharedTimer->pause();

    // If your core variant wants TIMER_CH1 instead of 1, replace 1 by TIMER_CH1.
    sharedTimer->setMode(1, TIMER_OUTPUT_COMPARE);

    // Request 100 kHz update interrupt.
    sharedTimer->setOverflow(ISR_TICK_HZ, HERTZ_FORMAT);
    sharedTimer->attachInterrupt(1, stepperTimerISR);

    // Raise interrupt priority.
    // If this block does not compile in your core variant, remove it.
#ifdef TIM5_BASE
    HAL_NVIC_SetPriority(TIM5_IRQn, 1, 0);
#endif

    sharedTimer->refresh();
    sharedTimer->resume();
}

void DMAStepper_InitAxis(uint8_t axisIdx, uint8_t stepPin, uint8_t dirPin, uint8_t limitPin) {
    if (axisIdx >= NUM_AXES) return;

    pins[axisIdx].stepPin = stepPin;
    pins[axisIdx].dirPin = dirPin;
    pins[axisIdx].limitPin = limitPin;

    pins[axisIdx].stepPort = digitalPinToPort(stepPin);
    pins[axisIdx].stepBit = digitalPinToBitMask(stepPin);

    pins[axisIdx].dirPort = digitalPinToPort(dirPin);
    pins[axisIdx].dirBit = digitalPinToBitMask(dirPin);

    pins[axisIdx].limitPort = digitalPinToPort(limitPin);
    pins[axisIdx].limitBit = digitalPinToBitMask(limitPin);

    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(limitPin, INPUT_PULLUP);

    digitalWrite(stepPin, LOW);
    digitalWrite(dirPin, LOW);

    limitDebounce[axisIdx] = 1;
}

// =============================================================================
// FRAME QUEUE
// =============================================================================

bool DMAStepper_QueueFrame(const int32_t targets[NUM_AXES], uint32_t duration_us, uint32_t seq) {
    if (targets == nullptr) return false;

    noInterrupts();

    if (!(globalMode == MODE_IDLE || globalMode == MODE_RUNNING) || !allHomed) {
        interrupts();
        return false;
    }

    // If queue is empty and no segment is active, re-sync planned position
    // to actual current position.
    if (queueCountUnlocked() == 0 && !segmentActive) {
        resetPlannedToCurrentUnlocked();
    }

    int32_t tgt[NUM_AXES];
    uint32_t maxDelta = 0;

    for (uint8_t i = 0; i < NUM_AXES; i++) {
        int32_t t = targets[i];

        if (t < minPos[i]) t = minPos[i];
        if (t > maxPos[i]) t = maxPos[i];

        tgt[i] = t;

        int32_t delta = t - plannedTarget[i];
        uint32_t ad = abs32(delta);

        if (ad > maxDelta) maxDelta = ad;
    }

    // No motion: do not consume a queue slot.
    if (maxDelta == 0) {
        for (uint8_t i = 0; i < NUM_AXES; i++) {
            targetPosition[i] = tgt[i];
        }

        interrupts();
        return true;
    }

    uint8_t nextHead = (qHead + 1) % FRAME_QUEUE_LEN;

    if (nextHead == qTail) {
        interrupts();
        return false;
    }

    uint32_t dur = duration_us;

    if (dur == 0) dur = DEFAULT_FRAME_US;

    if (dur < MIN_FRAME_US) dur = MIN_FRAME_US;
    if (dur > MAX_FRAME_US) dur = MAX_FRAME_US;

    // Speed validation.
    // If required pulse rate is above MAX_PULSE_HZ, stretch the segment.
    uint64_t minDur64 = (uint64_t)maxDelta * 1000000ULL / MAX_PULSE_HZ;

    if (minDur64 > MAX_FRAME_US) {
        interrupts();
        DMAStepper_EmergencyStop(ERR_OVERSPEED);
        return false;
    }

    if (dur < (uint32_t)minDur64) {
        dur = (uint32_t)minDur64;
    }

    MoveFrame f;

    for (uint8_t i = 0; i < NUM_AXES; i++) {
        f.target[i] = tgt[i];
        plannedTarget[i] = tgt[i];
        targetPosition[i] = tgt[i];
    }

    f.duration_us = dur;
    f.seq = seq;

    frameQueue[qHead] = f;
    qHead = nextHead;

    if (globalMode == MODE_IDLE) {
        globalMode = MODE_RUNNING;
    }

    interrupts();

    return true;
}

// =============================================================================
// MODE CONTROL
// =============================================================================

void DMAStepper_Enable(bool enable) {
    noInterrupts();

    if (enable) {
        if (globalMode != MODE_ALARM) {
            if (globalMode == MODE_DISABLED || globalMode == MODE_INIT) {
                globalMode = allHomed ? MODE_IDLE : MODE_INIT;
            }

            resetPlannedToCurrentUnlocked();
        }
    } else {
        globalMode = MODE_DISABLED;

        segmentActive = false;
        qHead = 0;
        qTail = 0;
        dirSettleTicks = 0;

        stopAllPulsesUnlocked();

        for (uint8_t i = 0; i < NUM_AXES; i++) {
            targetPosition[i] = currentPosition[i];
        }

        resetPlannedToCurrentUnlocked();
    }

    interrupts();
}

void DMAStepper_StartHoming(void) {
    noInterrupts();

    segmentActive = false;
    qHead = 0;
    qTail = 0;
    dirSettleTicks = 0;

    stopAllPulsesUnlocked();

    for (uint8_t i = 0; i < NUM_AXES; i++) {
        axisHomed[i] = false;

        limitDebounce[i] = 1;
        homingSeekDone[i] = false;

        homingActive[i] = false;
        homingPulseHigh[i] = false;
        homingAccum[i] = 0;

        targetPosition[i] = 0;
    }

    allHomed = false;
    lastError = ERR_NONE;
    homingTime = millis();

#ifdef DONT_USE_ENDSTOPS
    // Hard-stop homing without limit switches.
    // Assume all actuators are already at mechanical lower position.
    // Move away by retract distance and set zero reference after that.
    int32_t start = homingRetractStart();

    for (uint8_t i = 0; i < NUM_AXES; i++) {
        currentPosition[i] = start;
        targetPosition[i] = 0;

        startHomingAxisUnlocked(i, HOMING_RETRACT_DIRECTION);
    }

    homingPhase = HP_RETRACT;
#else
    // Normal homing with individual limit switches.
    // All axes move toward zero endstops simultaneously.
    for (uint8_t i = 0; i < NUM_AXES; i++) {
        currentPosition[i] = 0;
        targetPosition[i] = 0;

        startHomingAxisUnlocked(i, HOMING_SEEK_DIRECTION);
    }

    homingPhase = HP_SEEKING;
#endif

    globalMode = MODE_HOMING;

    interrupts();
}

void DMAStepper_SetHome(void) {
    noInterrupts();

    stopAllPulsesUnlocked();

    for (uint8_t i = 0; i < NUM_AXES; i++) {
        currentPosition[i] = 0;
        targetPosition[i] = 0;
        axisHomed[i] = true;
    }

    allHomed = true;

    if (globalMode == MODE_ALARM) {
        lastError = ERR_NONE;
    }

    globalMode = MODE_IDLE;

    resetPlannedToCurrentUnlocked();

    interrupts();
}

void DMAStepper_ClearAlarm(void) {
    noInterrupts();

    if (globalMode == MODE_ALARM) {
        lastError = ERR_NONE;

        segmentActive = false;
        qHead = 0;
        qTail = 0;
        dirSettleTicks = 0;

        stopAllPulsesUnlocked();

        for (uint8_t i = 0; i < NUM_AXES; i++) {
            targetPosition[i] = currentPosition[i];
            homingSeekDone[i] = false;
        }

        globalMode = allHomed ? MODE_IDLE : MODE_INIT;

        resetPlannedToCurrentUnlocked();
    }

    interrupts();
}

void DMAStepper_EmergencyStop(uint8_t err) {
    noInterrupts();

    globalMode = MODE_ALARM;
    lastError = err;

    segmentActive = false;
    qHead = 0;
    qTail = 0;
    dirSettleTicks = 0;

    stopAllPulsesUnlocked();

    for (uint8_t i = 0; i < NUM_AXES; i++) {
        targetPosition[i] = currentPosition[i];
    }

    resetPlannedToCurrentUnlocked();

    interrupts();
}

void DMAStepper_NotifyValidPacket(void) {
    lastRxMs = millis();
}

// =============================================================================
// STATE GETTERS
// =============================================================================

void DMAStepper_GetPositions(int32_t pos[NUM_AXES]) {
    noInterrupts();

    for (uint8_t i = 0; i < NUM_AXES; i++) {
        pos[i] = currentPosition[i];
    }

    interrupts();
}

void DMAStepper_GetTargets(int32_t tgt[NUM_AXES]) {
    noInterrupts();

    for (uint8_t i = 0; i < NUM_AXES; i++) {
        tgt[i] = targetPosition[i];
    }

    interrupts();
}

uint8_t DMAStepper_GetMode(void) {
    return globalMode;
}

uint8_t DMAStepper_GetError(void) {
    return lastError;
}

uint8_t DMAStepper_GetQueueCount(void) {
    noInterrupts();

    uint8_t n = queueCountUnlocked();

    interrupts();

    return n;
}

bool DMAStepper_IsHomed(void) {
    return allHomed;
}

bool DMAStepper_IsSegmentActive(void) {
    return segmentActive;
}

// =============================================================================
// LIMIT SWITCH PROCESSING
// =============================================================================

static bool updateLimit(uint8_t i) {
    bool raw = readLimitRaw(i);

#if LIMIT_ACTIVE_HIGH
    bool active = raw;
#else
    bool active = !raw;
#endif

    if (active) {
        if (limitDebounce[i] < 32768) {
            limitDebounce[i] <<= 1;
        }
    } else {
        if (limitDebounce[i] > 1) {
            limitDebounce[i] >>= 1;
        }
    }

    return (limitDebounce[i] == 32768);
}

// =============================================================================
// MAIN BACKGROUND PROCESS
// =============================================================================

void DMAStepper_Process(void) {
    uint32_t now = millis();

    for (uint8_t i = 0; i < NUM_AXES; i++) {
        limitConfirmed[i] = updateLimit(i);
    }

    // =========================================================================
    // SYNCHRONIZED HOMING
    // =========================================================================
    if (globalMode == MODE_HOMING) {
        if (homingPhase == HP_SEEKING) {
            if (now - homingTime > HOMING_SEEK_TIMEOUT_MS) {
                DMAStepper_EmergencyStop(ERR_HOMING_TIMEOUT);
                return;
            }

            bool allDone = true;

            for (uint8_t i = 0; i < NUM_AXES; i++) {
                if (homingSeekDone[i]) {
                    continue;
                }

                bool limitHit = false;

#ifndef DONT_USE_ENDSTOPS
                limitHit = limitConfirmed[i];
#endif

                bool overflow = homingSeekOverflow(currentPosition[i]);

                if (overflow) {
                    DMAStepper_EmergencyStop(ERR_HOMING_OVERFLOW);
                    return;
                }

                if (limitHit) {
                    noInterrupts();

                    homingSeekDone[i] = true;
                    stopHomingAxisUnlocked(i);

                    interrupts();
                } else {
                    allDone = false;

                    if (!homingActive[i]) {
                        noInterrupts();
                        startHomingAxisUnlocked(i, HOMING_SEEK_DIRECTION);
                        interrupts();
                    }
                }
            }

            if (allDone) {
                noInterrupts();

                int32_t start = homingRetractStart();

                for (uint8_t i = 0; i < NUM_AXES; i++) {
                    currentPosition[i] = start;
                    targetPosition[i] = 0;

                    homingSeekDone[i] = false;

                    startHomingAxisUnlocked(i, HOMING_RETRACT_DIRECTION);
                }

                homingPhase = HP_RETRACT;
                homingTime = now;

                interrupts();
            }
        } else if (homingPhase == HP_RETRACT) {
            if (now - homingTime > HOMING_RETRACT_TIMEOUT_MS) {
                DMAStepper_EmergencyStop(ERR_HOMING_TIMEOUT);
                return;
            }

            bool allDone = true;

            for (uint8_t i = 0; i < NUM_AXES; i++) {
                if (homingRetractReached(currentPosition[i])) {
                    if (homingActive[i]) {
                        noInterrupts();
                        stopHomingAxisUnlocked(i);
                        interrupts();
                    }

                    currentPosition[i] = 0;
                    targetPosition[i] = 0;
                } else {
                    allDone = false;

                    if (!homingActive[i]) {
                        noInterrupts();
                        startHomingAxisUnlocked(i, HOMING_RETRACT_DIRECTION);
                        interrupts();
                    }
                }
            }

            if (allDone) {
                noInterrupts();

                for (uint8_t i = 0; i < NUM_AXES; i++) {
                    stopHomingAxisUnlocked(i);

                    currentPosition[i] = 0;
                    targetPosition[i] = 0;

                    axisHomed[i] = true;
                }

                allHomed = true;
                globalMode = MODE_IDLE;
                homingPhase = HP_IDLE;

                resetPlannedToCurrentUnlocked();

                interrupts();
            }
        } else {
            // MODE_HOMING but no active homing phase.
            noInterrupts();
            globalMode = allHomed ? MODE_IDLE : MODE_INIT;
            interrupts();
        }

        return;
    }

    // =========================================================================
    // NORMAL MODE SAFETY
    // =========================================================================

#ifndef DONT_USE_ENDSTOPS
    // If homed and not homing, any confirmed limit switch is an emergency.
    if (allHomed) {
        for (uint8_t i = 0; i < NUM_AXES; i++) {
            if (limitConfirmed[i]) {
                DMAStepper_EmergencyStop(ERR_LIMIT);
                return;
            }
        }
    }
#endif

    // Communication watchdog.
    if (globalMode == MODE_RUNNING && lastRxMs != 0) {
        if (now - lastRxMs > COMM_TIMEOUT_MS) {
            DMAStepper_EmergencyStop(ERR_COMM_TIMEOUT);
            return;
        }
    }
}