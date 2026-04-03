// 3DOF by Andrey Zhuravlev
// v.azhure@gmail.com
// Discord: https://discord.gg/ynHCkrsmMA

#include "dma_stepper_hal.h"
#include <HardwareTimer.h>
#include <libmaple/gpio.h>
#include <string.h>

// =============================================================================
// ВНУТРЕННИЕ ПЕРЕМЕННЫЕ
// =============================================================================
static AxisConfig axisConfig[NUM_AXES];
static AxisState axisState[NUM_AXES];
static HardwareTimer* sharedTimer = nullptr;

static const gpio_dev* stepPorts[NUM_AXES];
static uint8_t stepBits[NUM_AXES];
static const gpio_dev* dirPorts[NUM_AXES];
static uint8_t dirBits[NUM_AXES];

static volatile bool stepState[NUM_AXES] = {false};
static volatile uint32_t axisAccum[NUM_AXES] = {0};

// Состояния для HOME/PARK (сохранённая скорость УБРАНА)
static HomingSubState axisHomeState[NUM_AXES] = {H_IDLE};
static uint32_t axisHomeTime[NUM_AXES] = {0};

#define ISR_BASE_FREQ 100000UL // 100 кГц → поддержка до 500 мм/с

// =============================================================================
// FAST GPIO & ISR
// =============================================================================
inline void stepPinHigh(uint8_t i) { stepPorts[i]->regs->BSRR = (1 << stepBits[i]); }
inline void stepPinLow(uint8_t i)  { stepPorts[i]->regs->BRR = (1 << stepBits[i]); }
inline void setDirection(uint8_t i, bool fwd) {
    if(fwd) dirPorts[i]->regs->BSRR = (1 << dirBits[i]);
    else    dirPorts[i]->regs->BRR = (1 << dirBits[i]);
}

void sharedTimerISR(void) {
    for(int i = 0; i < NUM_AXES; i++) {
        if(!axisState[i].stepping) continue;
        axisAccum[i] += axisState[i].frequency;
        if(axisAccum[i] >= ISR_BASE_FREQ) {
            axisAccum[i] -= ISR_BASE_FREQ;
            if(stepState[i]) {
                stepPinLow(i); stepState[i] = false;
                axisState[i].currentPosition += (axisState[i].direction ? 1 : -1);
            } else {
                stepPinHigh(i); stepState[i] = true;
            }
        }
    }
}

// =============================================================================
// ИНИЦИАЛИЗАЦИЯ
// =============================================================================
void DMAStepper_Init(void) {
    memset(axisConfig, 0, sizeof(axisConfig));
    memset(axisState, 0, sizeof(axisState));
    for(int i = 0; i < NUM_AXES; i++) axisAccum[i] = 0;
    
    sharedTimer = new HardwareTimer(2);
    sharedTimer->pause();
    sharedTimer->setPrescaleFactor(1);
    sharedTimer->setOverflow(719); // 72MHz / 100kHz = 720 → 719
    sharedTimer->attachInterrupt(0, sharedTimerISR);
    sharedTimer->refresh();
    sharedTimer->resume();
    
    for(int i=0; i<NUM_AXES; i++) {
        axisState[i].currentPosition = 0;
        axisState[i].targetPosition = 0;
        axisState[i].minPos = (int32_t)(STEPS_PER_REV * 0.25);
        axisState[i].maxPos = (int32_t)(MAX_REVOLUTIONS * STEPS_PER_REV) - axisState[i].minPos;
        axisState[i].frequency = DEFAULT_FREQUENCY_HZ;
        axisState[i].stepping = false;
        axisState[i].homed = false;
        axisState[i].mode = MODE_UNKNOWN;
        axisState[i].maxFreqHz = DEFAULT_FREQUENCY_HZ;
        axisState[i].maxSpeedMM = DEFAULT_SPEED_MM_SEC;
        axisState[i].maxAccel = MAX_ACCEL;
        axisState[i].limitedFreq = 0;
        axisState[i].Kp = 15.0f; axisState[i].Ki = 0.0f; axisState[i].Kd = 0.02f; axisState[i].Ks = 0.30f;
        axisState[i].pidMinFreq = (float)MIN_FREQUENCY_HZ; axisState[i].pidMaxFreq = (float)SAFE_FREQUENCY_HZ;
        axisState[i].pidEnabled = false; axisState[i].pidBlend = 0.35f;
        stepState[i] = false;
    }
}

void DMAStepper_InitAxis(uint8_t axisIdx, uint8_t stepPin, uint8_t dirPin, uint8_t limitPin) {
    if (axisIdx >= NUM_AXES) return;
    axisConfig[axisIdx].stepPin = stepPin;
    axisConfig[axisIdx].dirPin = dirPin;
    axisConfig[axisIdx].limitPin = limitPin;

    stepPorts[axisIdx] = PIN_MAP[stepPin].gpio_device;
    stepBits[axisIdx]  = PIN_MAP[stepPin].gpio_bit;
    dirPorts[axisIdx]  = PIN_MAP[dirPin].gpio_device;
    dirBits[axisIdx]   = PIN_MAP[dirPin].gpio_bit;

    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(limitPin, INPUT_PULLUP);
    digitalWrite(stepPin, LOW);
    digitalWrite(dirPin, LOW);
    axisState[axisIdx].mode = MODE_CONNECTED;
}

// =============================================================================
// УПРАВЛЕНИЕ
// =============================================================================
void DMAStepper_SetFrequency(uint8_t axisIdx, uint32_t freqHz) {
    if (axisIdx >= NUM_AXES) return;
    
    // Ограничиваем ПОЛЬЗОВАТЕЛЬСКУЮ частоту (полные шаги/с)
    // Реальная частота переключений будет ×2
    freqHz = constrain(freqHz, MIN_FREQUENCY_HZ, MAX_FREQUENCY_HZ / 2);  // <-- Делим MAX на 2
    
    axisState[axisIdx].frequency = freqHz * 2;
}

void DMAStepper_StartAxis(uint8_t axisIdx, bool forward) {
    if (axisIdx >= NUM_AXES) return;
    setDirection(axisIdx, forward);
    axisState[axisIdx].direction = forward;
    stepState[axisIdx] = false; stepPinLow(axisIdx);
    axisAccum[axisIdx] = 0;
    axisState[axisIdx].stepping = true;
}
void DMAStepper_StopAxis(uint8_t axisIdx) {
    if (axisIdx >= NUM_AXES) return;
    axisState[axisIdx].stepping = false;
    stepPinLow(axisIdx); stepState[axisIdx] = false;
    axisState[axisIdx].limitedFreq = 0;
}
void DMAStepper_StopAll(void) { for(int i=0;i<NUM_AXES;i++) DMAStepper_StopAxis(i); }

void DMAStepper_SetTarget(uint8_t axisIdx, int32_t target) {
    if (axisIdx >= NUM_AXES) return;
    AxisState* ax = &axisState[axisIdx];
    ax->targetPosition = constrain(target, ax->minPos, ax->maxPos);
}
int32_t DMAStepper_GetPosition(uint8_t axisIdx) { return (axisIdx < NUM_AXES) ? axisState[axisIdx].currentPosition : 0; }
void DMAStepper_SetPosition(uint8_t axisIdx, int32_t pos) { if (axisIdx < NUM_AXES) axisState[axisIdx].currentPosition = pos; }
AxisState* DMAStepper_GetAxis(uint8_t axisIdx) { return (axisIdx < NUM_AXES) ? &axisState[axisIdx] : nullptr; }

void DMAStepper_SetMaxSpeed(uint8_t axisIdx, uint32_t speedMM) {
    if (axisIdx >= NUM_AXES) return;
    speedMM = constrain(speedMM, (uint32_t)10, SAFE_SPEED_MM_SEC);
    axisState[axisIdx].maxSpeedMM = speedMM;
    axisState[axisIdx].maxFreqHz = (uint32_t)((float)speedMM / MM_PER_STEP);
    axisState[axisIdx].pidMaxFreq = (float)axisState[axisIdx].maxFreqHz;
}
bool DMAStepper_CheckLimit(uint8_t axisIdx) {
    if (axisIdx >= NUM_AXES) return false;
    return (digitalRead(axisConfig[axisIdx].limitPin) == LOW);
}

// =============================================================================
// PID & ACCEL
// =============================================================================
void DMAStepper_SetPID(uint8_t axisIdx, float Kp, float Ki, float Kd, float Ks) {
    if (axisIdx >= NUM_AXES) return;
    axisState[axisIdx].Kp = constrain(Kp, 0.0f, 200.0f);
    axisState[axisIdx].Ki = constrain(Ki, 0.0f, 50.0f);
    axisState[axisIdx].Kd = constrain(Kd, 0.0f, 50.0f);
    axisState[axisIdx].Ks = constrain(Ks, 0.0f, 1.0f);
}
void DMAStepper_SetPIDEnable(uint8_t axisIdx, bool enable) {
    if (axisIdx >= NUM_AXES) return;
    axisState[axisIdx].pidEnabled = enable;
    axisState[axisIdx].integral = 0;
    axisState[axisIdx].prevError = (float)(axisState[axisIdx].targetPosition - axisState[axisIdx].currentPosition);
    axisState[axisIdx].derivativeFilter = 0;
}
void DMAStepper_SetPIDBlend(uint8_t axisIdx, float blend) {
    if (axisIdx >= NUM_AXES) return;
    axisState[axisIdx].pidBlend = constrain(blend, 0.0f, 1.0f);
}

static float computePID(AxisState* ax, uint32_t currentTime) {
    float dt = (float)(currentTime - ax->accelLastTime) / 1000.0f;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.001f;
    ax->accelLastTime = currentTime;
    float error = (float)(ax->targetPosition - ax->currentPosition);
    float proportional = ax->Kp * error;
    if (fabs(error) < 800.0f) {
        ax->integral += error * dt;
        ax->integral = constrain(ax->integral, -400.0f, 400.0f);
    } else { ax->integral *= 0.9f; }
    float integralTerm = ax->Ki * ax->integral;
    float derivative = (error - ax->prevError) / dt;
    ax->derivativeFilter = ax->Ks * ax->derivativeFilter + (1.0f - ax->Ks) * derivative;
    ax->prevError = error;
    float output = proportional + integralTerm + ax->Kd * ax->derivativeFilter;
    float freq = fabs(output);
    if (freq < ax->pidMinFreq && fabs(error) < 2.0f) return 0.0f;
    return constrain(freq, ax->pidMinFreq, ax->pidMaxFreq);
}

static uint32_t applyAccelLimit(AxisState* ax, uint32_t desiredFreq) {
    uint32_t now = millis();
    uint32_t dt = now - ax->accelLastTime;
    if (dt == 0) dt = 1;
    ax->accelLastTime = now;
    float maxDelta = (float)ax->maxAccel * dt / 1000.0f;
    float delta = (float)desiredFreq - ax->limitedFreq;
    if (fabs(delta) > maxDelta) ax->limitedFreq += (delta > 0) ? maxDelta : -maxDelta;
    else ax->limitedFreq = (float)desiredFreq;
    return (uint32_t)constrain((int32_t)ax->limitedFreq, MIN_FREQUENCY_HZ, (int32_t)ax->maxFreqHz);
}

// =============================================================================
// CLEAR ALARM & MAIN PROCESS LOOP (БЕЗ axisSavedSpeedMM)
// =============================================================================
void DMAStepper_ClearAlarm(void) {
    DMAStepper_StopAll();
    for(int i=0; i<NUM_AXES; i++) {
        axisHomeState[i] = H_IDLE;
        AxisState* ax = &axisState[i];
        ax->mode = ax->homed ? MODE_READY : MODE_CONNECTED;
        ax->limitedFreq = 0;
        ax->accelLastTime = millis();
    }
}

void DMAStepper_Process(void) {
    uint32_t now = millis();
    int32_t maxRange = (int32_t)(MAX_REVOLUTIONS * STEPS_PER_REV);

    for(int i=0; i<NUM_AXES; i++) {
        AxisState* ax = &axisState[i];

        switch(ax->mode) {
            // =====================================================================
            // РЕЖИМ КАЛИБРОВКИ (жёсткая скорость HOMING_FREQUENCY_HZ)
            // =====================================================================
            case MODE_HOMING:
                // Принудительно ставим скорость калибровки каждый цикл
                DMAStepper_SetFrequency(i, HOMING_FREQUENCY_HZ);
                
                if(axisHomeState[i] == H_IDLE) {
                    DMAStepper_SetPosition(i, 0);
                    DMAStepper_SetTarget(i, (int32_t)(maxRange * HOMING_TRAVEL_LIMIT_MULT));
                    DMAStepper_StartAxis(i, true);
                    axisHomeState[i] = H_SEEKING;
                    axisHomeTime[i] = now;
                }

                switch(axisHomeState[i]) {
                    case H_SEEKING:
                        if(DMAStepper_CheckLimit(i) || now - axisHomeTime[i] > HOMING_SEEK_TIMEOUT_MS || ax->currentPosition > (int32_t)(maxRange * HOMING_OVERFLOW_LIMIT_MULT)) {
                            DMAStepper_StopAxis(i);
                            if(DMAStepper_CheckLimit(i)) {
                                DMAStepper_SetPosition(i, ax->maxPos); // Фиксируем физический конец
                                axisHomeState[i] = H_RETRACT;
                                axisHomeTime[i] = now;
                            } else {
                                ax->mode = MODE_ALARM; ax->homed = false;
                                axisHomeState[i] = H_IDLE;
                            }
                        } else if(!ax->stepping) {
                            DMAStepper_StartAxis(i, true);
                        }
                        break;
                    case H_RETRACT:
                        if(now - axisHomeTime[i] > HOMING_RETRACT_DURATION_MS) {
                            DMAStepper_StopAxis(i);
                            DMAStepper_SetTarget(i, (ax->minPos + ax->maxPos) / 2);
                            DMAStepper_StartAxis(i, false);
                            axisHomeState[i] = H_MOVING_CENTER;
                            axisHomeTime[i] = now;
                        } else if(!ax->stepping) {
                            DMAStepper_StartAxis(i, false);
                        }
                        break;
                    case H_MOVING_CENTER:
                        if(abs(ax->currentPosition - ax->targetPosition) <= HOMING_CENTER_TOLERANCE) {
                            DMAStepper_StopAxis(i);
                            ax->mode = MODE_READY; ax->homed = true;
                            axisHomeState[i] = H_DONE;
                            // ✅ Сброс лимитера для корректного старта в READY
                            ax->limitedFreq = 0;
                            ax->accelLastTime = millis();
                        } else if(now - axisHomeTime[i] > HOMING_CENTER_TIMEOUT_MS) {
                            ax->mode = MODE_ALARM; ax->homed = false;
                            DMAStepper_StopAxis(i);
                            axisHomeState[i] = H_IDLE;
                        } else if(!ax->stepping) {
                            DMAStepper_StartAxis(i, false);
                        }
                        break;
                    default: break;
                }
                break;

            // =====================================================================
            // РЕЖИМ ПАРКОВКИ (жёсткая скорость PARKING_FREQUENCY_HZ)
            // =====================================================================
            case MODE_PARKING:
                DMAStepper_SetFrequency(i, PARKING_FREQUENCY_HZ);
                DMAStepper_SetTarget(i, ax->minPos);
                
                if(abs(ax->currentPosition - ax->targetPosition) <= POSITION_TOLERANCE) {
                    DMAStepper_StopAxis(i);
                    ax->mode = MODE_READY;
                    ax->limitedFreq = 0;
                    ax->accelLastTime = millis();
                } else if(!ax->stepping) {
                    DMAStepper_StartAxis(i, ax->targetPosition > ax->currentPosition);
                }
                break;

            // =====================================================================
            // РАБОЧИЙ РЕЖИМ (использует ax->maxFreqHz из CMD_SET_SPEED)
            // =====================================================================
            case MODE_READY:
                {
                    int32_t error = ax->targetPosition - ax->currentPosition;
                    if(abs(error) > POSITION_DEADZONE) {
                        uint32_t desiredFreq;
                        bool forward = (error > 0);
                        
                        if(ax->pidEnabled) desiredFreq = (uint32_t)computePID(ax, now);
                        else {
                            int32_t dist = abs(error);
                            desiredFreq = (dist < ACCEL_RAMP_DISTANCE) ? map(dist, 0, ACCEL_RAMP_DISTANCE, MIN_FREQUENCY_HZ, ax->maxFreqHz) : ax->maxFreqHz;
                        }
                        
                        // Лимитер автоматически ограничит до ax->maxFreqHz
                        DMAStepper_SetFrequency(i, applyAccelLimit(ax, desiredFreq));
                        
                        if(!ax->stepping) {
                            if(ax->pidEnabled) { ax->integral=0; ax->prevError=(float)error; ax->derivativeFilter=0; }
                            DMAStepper_StartAxis(i, forward);
                        } else if(ax->direction != forward) {
                            DMAStepper_StopAxis(i); delayMicroseconds(DIRECTION_CHANGE_DELAY_US); ax->limitedFreq=0;
                            if(ax->pidEnabled) { ax->integral=0; ax->prevError=(float)error; ax->derivativeFilter=0; }
                            DMAStepper_StartAxis(i, forward);
                        }
                    } else {
                        if(ax->stepping) { DMAStepper_StopAxis(i); if(ax->pidEnabled) ax->integral *= 0.95f; }
                    }
                }
                break;

            // =====================================================================
            // ОСТАЛЬНЫЕ РЕЖИМЫ (остановка)
            // =====================================================================
            case MODE_CONNECTED:
            case MODE_DISABLED:
            case MODE_ALARM:
            case MODE_UNKNOWN:
                DMAStepper_StopAxis(i);
                break;
        }
    }
}
