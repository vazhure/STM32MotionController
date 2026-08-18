// 3DOF by Andrey Zhuravlev
// v.azhure@gmail.com
// Discord: https://discord.gg/ynHCkrsmMA
// ============================================================================
// CHANGELOG: See dma_stepper_hal.h
// ============================================================================

#include "dma_stepper_hal.h"
#include <HardwareTimer.h>
#include <libmaple/gpio.h>
#include <string.h>

#if (CONTROLLER_MODE == CONTROLLER_MODE_MASTER || CONTROLLER_MODE == CONTROLLER_MODE_SLAVE)
  #include <SPI.h>
#endif

// =============================================================================
// INTERNAL VARIABLES
// =============================================================================
static AxisConfig axisConfig[NUM_AXES];
static AxisState  axisState[NUM_AXES];
static HardwareTimer* sharedTimer = nullptr;

static const gpio_dev* stepPorts[NUM_AXES];
static uint8_t stepBits[NUM_AXES];
static const gpio_dev* dirPorts[NUM_AXES];
static uint8_t dirBits[NUM_AXES];

static volatile bool     stepState[NUM_AXES] = { false };
static volatile uint32_t axisAccum[NUM_AXES] = { 0 };

static HomingSubState axisHomeState[NUM_AXES] = { H_IDLE };
static uint32_t       axisHomeTime[NUM_AXES]  = { 0 };
static uint16_t       limitDebounce[NUM_AXES] = { 1, 1, 1 };

#define ISR_BASE_FREQ 100000UL

// =============================================================================
// SPI MULTI-CONTROLLER VARIABLES
// =============================================================================
#if (CONTROLLER_MODE == CONTROLLER_MODE_MASTER || CONTROLLER_MODE == CONTROLLER_MODE_SLAVE)
static SPI_FRAME spiTxBuf;
static SPI_FRAME spiRxBuf;
static volatile bool spiFrameReady = false;

#if (CONTROLLER_MODE == CONTROLLER_MODE_SLAVE)
static uint8_t lastFrameId = 0;
#endif

static uint32_t lastSyncTime = 0;
#endif

// =============================================================================
// FAST GPIO & ISR
// =============================================================================
inline bool fastReadPin(uint8_t pin) {
  return gpio_read_bit(PIN_MAP[pin].gpio_device, PIN_MAP[pin].gpio_bit);
}

inline void stepPinHigh(uint8_t i) { stepPorts[i]->regs->BSRR = (1 << stepBits[i]); }
inline void stepPinLow(uint8_t i)  { stepPorts[i]->regs->BRR  = (1 << stepBits[i]); }

inline void setDirection(uint8_t i, bool fwd) {
  if (fwd) dirPorts[i]->regs->BSRR = (1 << dirBits[i]);
  else     dirPorts[i]->regs->BRR  = (1 << dirBits[i]);
}

void sharedTimerISR(void) {
  for (int i = 0; i < NUM_AXES; i++) {
    if (!axisState[i].stepping) continue;
    axisAccum[i] += axisState[i].frequency;
    if (axisAccum[i] >= ISR_BASE_FREQ) {
      axisAccum[i] -= ISR_BASE_FREQ;
      if (stepState[i]) {
        stepPinLow(i);
        stepState[i] = false;
        axisState[i].currentPosition += (axisState[i].direction ? 1 : -1);
      } else {
        stepPinHigh(i);
        stepState[i] = true;
      }
    }
  }
}

// =============================================================================
// INITIALIZATION
// =============================================================================
void DMAStepper_Init(void) {
  memset(axisConfig, 0, sizeof(axisConfig));
  memset(axisState, 0, sizeof(axisState));
  for (int i = 0; i < NUM_AXES; i++) axisAccum[i] = 0;

  sharedTimer = new HardwareTimer(2);
  sharedTimer->pause();
  sharedTimer->setPrescaleFactor(1);
  sharedTimer->setOverflow(719);
  sharedTimer->attachInterrupt(0, sharedTimerISR);
  sharedTimer->refresh();
  sharedTimer->resume();

  uint32_t now = millis();
  int32_t fullRange = (int32_t)(MAX_REVOLUTIONS * STEPS_PER_REV);

  for (int i = 0; i < NUM_AXES; i++) {
    axisState[i].currentPosition = 0;
    axisState[i].targetPosition  = 0;
    // 2026-08-18: Working range inset by END_CLEARANCE_STEPS from both ends
    axisState[i].minPos = END_CLEARANCE_STEPS;
    axisState[i].maxPos = fullRange - END_CLEARANCE_STEPS;

    axisState[i].frequency   = DEFAULT_FREQUENCY_HZ;
    axisState[i].stepping    = false;
    axisState[i].homed       = false;
    axisState[i].mode        = MODE_UNKNOWN;
    axisState[i].maxFreqHz   = DEFAULT_FREQUENCY_HZ;
    axisState[i].maxSpeedMM  = DEFAULT_SPEED_MM_SEC;
    axisState[i].maxAccel    = MAX_ACCEL;
    axisState[i].limitedFreq = 0;
    axisState[i].accelLastTime = now;
    axisState[i].pidLastTime   = now;
    axisState[i].Kp = 15.0f; axisState[i].Ki = 0.0f;
    axisState[i].Kd = 0.02f; axisState[i].Ks = 0.30f;
    axisState[i].pidMinFreq = (float)MIN_FREQUENCY_HZ;
    axisState[i].pidMaxFreq = (float)SAFE_FREQUENCY_HZ;
    axisState[i].pidEnabled = false;
    axisState[i].pidBlend   = 0.35f;
    stepState[i] = false;
    limitDebounce[i] = 1;
    axisState[i].pendingTarget = PENDING_TARGET_NONE;
  }
}

void DMAStepper_InitAxis(uint8_t axisIdx, uint8_t stepPin, uint8_t dirPin, uint8_t limitPin) {
  if (axisIdx >= NUM_AXES) return;
  axisConfig[axisIdx].stepPin  = stepPin;
  axisConfig[axisIdx].dirPin   = dirPin;
  axisConfig[axisIdx].limitPin = limitPin;
  stepPorts[axisIdx] = PIN_MAP[stepPin].gpio_device;
  stepBits[axisIdx]  = PIN_MAP[stepPin].gpio_bit;
  dirPorts[axisIdx]  = PIN_MAP[dirPin].gpio_device;
  dirBits[axisIdx]   = PIN_MAP[dirPin].gpio_bit;
  
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  
  // 2026-08-18: Pull mode resolved at compile time for optimal EMI immunity
#if (_RESOLVED_PULL == LIMIT_PULL_UP)
  pinMode(limitPin, INPUT_PULLUP);
#else
  pinMode(limitPin, INPUT_PULLDOWN);
#endif

  digitalWrite(stepPin, LOW);
  digitalWrite(dirPin, LOW);
  limitDebounce[axisIdx] = 1;
  axisState[axisIdx].mode = MODE_CONNECTED;
}

// =============================================================================
// CONTROL FUNCTIONS
// =============================================================================
void DMAStepper_SetFrequency(uint8_t axisIdx, uint32_t freqHz) {
  if (axisIdx >= NUM_AXES) return;
  freqHz = constrain(freqHz, MIN_FREQUENCY_HZ, MAX_FREQUENCY_HZ / 2);
  axisState[axisIdx].frequency = freqHz * 2;
}

void DMAStepper_StartAxis(uint8_t axisIdx, bool forward) {
  if (axisIdx >= NUM_AXES) return;
  setDirection(axisIdx, forward);
  axisState[axisIdx].direction = forward;
  stepState[axisIdx] = false;
  stepPinLow(axisIdx);
  axisAccum[axisIdx] = 0;
  axisState[axisIdx].stepping = true;
}

void DMAStepper_StopAxis(uint8_t axisIdx) {
  if (axisIdx >= NUM_AXES) return;
  axisState[axisIdx].stepping = false;
  stepPinLow(axisIdx);
  stepState[axisIdx] = false;
  axisState[axisIdx].limitedFreq = 0;
}

void DMAStepper_StopAll(void) {
  for (int i = 0; i < NUM_AXES; i++) DMAStepper_StopAxis(i);
}

void DMAStepper_SetTarget(uint8_t axisIdx, int32_t target) {
  if (axisIdx >= NUM_AXES) return;
  AxisState* ax = &axisState[axisIdx];
  ax->targetPosition = constrain(target, ax->minPos, ax->maxPos);
}

int32_t DMAStepper_GetPosition(uint8_t axisIdx) {
  return (axisIdx < NUM_AXES) ? axisState[axisIdx].currentPosition : 0;
}

void DMAStepper_SetPosition(uint8_t axisIdx, int32_t pos) {
  if (axisIdx < NUM_AXES) axisState[axisIdx].currentPosition = pos;
}

AxisState* DMAStepper_GetAxis(uint8_t axisIdx) {
  return (axisIdx < NUM_AXES) ? &axisState[axisIdx] : nullptr;
}

void DMAStepper_SetMaxSpeed(uint8_t axisIdx, uint32_t speedMM) {
  if (axisIdx >= NUM_AXES) return;
  speedMM = constrain(speedMM, (uint32_t)10, SAFE_SPEED_MM_SEC);
  axisState[axisIdx].maxSpeedMM = speedMM;
  axisState[axisIdx].maxFreqHz  = (uint32_t)((float)speedMM / MM_PER_STEP);
  axisState[axisIdx].pidMaxFreq = (float)axisState[axisIdx].maxFreqHz;
}

void DMAStepper_StartHoming(uint8_t axisIdx) {
  if (axisIdx >= NUM_AXES) return;
  AxisState* ax = &axisState[axisIdx];
  if (ax->mode == MODE_DISABLED) return;
  DMAStepper_StopAxis(axisIdx);
  ax->pendingTarget = PENDING_TARGET_NONE;
  ax->limitedFreq   = 0;
  axisHomeState[axisIdx] = H_IDLE;
  limitDebounce[axisIdx] = 1;
  ax->mode = MODE_HOMING;
}

// =============================================================================
// LIMIT SWITCH CHECK WITH DEBOUNCE (NC/NO aware)
// =============================================================================
bool DMAStepper_CheckLimit(uint8_t axisIdx) {
  if (axisIdx >= NUM_AXES) return false;
  bool rawState = fastReadPin(axisConfig[axisIdx].limitPin);

  // 2026-08-18: Normalize to "triggered" semantics
#if LIMIT_ACTIVE_HIGH
  bool triggeredRaw = rawState;
#else
  bool triggeredRaw = !rawState;
#endif

  if (triggeredRaw) {
    if (limitDebounce[axisIdx] < 32768) limitDebounce[axisIdx] <<= 1;
  } else {
    if (limitDebounce[axisIdx] > 1) limitDebounce[axisIdx] >>= 1;
  }
  return (limitDebounce[axisIdx] == 32768);
}

// =============================================================================
// PID & ACCELERATION CONTROL
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
  axisState[axisIdx].integral   = 0;
  axisState[axisIdx].prevError  = (float)(axisState[axisIdx].targetPosition - axisState[axisIdx].currentPosition);
  axisState[axisIdx].derivativeFilter = 0;
  axisState[axisIdx].pidLastTime = millis();
}

void DMAStepper_SetPIDBlend(uint8_t axisIdx, float blend) {
  if (axisIdx >= NUM_AXES) return;
  axisState[axisIdx].pidBlend = constrain(blend, 0.0f, 1.0f);
}

static float computePID(AxisState* ax, uint32_t currentTime) {
  float dt = (float)(currentTime - ax->pidLastTime) / 1000.0f;
  if (dt <= 0.0f || dt > 0.5f) dt = 0.001f;
  ax->pidLastTime = currentTime;

  float error = (float)(ax->targetPosition - ax->currentPosition);
  float proportional = ax->Kp * error;

  if (fabs(error) < 800.0f) {
    ax->integral += error * dt;
    ax->integral = constrain(ax->integral, -400.0f, 400.0f);
  } else {
    ax->integral *= 0.9f;
  }
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
  uint32_t dt  = now - ax->accelLastTime;
  if (dt == 0) dt = 1;
  ax->accelLastTime = now;

  float maxDelta = (float)ax->maxAccel * dt / 1000.0f;
  float delta    = (float)desiredFreq - ax->limitedFreq;
  if (fabs(delta) > maxDelta) ax->limitedFreq += (delta > 0) ? maxDelta : -maxDelta;
  else ax->limitedFreq = (float)desiredFreq;

  return (uint32_t)constrain((int32_t)ax->limitedFreq, MIN_FREQUENCY_HZ, (int32_t)ax->maxFreqHz);
}

// =============================================================================
// CLEAR ALARM
// =============================================================================
void DMAStepper_ClearAlarm(void) {
  DMAStepper_StopAll();
  for (int i = 0; i < NUM_AXES; i++) {
    axisHomeState[i] = H_IDLE;
    limitDebounce[i] = 1;
    AxisState* ax = &axisState[i];
    if (ax->mode == MODE_ALARM) {
      ax->mode = ax->homed ? MODE_READY : MODE_CONNECTED;
      ax->limitedFreq   = 0;
      ax->accelLastTime = millis();
      ax->pidLastTime   = millis();
    }
  }
#if (CONTROLLER_MODE == CONTROLLER_MODE_SLAVE)
  digitalWrite(ALARM_PIN, HIGH);
#endif
}

// =============================================================================
// MAIN PROCESS LOOP
// =============================================================================
void DMAStepper_Process(void) {
  uint32_t now = millis();
  int32_t maxRange = (int32_t)(MAX_REVOLUTIONS * STEPS_PER_REV);

  // 2026-08-18: Pre-compute homing direction flags
  const bool homeForward    = (HOME_DIRECTION > 0);
  const bool retractForward = (HOME_DIRECTION < 0);

  for (int i = 0; i < NUM_AXES; i++) {
    AxisState* ax = &axisState[i];

    switch (ax->mode) {
      case MODE_HOMING:
        if (axisHomeState[i] == H_IDLE) {
          DMAStepper_SetFrequency(i, HOMING_FREQUENCY_HZ);
          DMAStepper_SetPosition(i, homeForward ? 0 : maxRange);
          ax->targetPosition = homeForward
              ? (int32_t)(maxRange * HOMING_TRAVEL_LIMIT_MULT)
              : (int32_t)(maxRange * (1.0f - HOMING_TRAVEL_LIMIT_MULT));
          DMAStepper_StartAxis(i, homeForward);
          axisHomeState[i] = H_SEEKING;
          axisHomeTime[i]  = now;
        }
        switch (axisHomeState[i]) {
          case H_SEEKING:
            {
              bool limitHit = DMAStepper_CheckLimit(i);
              bool timeout  = now - axisHomeTime[i] > HOMING_SEEK_TIMEOUT_MS;
              // Direction-aware overflow detection
              bool overflow = homeForward
                  ? (ax->currentPosition > (int32_t)(maxRange * HOMING_OVERFLOW_LIMIT_MULT))
                  : (ax->currentPosition < (int32_t)(maxRange * (1.0f - HOMING_OVERFLOW_LIMIT_MULT)));

              if (limitHit || timeout || overflow) {
                DMAStepper_StopAxis(i);
                if (limitHit) {
                  DMAStepper_SetPosition(i, homeForward ? ax->maxPos : ax->minPos);
                  axisHomeState[i] = H_RETRACT;
                  axisHomeTime[i]  = now;
                } else {
                  ax->mode  = MODE_ALARM;
                  ax->homed = false;
                  axisHomeState[i] = H_IDLE;
#if (CONTROLLER_MODE == CONTROLLER_MODE_SLAVE)
                  digitalWrite(ALARM_PIN, LOW);
#endif
                }
              } else if (!ax->stepping) {
                DMAStepper_StartAxis(i, homeForward);
              }
            }
            break;
          case H_RETRACT:
            if (now - axisHomeTime[i] > HOMING_RETRACT_DURATION_MS) {
              DMAStepper_StopAxis(i);
              axisHomeTime[i]  = now;
              axisHomeState[i] = H_RETRACT_SETTLE;
            } else if (!ax->stepping) {
              DMAStepper_StartAxis(i, retractForward);
            }
            break;
          case H_RETRACT_SETTLE:
            if (now - axisHomeTime[i] > HOMING_RETRACT_SETTLE_MS) {
              int32_t centerPos = (ax->minPos + ax->maxPos) / 2;
              DMAStepper_SetTarget(i, centerPos);
              bool toCenterForward = (centerPos > ax->currentPosition);
              DMAStepper_StartAxis(i, toCenterForward);
              axisHomeState[i] = H_MOVING_CENTER;
              axisHomeTime[i]  = now;
            }
            break;
          case H_MOVING_CENTER:
            {
              int32_t centerPos = (ax->minPos + ax->maxPos) / 2;
              if (abs(ax->currentPosition - centerPos) <= HOMING_CENTER_TOLERANCE) {
                DMAStepper_StopAxis(i);
                ax->mode  = MODE_READY;
                ax->homed = true;
                axisHomeState[i] = H_DONE;
                ax->limitedFreq   = 0;
                ax->accelLastTime = millis();
                ax->pidLastTime   = millis();
              } else if (now - axisHomeTime[i] > HOMING_CENTER_TIMEOUT_MS) {
                ax->mode  = MODE_ALARM;
                ax->homed = false;
                DMAStepper_StopAxis(i);
                axisHomeState[i] = H_IDLE;
#if (CONTROLLER_MODE == CONTROLLER_MODE_SLAVE)
                digitalWrite(ALARM_PIN, LOW);
#endif
              } else if (!ax->stepping) {
                bool toCenterForward = (centerPos > ax->currentPosition);
                DMAStepper_StartAxis(i, toCenterForward);
              }
            }
            break;
          default: break;
        }
        break;

      case MODE_PARKING:
        DMAStepper_SetTarget(i, ax->minPos);
        if (abs(ax->currentPosition - ax->targetPosition) <= POSITION_TOLERANCE) {
          DMAStepper_StopAxis(i);
          ax->mode = MODE_PARKED;
          ax->limitedFreq   = 0;
          ax->accelLastTime = millis();
        } else if (!ax->stepping) {
          DMAStepper_SetFrequency(i, PARKING_FREQUENCY_HZ);
          DMAStepper_StartAxis(i, ax->targetPosition > ax->currentPosition);
        }
        break;

      case MODE_UNPARKING:
        {
          int32_t centerPos = (ax->minPos + ax->maxPos) / 2;
          DMAStepper_SetTarget(i, centerPos);
          if (abs(ax->currentPosition - centerPos) <= HOMING_CENTER_TOLERANCE) {
            DMAStepper_StopAxis(i);
            ax->mode = MODE_READY;
            ax->limitedFreq   = 0;
            ax->accelLastTime = millis();
            ax->pidLastTime   = millis();
            if (ax->pendingTarget != PENDING_TARGET_NONE) {
              DMAStepper_SetTarget(i, ax->pendingTarget);
              ax->pendingTarget = PENDING_TARGET_NONE;
            }
          } else if (!ax->stepping) {
            DMAStepper_SetFrequency(i, PARKING_FREQUENCY_HZ);
            DMAStepper_StartAxis(i, centerPos > ax->currentPosition);
          }
        }
        break;

      case MODE_READY:
        {
          int32_t error = ax->targetPosition - ax->currentPosition;
          if (abs(error) > POSITION_DEADZONE) {
            uint32_t desiredFreq;
            bool forward = (error > 0);
            if (ax->pidEnabled) {
              desiredFreq = (uint32_t)computePID(ax, now);
            } else {
              int32_t dist = abs(error);
              desiredFreq = (dist < ACCEL_RAMP_DISTANCE)
                  ? map(dist, 0, ACCEL_RAMP_DISTANCE, MIN_FREQUENCY_HZ, ax->maxFreqHz)
                  : ax->maxFreqHz;
            }
            DMAStepper_SetFrequency(i, applyAccelLimit(ax, desiredFreq));
            if (!ax->stepping) {
              if (ax->pidEnabled) {
                ax->integral = 0; ax->prevError = (float)error;
                ax->derivativeFilter = 0; ax->pidLastTime = millis();
              }
              DMAStepper_StartAxis(i, forward);
            } else if (ax->direction != forward) {
              DMAStepper_StopAxis(i);
              delayMicroseconds(DIRECTION_CHANGE_DELAY_US);
              ax->limitedFreq = 0;
              if (ax->pidEnabled) {
                ax->integral = 0; ax->prevError = (float)error;
                ax->derivativeFilter = 0; ax->pidLastTime = millis();
              }
              DMAStepper_StartAxis(i, forward);
            }
          } else {
            if (ax->stepping) {
              DMAStepper_StopAxis(i);
              if (ax->pidEnabled) ax->integral *= 0.95f;
            }
          }
        }
        break;

      case MODE_CONNECTED: case MODE_DISABLED: case MODE_ALARM:
      case MODE_UNKNOWN: case MODE_PARKED:
        DMAStepper_StopAxis(i);
        break;
    }
  }
}

// =============================================================================
// SPI MULTI-CONTROLLER IMPLEMENTATION
// =============================================================================
#if (CONTROLLER_MODE == CONTROLLER_MODE_MASTER || CONTROLLER_MODE == CONTROLLER_MODE_SLAVE)

void SPI_Controller_Init(void) {
#if (CONTROLLER_MODE == CONTROLLER_MODE_MASTER)
  pinMode(SYNC_PIN, OUTPUT);
  pinMode(ALARM_PIN, INPUT_PULLUP);
  digitalWrite(SYNC_PIN, LOW);
#else
  pinMode(SYNC_PIN, INPUT);
  pinMode(ALARM_PIN, OUTPUT);
  digitalWrite(ALARM_PIN, HIGH);
#endif

  SPI.begin();
#if (CONTROLLER_MODE == CONTROLLER_MODE_MASTER)
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
#else
  // Slave mode handled by STM32duino SPI library
#endif

  memset(&spiTxBuf, 0, sizeof(spiTxBuf));
  memset(&spiRxBuf, 0, sizeof(spiRxBuf));
  spiTxBuf.header = SPI_HEADER_MAGIC;
}

#if (CONTROLLER_MODE == CONTROLLER_MODE_MASTER)

bool SPI_Controller_SendCommand(uint8_t cmd, int32_t* data, bool sync) {
  static uint8_t frameCounter = 0;
  if (digitalRead(ALARM_PIN) == LOW) return false;

  frameCounter++;
  spiTxBuf.frameId = frameCounter;
  spiTxBuf.command = cmd;
  spiTxBuf.flags   = sync ? SPI_FLAG_SYNC : 0;
  for (int i = 0; i < 3; i++) spiTxBuf.cmdData[i] = (data != nullptr) ? data[i] : 0;

  digitalWrite(SPI_NSS_PIN, LOW);
  uint8_t* txPtr = (uint8_t*)&spiTxBuf;
  uint8_t* rxPtr = (uint8_t*)&spiRxBuf;
  for (uint16_t i = 0; i < sizeof(SPI_FRAME); i++) rxPtr[i] = SPI.transfer(txPtr[i]);
  digitalWrite(SPI_NSS_PIN, HIGH);

  if (spiRxBuf.header == SPI_HEADER_MAGIC) spiFrameReady = true;

  if (sync) {
    digitalWrite(SYNC_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(SYNC_PIN, LOW);
    lastSyncTime = millis();
  }
  return true;
}

bool SPI_Controller_IsSlaveReady(void) { return (digitalRead(ALARM_PIN) == HIGH); }
void SPI_Controller_TriggerAlarm(void) {}
SPI_FRAME* SPI_Controller_GetLastFrame(void) { return spiFrameReady ? &spiRxBuf : nullptr; }
void SPI_Controller_Process(void) {}

#else // SLAVE MODE

void SPI_Controller_Process(void) {
  spiTxBuf.slaveStatus = 0;
  for (int i = 0; i < NUM_AXES; i++) {
    spiTxBuf.positions[i] = axisState[i].currentPosition;
    spiTxBuf.targets[i]   = axisState[i].targetPosition;
    spiTxBuf.modes[i]     = axisState[i].mode;
    if (axisState[i].homed) spiTxBuf.slaveStatus |= (1 << i);
    if (axisState[i].mode == MODE_ALARM) spiTxBuf.slaveStatus |= 0x08;
  }

  if (digitalRead(SPI_NSS_PIN) == LOW) {
    uint8_t* txPtr = (uint8_t*)&spiTxBuf;
    uint8_t* rxPtr = (uint8_t*)&spiRxBuf;
    for (uint16_t i = 0; i < sizeof(SPI_FRAME); i++) rxPtr[i] = SPI.transfer(txPtr[i]);

    if (spiRxBuf.header == SPI_HEADER_MAGIC && spiRxBuf.frameId != lastFrameId) {
      lastFrameId = spiRxBuf.frameId;
      switch (spiRxBuf.command) {
        case 1: // CMD_MOVE
          for (int i = 0; i < NUM_AXES; i++) {
            AxisState* ax = &axisState[i];
            if (ax->homed && ax->mode == MODE_READY) DMAStepper_SetTarget(i, spiRxBuf.cmdData[i]);
            else if (ax->mode == MODE_PARKED || ax->mode == MODE_UNPARKING) {
              ax->pendingTarget = spiRxBuf.cmdData[i];
              if (ax->mode == MODE_PARKED) ax->mode = MODE_UNPARKING;
            }
          }
          break;
        case 0: // CMD_HOME
          for (int i = 0; i < NUM_AXES; i++) {
            AxisState* ax = &axisState[i];
            if (!ax) continue;
            if (ax->homed && (ax->mode == MODE_READY || ax->mode == MODE_PARKED)) continue;
            DMAStepper_StartHoming(i);
          }
          break;
        case 2: // CMD_SET_SPEED
          for (int i = 0; i < NUM_AXES; i++) DMAStepper_SetMaxSpeed(i, (uint32_t)spiRxBuf.cmdData[0]);
          break;
        case 3: // CMD_DISABLE
          DMAStepper_StopAll();
          for (int i = 0; i < NUM_AXES; i++) axisState[i].mode = MODE_DISABLED;
          break;
        case 4: // CMD_ENABLE
          for (int i = 0; i < NUM_AXES; i++) axisState[i].mode = axisState[i].homed ? MODE_READY : MODE_CONNECTED;
          break;
        case 6: // CMD_CLEAR_ALARM
          DMAStepper_ClearAlarm();
          break;
        case 7: // CMD_PARK
          for (int i = 0; i < NUM_AXES; i++) if (axisState[i].homed) axisState[i].mode = MODE_PARKING;
          break;
        case 10: // CMD_SET_ACCEL
          for (int i = 0; i < NUM_AXES; i++) axisState[i].maxAccel = spiRxBuf.cmdData[0];
          break;
      }

      if (spiRxBuf.flags & SPI_FLAG_SYNC) {
        uint32_t syncStart = millis();
        while (digitalRead(SYNC_PIN) == LOW) {
          if (millis() - syncStart > 100) break;
        }
      }
    }
  }
}

bool SPI_Controller_SendCommand(uint8_t cmd, int32_t* data, bool sync) { return false; }
bool SPI_Controller_IsSlaveReady(void) { return true; }
void SPI_Controller_TriggerAlarm(void) { digitalWrite(ALARM_PIN, LOW); }
SPI_FRAME* SPI_Controller_GetLastFrame(void) { return nullptr; }

#endif
#endif