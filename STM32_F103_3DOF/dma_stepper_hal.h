#ifndef DMA_STEPPER_HAL_H
#define DMA_STEPPER_HAL_H
#include <Arduino.h>

// =============================================================================
// 🛠️ НАСТРОЙКИ ПОЛЬЗОВАТЕЛЯ (меняйте значения только здесь)
// =============================================================================
// Механика
#define MM_PER_REV 10.0f
#define STEPS_PER_REV 1000
#define MAX_REVOLUTIONS 9.0f

// =============================================================================
// МАКРОСЫ ПРЕОБРАЗОВАНИЯ ЕДИНИЦ
// =============================================================================
// Преобразование скорости (мм/с) в частоту шагов (Гц)
// Формула: частота = (мм/с) × (шагов/оборот) / (мм/оборот)
// Пример: 10 мм/с × 1000 шагов/об / 10 мм/об = 1000 Гц
#define MMPERSECTOFREQHZ(mmpers) ((uint32_t)((float)(mmpers) * (float)STEPS_PER_REV / (float)MM_PER_REV))

// Обратное преобразование: частота (Гц) → скорость (мм/с)
#define FREQHZTOMMPERSEC(freqhz) ((float)(freqhz) * (float)MM_PER_REV / (float)STEPS_PER_REV)

#define MMPERSECTOFREQHZ_SAFE(mmpers) constrain(MMPERSECTOFREQHZ(mmpers), MIN_FREQUENCY_HZ, MAX_FREQUENCY_HZ)

// Ограничения частоты (Гц)
#define MAX_FREQUENCY_HZ 200000
#define SAFE_FREQUENCY_HZ 100000
#define DEFAULT_FREQUENCY_HZ 50000
#define MIN_FREQUENCY_HZ 50

// Скорости специальных режимов (Гц)
#define HOMING_FREQUENCY_HZ MMPERSECTOFREQHZ_SAFE(15)   // Скорость поиска концевика
#define PARKING_FREQUENCY_HZ MMPERSECTOFREQHZ_SAFE(15)  // Скорость парковки

// Таймауты калибровки (мс)
#define HOMING_SEEK_TIMEOUT_MS 30000    // Макс. время поиска концевика
#define HOMING_RETRACT_DURATION_MS 300  // Время отката от сработавшего концевика
#define HOMING_CENTER_TIMEOUT_MS 30000  // Макс. время движения в центр

#define MAX_ACCEL 80000

// Множители хода (относительно MAX_REVOLUTIONS * STEPS_PER_REV)
#define HOMING_TRAVEL_LIMIT_MULT 1.5f    // Целевая позиция при поиске (запас)
#define HOMING_OVERFLOW_LIMIT_MULT 1.4f  // Аварийный предел переполнения

// Допуски и задержки
#define POSITION_TOLERANCE 50         // Допуск достижения позиции (шаги)
#define POSITION_DEADZONE 2           // Мёртвая зона остановки (шаги)
#define ACCEL_RAMP_DISTANCE 50       // Дистанция плавного разгона/торможения (шаги)
#define DIRECTION_CHANGE_DELAY_US 10  // Задержка при смене направления (мкс)
#define HOMING_CENTER_TOLERANCE 100     // Допуск при движении в центр (шаги) - больше чем POSITION_TOLERANCE

// =============================================================================
// СИСТЕМНЫЕ КОНСТАНТЫ (не менять)
// =============================================================================
#define MM_PER_STEP (MM_PER_REV / STEPS_PER_REV)
#define MAX_SPEED_MM_SEC (int)(MAX_FREQUENCY_HZ * MM_PER_STEP)
#define SAFE_SPEED_MM_SEC (int)(SAFE_FREQUENCY_HZ * MM_PER_STEP)
#define DEFAULT_SPEED_MM_SEC (int)(DEFAULT_FREQUENCY_HZ * MM_PER_STEP)
#define NUM_AXES 4

// =============================================================================
// СТРУКТУРЫ ДАННЫХ
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

enum AxisMode { MODE_UNKNOWN = 0,
                MODE_CONNECTED = 1,
                MODE_DISABLED = 2,
                MODE_HOMING = 3,
                MODE_PARKING = 4,
                MODE_READY = 5,
                MODE_ALARM = 6 };
enum HomingSubState { H_IDLE = 0,
                      H_SEEKING,
                      H_RETRACT,
                      H_MOVING_CENTER,
                      H_DONE };

// =============================================================================
// API
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