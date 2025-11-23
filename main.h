#pragma once

#ifndef MAIN_H
#define MAIN_H

#define NODE_ID 1
#define ZONE_ID 0

#include <nrf.h>
#include <Nicla_System.h>

#include <sensors/SensorXYZ.h>
SensorXYZ accelerometer(BHY2_SENSOR_ID_ACC);
SensorXYZ gyroscope(BHY2_SENSOR_ID_GYRO);
SensorXYZ magnetometer(BHY2_SENSOR_ID_MAG);

#include <sensors/Sensor.h>
Sensor pressure(BHY2_SENSOR_ID_BARO);
Sensor humidity(BHY2_SENSOR_ID_HUM);
Sensor temperature(BHY2_SENSOR_ID_TEMP);

#include <sensors/SensorQuaternion.h>
SensorQuaternion quaternion(BHY2_SENSOR_ID_RV);

#ifdef Mode
#undef Mode
#endif

#include <vl53l4cx_class.h>
VL53L4CX vl53l4cx(&Wire, NC);
VL53L4CX_UserRoi_t vl53l4cx_UserRoi = {6, 6, 9, 9};

#define MOTOR1_PIN 11
#define MOTOR2_PIN 28
#define MOTOR3_PIN 27
#define MOTOR4_PIN 29

#define INV(x) (1.0f / ((float)(x) + __FLT_EPSILON__))
#define HZ_TO_US(Hz) ((uint32_t)(1e6f * INV(Hz)))
#define S_TO_US(s) ((uint32_t)(((float)(s) + __FLT_EPSILON__) * 1e6f))
#define S_TO_US_INV(s) (1e6f * INV(s))

#define VL53L4CX_I2C_SPEED 400000

#define MOTOR_MIN 0
#define MOTOR_MAX 800
#define THRUST_HOVER 350
#define THRUST_HEADROOM 100
#define THRUST_MAX (THRUST_HOVER + THRUST_HEADROOM)
#define THRUST_MIN 0
#define SETPOINT_MIN -1.0f
#define SETPOINT_MAX 1.0f

#define PID_GAIN_MAX 100.0f
#define PID_GAIN_MIN 0.0f
#define PID_OUT_MAX ((MOTOR_MAX - THRUST_MAX) / 3.0f)
#define PID_OUT_MIN (-PID_OUT_MAX)
#define PID_I_MAX (PID_OUT_MAX / 2.0f)
#define PID_I_MIN (-PID_I_MAX)
#define PID_LOOP_HZ 211.0f
#define PID_LOOP_PERIOD (1.0f / PID_LOOP_HZ)
#define PID_ARRAY_SIZE 3

/**
 * PiiTune StepSync - Adaptive PID Tuning System
 *
 * Model-free reinforcement learning approach using relay excitation
 * and hypervolume optimization. Sequentially tunes P, D, I gains
 * without artificial limits to find optimal speed-stability balance.
 */
#define TUNE_RAMP_MAX THRUST_HOVER // Max thrust during tuning
#define TUNE_RAMP_S 5.0f           // 5s thrust ramp

#define TUNE_RELAY_HERTZ 0.5f     // 2s period for control dynamics
#define TUNE_RELAY_RADIANS 0.2f   // ±11.5° excitation amplitude
#define TUNE_SETTLE_RADIANS 0.08f // ~4.6° settling threshold

#define TUNE_P_GAIN_INCREMENT 0.1f   // P-gain exploration step
#define TUNE_D_GAIN_INCREMENT 0.01f  // D-gain exploration step
#define TUNE_I_GAIN_INCREMENT 0.001f // I-gain exploration step

#define TUNE_HYPERVOLUME_CONVERGENCE 0.10f // 10% convergence threshold
#define TUNE_NON_RESPONSIVE_PENALTY 0.10f  // Hypervolume threshold for non-responsive tuning (no oscillation)

#define ENV_ALPHA 0.25f
#define D_ALPHA 0.75f

#define TEMPERATURE_OFFSET -3.8f
#define DISTANCE_OFFSET -20

#define SCHEDULER_TASK_COUNT 6
#define PACKET_TYPE_COUNT 4

#define TYPE_PID_TUNE 0
#define TYPE_PID 1
#define TYPE_SETPOINT 2
#define TYPE_THRUST 3
#define TYPE_TELEMETRY 4

#define FCU_STATUS_ACTIVE (1U << 0)
#define FCU_STATUS_POFWARN (1U << 1)
#define FCU_STATUS_AUTOTUNE (1U << 2)

#define FCU_IS_ACTIVE(status) (status & FCU_STATUS_ACTIVE)
#define FCU_SET_ACTIVE(status) (status |= FCU_STATUS_ACTIVE)
#define FCU_CLEAR_ACTIVE(status) (status &= ~FCU_STATUS_ACTIVE)
#define FCU_UPDATE_ACTIVE(status, cond) ((cond) ? FCU_SET_ACTIVE(status) : FCU_CLEAR_ACTIVE(status))

#define FCU_IS_POFWARN(status) (status & FCU_STATUS_POFWARN)
#define FCU_SET_POFWARN(status) (status |= FCU_STATUS_POFWARN)
#define FCU_CLEAR_POFWARN(status) (status &= ~FCU_STATUS_POFWARN)
#define FCU_UPDATE_POFWARN(status, cond) ((cond) ? FCU_SET_POFWARN(status) : FCU_CLEAR_POFWARN(status))

#define FCU_IS_AUTOTUNE(status) (status & FCU_STATUS_AUTOTUNE)
#define FCU_SET_AUTOTUNE(status) (status |= FCU_STATUS_AUTOTUNE)
#define FCU_CLEAR_AUTOTUNE(status) (status &= ~FCU_STATUS_AUTOTUNE)
#define FCU_UPDATE_AUTOTUNE(status, cond) ((cond) ? FCU_SET_AUTOTUNE(status) : FCU_CLEAR_AUTOTUNE(status))

#define FCU_LANDING_STEP(thrust, threshold, step, status) \
  ((thrust) >= (threshold) ? ((thrust) -= (step)) : FCU_CLEAR_ACTIVE(status))

#define FCU_UPDATE_GAIN(gain_array, axis, gain, value, min, max) \
  (gain_array[(axis) % PID_ARRAY_SIZE][(gain) % PID_ARRAY_SIZE] = constrain((value), (min), (max)))

#define FCU_UPDATE_SETPOINT(setpoint_array, axis, value, min, max) \
  (setpoint_array[(axis) % PID_ARRAY_SIZE] = constrain((value), (min), (max)))

#define FCU_UPDATE_THRUST(status, thrust, value, min, max) \
  (thrust = constrain(FCU_IS_POFWARN(status) ? ((value) < (thrust) ? (value) : (thrust)) : (value), (min), (max)))

#define ACCELEROMETER_HZ 400
#define ACCELEROMETER_LATENCY 1
#define ACCELEROMETER_RANGE 8

#define GYROSCOPE_HZ 400
#define GYROSCOPE_LATENCY 1
#define GYROSCOPE_RANGE 1000

#define MAGNETOMETER_HZ 0
#define MAGNETOMETER_LATENCY 0
#define MAGNETOMETER_RANGE 2500

#define PRESSURE_HZ 1
#define PRESSURE_LATENCY 1000

#define HUMIDITY_HZ 1
#define HUMIDITY_LATENCY 1000

#define TEMPERATURE_HZ 1
#define TEMPERATURE_LATENCY 1000

#define QUATERNION_HZ 400
#define QUATERNION_LATENCY 1

typedef struct __attribute__((packed, aligned(4)))
{
  float pid_gain[PID_ARRAY_SIZE][PID_ARRAY_SIZE]; // [roll,pitch,yaw][P,I,D]: 0.0 to 100.0
  float pid_setpoint[PID_ARRAY_SIZE];             // roll, pitch, yaw: -1.0 to 1.0
  float pressure;                                 // hectopascal (hPa)
  float humidity;                                 // percent (%)
  float temperature;                              // Celsius (°C)
  float battery;                                  // Volts (V)
  uint16_t thrust;                                // PWM value (0-800)
  uint16_t distance;                              // millimeters (mm)
  uint8_t status;                                 // bit 0: FCU active, bit 1: POF warning, bit 2: Auto-Tune active
  uint8_t reserved[183];                          // Padding to 252 bytes for RCU data, preserving 4-byte alignment
} Fcu;

static_assert(sizeof(Fcu) == 252, "Fcu struct must be 252 bytes (63 words)");

typedef struct __attribute__((packed, aligned(2)))
{
  uint16_t m1, m2, m3, m4;
} Esc;

static_assert(sizeof(Esc) == 8, "Esc struct must be 8 bytes (2 words)");

typedef struct __attribute__((packed, aligned(1)))
{
  uint8_t node;
  uint8_t zone;
  uint8_t type;
  uint8_t data[252];
} Rcu;

static_assert(sizeof(Rcu) == 255, "Rcu struct must be 255 bytes (63.75 words)");

typedef struct __attribute__((packed, aligned(4)))
{
  float I, Df, pv, out;
} Pid;

static_assert(sizeof(Pid) == 16, "Pid struct must be 16 bytes (4 words)");

typedef struct __attribute__((packed, aligned(4)))
{
  const char *name;
  void (*task)(void);
  const uint32_t interval_us;
  uint32_t previous_us;
} Task;

static_assert(sizeof(Task) == 16, "Task struct must be 16 bytes (4 words)");

typedef struct __attribute__((packed, aligned(4)))
{
  bool at_progress;    // Is autotune in progress
  bool is_at_hover;    // Has the drone reached stable hover
  uint8_t tuning_axis; // Progressing axis (0=roll, 1=pitch, 2=yaw, 3=complete)
  uint8_t padding[1];  // Padding for 4-byte alignment
} TuneGlobalState;

static_assert(sizeof(TuneGlobalState) == 4, "TuneGlobal State struct must be 4 bytes (1 words)");

static Fcu fcu = {0};
static Esc esc = {0x8000, 0x8000, 0x8000, 0x8000};
static volatile Rcu received_packet = {0};
static Pid pid_state[3] = {0};
static TuneGlobalState tune_state = {0};

static inline void task_imu_update(void);
static inline void task_fcu_update(void);
static inline void task_esc_update(void);
static inline void task_tof_update(void);
static inline void task_tel_update(void);
static inline void task_pof_update(void);

static Task tasks[SCHEDULER_TASK_COUNT] = {
    {"IMU", task_imu_update, HZ_TO_US(401), 0},
    {"FCU", task_fcu_update, HZ_TO_US(211), 0},
    {"ESC", task_esc_update, HZ_TO_US(101), 0},
    {"TOF", task_tof_update, HZ_TO_US(31), 0},
    {"TEL", task_tel_update, HZ_TO_US(3), 0},
    {"POF", task_pof_update, HZ_TO_US(2), 0}};

static inline void handle_pid_tune(void);
static inline void handle_pid_update(void);
static inline void handle_setpoint_update(void);
static inline void handle_thrust_update(void);

static void (*const handle_type[PACKET_TYPE_COUNT])(void) = {
    handle_pid_tune,
    handle_pid_update,
    handle_setpoint_update,
    handle_thrust_update,
};

static inline void pid_store_gains(void);
static inline void pid_state_clear(void);
static inline void pid_tune_stop(void);
static inline void pid_calculate(const float sp, const float pv, const float Kp, const float Ki,
                                 const float Kd, float *_I, float *_D, float *_pv, float *out);
static inline bool pid_tune_step(const uint8_t axis, const float err);
static inline bool pid_thrust_ramp(const float target, const float s);

static inline void quaternion_multiply(DataQuaternion *result, const DataQuaternion *q1, const DataQuaternion *q2);
static inline void quaternion_normalize(DataQuaternion *q);

#endif // MAIN_H