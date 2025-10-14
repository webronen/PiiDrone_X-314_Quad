#pragma once

#ifndef MAIN_H
#define MAIN_H

// #define DEBUG
#define NODE_ID 1
#define ZONE_ID 0

#include <nrf.h>
#include <Nicla_System.h>
#include <sensors/Sensor.h>
#include <sensors/SensorQuaternion.h>
#include <sensors/SensorXYZ.h>

#ifdef Mode
#undef Mode
#endif

#include <vl53l4cx_class.h>

#define HZ_TO_US(Hz) ((uint32_t)(1000000.0f / (Hz)))

#define PWM_BASE_CLOCK 16000000UL
#define PWM_FREQUENCY 20000UL
#define PWM_TOP (PWM_BASE_CLOCK / PWM_FREQUENCY)

#define MOTOR1_PIN 11
#define MOTOR2_PIN 28
#define MOTOR3_PIN 27
#define MOTOR4_PIN 29

#define PID_LOOP_HZ 211.0f
#define PID_LOOP_PERIOD (1.0f / PID_LOOP_HZ)

#define PID_MAX 800.0f
#define PID_MIN -800.0f
#define GAIN_MAX 100.0f
#define GAIN_MIN 0.0f
#define SETPOINT_MAX 1.0f
#define SETPOINT_MIN -1.0f
#define THRUST_MAX 800
#define THRUST_MIN 0
#define FLASH_BLOCK_WORDS 32
#define FLASH_BLOCK_BYTES (FLASH_BLOCK_WORDS * 4)
#define EMA_ALPHA 0.3f
#define EMA_BETA (1.0f - EMA_ALPHA)
#define TEMP_CORRECTION -3.8f
#define DIST_CORRECTION -20
#define DIST_MAX 4000
#define DIST_MIN 20
#define BAT_V_MAX 4.2f
#define BAT_V_MIN 0.0f

#define SCHEDULER_TASK_COUNT 6
#define HANDLER_TABLE_SIZE 5
#define TYPE_PID 0
#define TYPE_SETPOINT 1
#define TYPE_THRUST 2
#define TYPE_LOAD 3
#define TYPE_SAVE 4
#define TYPE_TELEMETRY 5

#define ACCELEROMETER_HZ 400
#define ACCELEROMETER_LATENCY 1
#define ACCELEROMETER_RANGE 8

#define GYROSCOPE_HZ 400
#define GYROSCOPE_LATENCY 1
#define GYROSCOPE_RANGE 1000

#define MAGNETOMETER_HZ 25
#define MAGNETOMETER_LATENCY 40
#define MAGNETOMETER_RANGE 2500

#define PRESSURE_HZ 1
#define PRESSURE_LATENCY 1000

#define HUMIDITY_HZ 1
#define HUMIDITY_LATENCY 1000

#define TEMPERATURE_HZ 1
#define TEMPERATURE_LATENCY 1000

#define QUATERNION_HZ 400
#define QUATERNION_LATENCY 1

#ifdef DEBUG
#define DEBUG_FUNC_TIME_START()     \
  NRF_TIMER0->TASKS_CAPTURE[1] = 1; \
  uint32_t __start_us = NRF_TIMER0->CC[1];

#define DEBUG_FUNC_TIME_END(msg)         \
  NRF_TIMER0->TASKS_CAPTURE[2] = 1;      \
  uint32_t __end_us = NRF_TIMER0->CC[2]; \
  printf("%s: %lu us\n", msg, (__end_us - __start_us));
#endif

SensorXYZ accelerometer(BHY2_SENSOR_ID_ACC);
SensorXYZ gyroscope(BHY2_SENSOR_ID_GYRO);
SensorXYZ magnetometer(BHY2_SENSOR_ID_MAG);
Sensor pressure(BHY2_SENSOR_ID_BARO);
Sensor humidity(BHY2_SENSOR_ID_HUM);
Sensor temperature(BHY2_SENSOR_ID_TEMP);
SensorQuaternion quaternion(BHY2_SENSOR_ID_RV);
VL53L4CX vl53l4cx(&Wire, NC);

typedef struct __attribute__((packed, aligned(4)))
{
  float pid[3][3];
  float setpoint[3];
  float pressure;
  float humidity;
  float temperature;
  float battery;
  uint16_t thrust;
  uint16_t distance;
  uint8_t status;
  uint8_t reserved[59];
} Fcu;

static_assert(sizeof(Fcu) == FLASH_BLOCK_BYTES, "Fcu struct must be 128 bytes (32 words)");

typedef struct __attribute__((packed, aligned(4)))
{
  uint16_t m1, m2, m3, m4;
} Esc;

static_assert(sizeof(Esc) == 8, "Esc struct must be 8 bytes (2 words)");

typedef struct __attribute__((packet, aligned(1)))
{
  uint8_t node;
  uint8_t zone;
  uint8_t type;
  uint8_t data[252];
} Rcu;

static_assert(sizeof(Rcu) == 255, "Rcu struct must be 255 bytes");

typedef struct __attribute__((packed, aligned(4)))
{
  float integral, output, prev;
} Pid;

static_assert(sizeof(Pid) == 12, "Pid struct must be 12 bytes (3 words)");

typedef struct __attribute__((packed, aligned(4)))
{
  const uint32_t interval_us;
  uint32_t previous_us;
  const char *name;
  void (*task)(void);
} Task;

static_assert(sizeof(Task) == 16, "Task struct must be 16 bytes (4 words)");

static Fcu fcu = {0};
static Esc esc = {0x8000, 0x8000, 0x8000, 0x8000};
volatile Rcu received_packet;
static Rcu transmit_packet = {NODE_ID, ZONE_ID, TYPE_TELEMETRY, {0}};
static Pid pid_state[3] = {0};
const DataQuaternion hover_quaternion = {0.0f, 0.0f, 0.0f, 1.0f, 0.0f};

static inline void run_tasks(const uint32_t now_us);
static inline void update_imu(void);
static inline void update_fcu(void);
static inline void update_esc(void);
static inline void update_tof(void);
static inline void send_rcu(void);
static inline void handle_pof(void);

static Task tasks[SCHEDULER_TASK_COUNT] = {
    {HZ_TO_US(401), 0, "IMU", update_imu},
    {HZ_TO_US(211), 0, "FCU", update_fcu},
    {HZ_TO_US(101), 0, "ESC", update_esc},
    {HZ_TO_US(31), 0, "TOF", update_tof},
    {HZ_TO_US(3), 0, "TEL", send_rcu},
    {HZ_TO_US(2), 0, "POF", handle_pof}};

static inline void read_rcu(void);
static inline void update_pid(const float setpoint, const float value, const float kp, const float ki, const float kd, float *integral, float *prev_value, float *output);

static inline void handle_load_request(void);
static inline void handle_save_request(void);
static inline void handle_pid_request(void);
static inline void handle_setpoint_request(void);
static inline void handle_thrust_request(void);

static inline void multiply_quaternion(DataQuaternion &r, const DataQuaternion &q1, const DataQuaternion &q2);
static inline void normalize_quaternion(DataQuaternion &q);

#endif // MAIN_H