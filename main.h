#pragma once

#ifndef MAIN_H
#define MAIN_H

// #define DEBUG
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

#define MOTOR1_PIN 11
#define MOTOR2_PIN 28
#define MOTOR3_PIN 27
#define MOTOR4_PIN 29

#define HZ_TO_US(Hz) ((uint32_t)(1000000.0f / (Hz)))

#define PWM_BASE_CLOCK 16000000UL
#define PWM_FREQUENCY 20000UL
#define PWM_TOP (PWM_BASE_CLOCK / PWM_FREQUENCY)

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

#define TELEMETRY_DATA_BYTES 252
#define FLASH_BLOCK_WORDS 32

#define EMA_ALPHA 0.3f
#define EMA_BETA (1.0f - EMA_ALPHA)

#define TEMPERATURE_OFFSET -3.8f
#define DISTANCE_OFFSET -20

#define SCHEDULER_TASK_COUNT 6
#define REQUEST_HANDLER_COUNT 4

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

// Struct definitions with static assertions for size verification
typedef struct __attribute__((packed, aligned(1)))
{
  float pid_gain[3][3];  // PID gains: [roll, pitch, yaw][Kp, Ki, Kd], range 0.0–100.0, default 0.0
  float pid_setpoint[3]; // PID setpoints: [roll, pitch, yaw], unit: rad/s, range: -1.0 to 1.0, default: 0.0
  float pressure;        // Barometric pressure, unit: Pascals (Pa), range: 30000–110000 Pa (300–1100 hPa)
  float humidity;        // Relative humidity, unit: percent (%), range: 0–100%
  float temperature;     // Temperature, unit: degrees Celsius (°C), range: -40 to 85°C
  float battery;         // Battery voltage, unit: volts (V), range: 0.0–4.2V
  uint16_t thrust;       // Thrust command, unit: uint16_t, range: 0–800 (0–100%)
  uint16_t distance;     // ToF distance, unit: mm, range: 20–4000 (2–400 cm)
  uint8_t status;        /**  Status flags (LSb first):
                          *   Bit | Description           | 0 (false) | 1 (true)
                          *   ----|-----------------------|-----------|----------
                          *    0  | FCU active            | Inactive  | Active
                          *    1  | Power failure warning | Normal    | Warning
                          *  2-7  | Reserved              | (0)       | (0)
                          */
  uint8_t reserved[183]; // Padding to make the struct size 252 bytes (63 words) total for telemetry
} Fcu;

static_assert(sizeof(Fcu) == TELEMETRY_DATA_BYTES, "Fcu struct must be 252 bytes (63 words)");

typedef struct __attribute__((packed, aligned(4)))
{
  uint16_t m1, m2, m3, m4;
} Esc;

static_assert(sizeof(Esc) == 8, "Esc struct must be 8 bytes (2 words)");

typedef struct __attribute__((packed, aligned(1)))
{
  uint8_t node;                       // Node ID for this or other devices (e.g. drone, car)
  uint8_t zone;                       // Zone ID for this or other devices (e.g. indoor, outdoor)
  uint8_t type;                       // Packet type (e.g. pid, setpoint, thrust, telemetry)
  uint8_t data[TELEMETRY_DATA_BYTES]; // Payload data for requests or telemetry (252 bytes, 63 words)
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

// Object instances
static Fcu fcu = {0};
static Esc esc = {0x8000, 0x8000, 0x8000, 0x8000};
static volatile Rcu received_packet = {0};
static Rcu transmit_packet = {NODE_ID, ZONE_ID, TYPE_TELEMETRY, {0}};
static Pid pid_state[3] = {0};
static const DataQuaternion hover_quaternion = {0.0f, 0.0f, 0.0f, 1.0f, 0.0f};

// Scheduler function declaration
static inline void run_scheduler_tasks(const uint32_t time_us);

// Task function declarations
static inline void task_update_imu(void);
static inline void task_update_fcu(void);
static inline void task_update_esc(void);
static inline void task_update_tof(void);
static inline void task_send_rcu(void);
static inline void task_handle_pof(void);

// Scheduler task list
static Task tasks[SCHEDULER_TASK_COUNT] = {
    {HZ_TO_US(401), 0, "IMU", task_update_imu},
    {HZ_TO_US(211), 0, "FCU", task_update_fcu},
    {HZ_TO_US(101), 0, "ESC", task_update_esc},
    {HZ_TO_US(31), 0, "TOF", task_update_tof},
    {HZ_TO_US(3), 0, "TEL", task_send_rcu},
    {HZ_TO_US(2), 0, "POF", task_handle_pof}};

// Common function declarations
static inline void read_rcu(void);
static inline void update_pid(const float setpoint, const float value, const float kp, const float ki, const float kd, float *integral, float *prev_value, float *output);
static inline void multiply_quaternion(DataQuaternion &r, const DataQuaternion &q1, const DataQuaternion &q2);
static inline void normalize_quaternion(DataQuaternion &q);

// Handlers for RCU commands
static inline void handle_pid_request(void);
static inline void handle_setpoint_request(void);
static inline void handle_thrust_request(void);
static inline void handle_save_request(void);

#endif // MAIN_H