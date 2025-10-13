#pragma once

#ifndef MAIN_H
#define MAIN_H

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

// Convert frequency in Hz to microseconds period (supports float Hz)
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

#define TYPE_PID 0
#define TYPE_SETPOINT 1
#define TYPE_THRUST 2
#define TYPE_TELEMETRY 3
#define TYPE_LOAD 254
#define TYPE_SAVE 255

#define AXIS_PITCH 0
#define AXIS_ROLL 1
#define AXIS_YAW 2

#define GAIN_KP 0
#define GAIN_KI 1
#define GAIN_KD 2

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

#define FLASH_BLOCK_WORDS 32
#define FLASH_BLOCK_BYTES (FLASH_BLOCK_WORDS * 4)

#define EMA_ALPHA 0.3f
#define EMA_BETA (1.0f - EMA_ALPHA)

// Empirical temperature correction for sensor offset
#define TEMPERATURE_CORRECTION -3.8f

#define SCHEDULER_TASK_COUNT 4

// Sensor objects for IMU and environment
SensorXYZ accelerometer(BHY2_SENSOR_ID_ACC);
SensorXYZ gyroscope(BHY2_SENSOR_ID_GYRO);
SensorXYZ magnetometer(BHY2_SENSOR_ID_MAG);
Sensor pressure(BHY2_SENSOR_ID_BARO);
Sensor humidity(BHY2_SENSOR_ID_HUM);
Sensor temperature(BHY2_SENSOR_ID_TEMP);
SensorQuaternion quaternion(BHY2_SENSOR_ID_RV);
VL53L4CX vl53l4cx(&Wire, NC);

// Main flight control unit state (128 bytes, see static_assert)
typedef struct __attribute__((aligned(4), packed))
{
  uint16_t thrust, distance;
  float roll_p, roll_i, roll_d, roll_setpoint;
  float pitch_p, pitch_i, pitch_d, pitch_setpoint;
  float yaw_p, yaw_i, yaw_d, yaw_setpoint;
  float pressure, humidity, temperature, battery;
  bool active;
  uint8_t _pad[56]; // Padding to ensure struct is exactly 128 bytes
} Fcu;

static_assert(sizeof(Fcu) == FLASH_BLOCK_BYTES, "Fcu struct must be 128 bytes (32 words)");

typedef struct __attribute__((aligned(4), packed))
{
  uint16_t m1, m2, m3, m4;
} Esc;

static_assert(sizeof(Esc) == 8, "Esc struct must be 8 bytes (2 words)");

typedef struct __attribute__((aligned(1), packed))
{
  uint8_t node, zone, type, data[252];
} DataPacket;

static_assert(sizeof(DataPacket) == 255, "DataPacket struct must be 255 bytes");

typedef struct __attribute__((aligned(4), packed))
{
  float integral, output, prev;
} Pid;

static_assert(sizeof(Pid) == 12, "Pid struct must be 12 bytes (3 words)");

typedef struct
{
  const uint32_t interval_us;
  uint32_t previous_us;
  void (*task)(void);
} Task;

Fcu fcu = {0};
Esc esc = {0x8000, 0x8000, 0x8000, 0x8000};
volatile DataPacket received_packet;
DataPacket transmit_packet = {NODE_ID, ZONE_ID, TYPE_TELEMETRY, {0}};
const DataQuaternion hover_quaternion = {0.0f, 0.0f, 0.0f, 1.0f, 0.0f};
Pid roll_pid = {0};
Pid pitch_pid = {0};
Pid yaw_pid = {0};

static uint32_t global_time_us = 0;

static inline void run_scheduler_tasks(void);

static void update_inertial_measurement_unit(void);
static void update_flight_control_unit(void);
static void update_motor_speed(void);

static void send_radio_packet(void);
static void read_radio_packet(void);

static Task tasks[SCHEDULER_TASK_COUNT] = {
    {HZ_TO_US(401), 0, update_inertial_measurement_unit}, // IMU update at 401 Hz
    {HZ_TO_US(211), 0, update_flight_control_unit},       // FCU update at 211 Hz
    {HZ_TO_US(101), 0, update_motor_speed},               // Motor update at 101 Hz
    {HZ_TO_US(2), 0, send_radio_packet}};                 // Telemetry at 2 Hz

static inline void update_pid(const float setpoint, const float value, const float kp, const float ki, const float kd, float *integral, float *prev_value, float *output);

static inline void extract_float_bytes(float &value, const uint8_t index);

static inline void erase_user_flash(void);
static inline void load_user_flash(void);
static inline void save_user_flash(void);

static inline void handle_pid_packet(void);
static inline void handle_setpoint_packet(void);
static inline void handle_thrust_packet(void);

static inline void multiply_quaternion(DataQuaternion &r, const DataQuaternion &q1, const DataQuaternion &q2);
static inline void normalize_quaternion(DataQuaternion &q);

#endif