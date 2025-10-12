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

#define HZ_TO_US(Hz) (1000000.0f / (Hz))

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

#define PRESSURE_HZ 10
#define PRESSURE_LATENCY 200

#define HUMIDITY_HZ 1
#define HUMIDITY_LATENCY 1000

#define TEMPERATURE_HZ 1
#define TEMPERATURE_LATENCY 500

#define QUATERNION_HZ 400
#define QUATERNION_LATENCY 1

#define FLASH_BLOCK_WORDS 32
#define FLASH_BLOCK_BYTES (FLASH_BLOCK_WORDS * 4)

#define EMA_ALPHA 0.3f
#define EMA_BETA (1.0f - EMA_ALPHA)

SensorXYZ accelerometer(BHY2_SENSOR_ID_ACC);
SensorXYZ gyroscope(BHY2_SENSOR_ID_GYRO);
SensorXYZ magnetometer(BHY2_SENSOR_ID_MAG);
Sensor pressure(BHY2_SENSOR_ID_BARO);
Sensor humidity(BHY2_SENSOR_ID_HUM);
Sensor temperature(BHY2_SENSOR_ID_TEMP);
SensorQuaternion quaternion(BHY2_SENSOR_ID_RV);
VL53L4CX vl53l4cx(&Wire, NC);

typedef struct __attribute__((aligned(4), packed))
{
  uint16_t thrust, distance;
  float roll_p, roll_i, roll_d, roll_setpoint;
  float pitch_p, pitch_i, pitch_d, pitch_setpoint;
  float yaw_p, yaw_i, yaw_d, yaw_setpoint;
  float pressure, humidity, temperature, battery;
  bool active;
  uint8_t _pad[56];
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

Fcu fcu = {0};
Esc esc = {0x8000, 0x8000, 0x8000, 0x8000};
volatile DataPacket rxPacket;
DataPacket txPacket = {NODE_ID, ZONE_ID, TYPE_TELEMETRY, {0}};
const DataQuaternion HoverQuaternion = {0.0f, 0.0f, 0.0f, 1.0f, 0.0f};
Pid roll_pid = {0};
Pid pitch_pid = {0};
Pid yaw_pid = {0};

static inline void updateESC(void);
static inline void updateFCU(void);
static inline void updatePID(const float setpoint, const float value, const float kp, const float ki, const float kd, float *integral, float *prev_value, float *output);

static inline void extractFloat(float &value, const uint8_t index);
static inline void readRCU(void);
static inline void sendRCU(void);

static inline void eraseFlash(void);
static inline void loadFlash(void);
static inline void saveFlash(void);

static inline void handlePID(void);
static inline void handleSetpoint(void);
static inline void handleThrust(void);

static inline void multiplyQuaternion(DataQuaternion &r, const DataQuaternion &q1, const DataQuaternion &q2);
static inline void normalizeQuaternion(DataQuaternion &q);

#endif