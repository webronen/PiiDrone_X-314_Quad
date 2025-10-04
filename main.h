#pragma once

#ifndef MAIN_H
#define MAIN_H

#define NODE_ID 1
#define ZONE_ID 0

#include <nrf.h>
#include <Nicla_System.h>
#include <Serial.h>
#include "BQ25120A.h"

#include <sensors/Sensor.h>
#include <sensors/SensorQuaternion.h>
#include <sensors/SensorXYZ.h>

// Remove conflict before including vl53l4cx_class.h
#ifdef Mode
#undef Mode
#endif

#include <vl53l4cx_class.h>

// Constants for filters and timing
#define LPF 0.1f
#define HPF (1.0f - LPF)
#define ONE_SECOND_IN_US 1000000
#define HZ_TO_US(Hz) (ONE_SECOND_IN_US / (Hz))
#define INV_SEA_LEVEL_PRESSURE (1.0f / 1013.25f)
#define TEMPERATURE_CORRECTION_FACTOR 6.95f
#define PWM_BASE_CLOCK 16000000UL                // nRF52 PWM default (16MHz)
#define PWM_FREQUENCY 20000UL                    // 20kHz target frequency
#define PWM_TOP (PWM_BASE_CLOCK / PWM_FREQUENCY) // 19.975kHz PWM (-0.125% error)

// Constants for altitude calculation
#define BARO_ALTITUDE_CONSTANT 44307.694f
#define BARO_PRESSURE_EXPONENT 0.190284f

// PID default gains
#define KP_PITCH 0.0f
#define KI_PITCH 0.0f
#define KD_PITCH 0.0f

#define KP_ROLL 0.0f
#define KI_ROLL 0.0f
#define KD_ROLL 0.0f

#define KP_YAW 0.0f
#define KI_YAW 0.0f
#define KD_YAW 0.0f

// Motor pin definitions
#define MOTOR1_PIN 11
#define MOTOR2_PIN 28
#define MOTOR3_PIN 27
#define MOTOR4_PIN 29

// Setpoints for roll, pitch, yaw, and thrust
#define ROLL_SETPOINT 0.0f
#define PITCH_SETPOINT 0.0f
#define YAW_SETPOINT 0.0f
#define THRUST_SETPOINT 0.0f

// Thresholds for PID and Setpoint
#define PID_LOOP_HZ 211.0f                   // The frequency of the PID loop
#define PID_LOOP_PERIOD (1.0f / PID_LOOP_HZ) // The period (delta time) between updates

#define PID_MAX 800.0f
#define PID_MIN -800.0f
#define GAIN_MAX 100.0f
#define GAIN_MIN 0.0f
#define SETPOINT_MAX 1.0f
#define SETPOINT_MIN -1.0f
#define THRUST_MIN 0.0f

// Data packet types
#define TYPE_PID 0
#define TYPE_SETPOINT 1
#define TYPE_THRUST 2
#define TYPE_TELEMETRY 3

// Axis types
#define AXIS_PITCH 0
#define AXIS_ROLL 1
#define AXIS_YAW 2

// Gain types
#define GAIN_KP 0
#define GAIN_KI 1
#define GAIN_KD 2

// Sensor configuration constants
#define ACCELEROMETER_HZ 400
#define ACCELEROMETER_LATENCY 1
#define ACCELEROMETER_RANGE 8 // ±8g

#define GYROSCOPE_HZ 400
#define GYROSCOPE_LATENCY 1
#define GYROSCOPE_RANGE 1000 // ±1000°/s

#define MAGNETOMETER_HZ 25
#define MAGNETOMETER_LATENCY 40
#define MAGNETOMETER_RANGE 2500 // ±2500µT

#define PRESSURE_HZ 1
#define PRESSURE_LATENCY 1000
#define HUMIDITY_HZ 1
#define HUMIDITY_LATENCY 1000
#define TEMPERATURE_HZ 1
#define TEMPERATURE_LATENCY 1000

#define QUATERNION_HZ 400
#define QUATERNION_LATENCY 1

#define SERIAL_BAUDRATE 115200

// Sensor objects
SensorXYZ accelerometer(BHY2_SENSOR_ID_ACC);
SensorXYZ gyroscope(BHY2_SENSOR_ID_GYRO);
SensorXYZ magnetometer(BHY2_SENSOR_ID_MAG);
Sensor pressure(BHY2_SENSOR_ID_BARO);
Sensor humidity(BHY2_SENSOR_ID_HUM);
Sensor temperature(BHY2_SENSOR_ID_TEMP);
SensorQuaternion quaternion(BHY2_SENSOR_ID_RV);

// BQ25120A power management
BQ25120A bq25120a;

// VL53L4CX Time-of-Flight distance sensor
VL53L4CX vl53l4cx(&Wire, NC); // XSHUT pin not connected (NC)

// Flight control unit structure
typedef struct
{
  float thrust;
  float roll;
  float pitch;
  float yaw;
  float pressure;
  float altitude;
  float humidity;
  float temperature;
} FCU;

// ESC structure for motor control
typedef struct
{
  uint16_t m1;
  uint16_t m2;
  uint16_t m3;
  uint16_t m4;
} ESC;

// PID structure
typedef struct
{
  float setpoint;
  float kp, ki, kd;
  float integral;
  float previous_value;
  float output;
} PID;

// Data packet structure
typedef struct __attribute__((packed))
{
  uint8_t node;
  uint8_t zone;
  uint8_t type;
  uint8_t data[252];
} DataPacket;

// External variables
extern FCU fcu;
extern ESC esc;
extern const DataQuaternion hoverQuaternion;
extern volatile DataPacket rx_packet;
extern DataPacket tx_packet;
extern PID roll;
extern PID pitch;
extern PID yaw;
extern PID thrust;

// Function declarations
static inline void radioInit(void);
static inline void sendDataPacket(void);
static inline void pwmInit(void);
static inline void timerInit(void);
static inline void niclaInit(void);
static inline void imuInit(void);
static inline void quaternionMultiply(DataQuaternion &r, const DataQuaternion &q1, const DataQuaternion &q2);
static inline void setControlInputs(const float thrust, const float roll, const float pitch, const float yaw);
static inline void updateESC(void);
static inline void updatePID(PID &pid, const float value);
static inline void updateFlightControl(void);
static inline void parseDataPacket(void);
static inline void handlePidPacket(void);
static inline void handleSetpointPacket(void);
static inline void handleThrustPacket(void);
static inline void extractFloatFromData(float &value, const uint8_t index);

#endif