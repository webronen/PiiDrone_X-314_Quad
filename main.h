#pragma once

#ifndef MAIN_H
#define MAIN_H

#include <Nicla_System.h>

#include <sensors/Sensor.h>
#include <sensors/SensorQuaternion.h>
#include <sensors/SensorXYZ.h>

// Constants for filters and timing
#define LPF 0.1f
#define HPF (1.0f - LPF)
#define ONE_SECOND_IN_US 1000000
#define INV_ONE_SECOND_IN_US (1.0f / ONE_SECOND_IN_US)
#define HZ_TO_US(Hz) (ONE_SECOND_IN_US / (Hz))
#define INV_SEA_LEVEL_PRESSURE (1.0f / 1013.25f)
#define TEMPERATURE_CORRECTION_FACTOR 6.95f
#define EPSILON 1e-6f

// Constants for altitude calculation
#define BARO_ALTITUDE_CONSTANT 44307.694f
#define BARO_PRESSURE_EXPONENT 0.190284f

// PID constants for roll, pitch, yaw, thrust, and altitude
#define KP_ROLL 1.0f
#define KI_ROLL 0.03f
#define KD_ROLL 0.05f
#define WP_ROLL 1.0f
#define WI_ROLL 1.0f
#define WD_ROLL 1.0f

#define KP_PITCH 1.0f
#define KI_PITCH 0.03f
#define KD_PITCH 0.05f
#define WP_PITCH 1.0f
#define WI_PITCH 1.0f
#define WD_PITCH 1.0f

#define KP_YAW 0.6f
#define KI_YAW 0.02f
#define KD_YAW 0.0f
#define WP_YAW 1.0f
#define WI_YAW 1.0f
#define WD_YAW 1.0f

#define KP_THRUST 0.8f
#define KI_THRUST 0.04f
#define KD_THRUST 0.1f
#define WP_THRUST 1.0f
#define WI_THRUST 1.0f
#define WD_THRUST 1.0f

#define KP_ALTITUDE 0.8f
#define KI_ALTITUDE 0.04f
#define KD_ALTITUDE 0.2f
#define WP_ALTITUDE 1.0f
#define WI_ALTITUDE 1.0f
#define WD_ALTITUDE 1.0f

// PID initialization macro
#define INIT_PID(pid, setpoint, kp, ki, kd, wp, wi, wd) \
  {setpoint, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, kp, ki, kd, wp, wi, wd}

// Motor pin definitions
#define MOTOR1_PIN 11
#define MOTOR2_PIN 28
#define MOTOR3_PIN 27
#define MOTOR4_PIN 29

// Setpoints for roll, pitch, yaw, altitude, and thrust
#define ROLL_SETPOINT 0.0f
#define PITCH_SETPOINT 0.0f
#define YAW_SETPOINT 0.0f
#define ALTITUDE_SETPOINT 0.0f
#define THRUST_SETPOINT 0.0f

// Thresholds for PID and thrust
#define THRESHOLD_INTEGRAL 100.0f
#define THRESHOLD_ERROR 0.1f
#define PID_OUTPUT_MIN -0x3FFF
#define PID_OUTPUT_MAX 0x3FFF
#define THRESHOLD_THRUST 10.0f

// Sensor configuration constants
#define ACCELEROMETER_HZ 400
#define ACCELEROMETER_LATENCY 1
#define ACCELEROMETER_RANGE 4

#define GYROSCOPE_HZ 400
#define GYROSCOPE_LATENCY 1
#define GYROSCOPE_RANGE 1000

#define MAGNETOMETER_HZ 25
#define MAGNETOMETER_LATENCY 4
#define MAGNETOMETER_RANGE 2500

#define PRESSURE_HZ 1
#define PRESSURE_LATENCY 100
#define HUMIDITY_HZ 1
#define HUMIDITY_LATENCY 100
#define TEMPERATURE_HZ 1
#define TEMPERATURE_LATENCY 100

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

// Flight control unit structure
typedef struct
{
  float roll;
  float pitch;
  float yaw;
  float thrust;
  float pressure;
  float altitude;
  float humidity;
  float temperature;
  int8_t rssi;
} FCU;

// ESC structure for motor control
typedef struct
{
  uint16_t motor1, motor2, motor3, motor4;
} ESC;

// PID structure
typedef struct
{
  float setpoint;
  float error;
  float lastError;
  float integral;
  float derivative;
  float output;
  float kp, ki, kd;
  float wp, wi, wd;
  float lastMeasurement;
} PID;

// Data packet structure
typedef struct
{
  uint8_t node;
  uint8_t zone;
  uint8_t type;
  uint8_t data[252];
} DataPacket;

// Data packet types
#define TYPE_QUATERNION 0x01
#define TYPE_PRESSURE 0x02
#define TYPE_TEMPERATURE 0x03
#define TYPE_ALTITUDE 0x04
#define TYPE_PID 0x05
#define TYPE_RSSI 0x06
#define TYPE_HUMIDITY 0x07
#define TYPE_MOTOR 0x08

// External variables
extern FCU fcu;
extern ESC esc;
extern DataQuaternion hoverQuaternion;
extern volatile DataPacket rx_packet;
extern DataPacket tx_packet;
extern PID rollPID;
extern PID pitchPID;
extern PID yawPID;
extern PID thrustPID;
extern PID altitudePID;

// Function declarations
static inline void initialize(void);
static inline void quaternionMultiply(DataQuaternion &result, const DataQuaternion &q1, const DataQuaternion &q2);
static inline void quaternionNormalize(DataQuaternion &q);
static inline void setControlInputs(float desiredYaw, float desiredPitch, float desiredRoll, float desiredThrust);
static inline void updateESC(void);
static inline void updatePID(PID &pid, float currentValue);
static inline void updateFlightControl(void);
static inline void sendRadioData(void);

#endif