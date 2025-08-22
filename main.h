#pragma once

#ifndef MAIN_H
#define MAIN_H

#define DEBUG

#include <Nicla_System.h>
#include <Serial.h>

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
#define PWM_BASE_CLOCK 16000000UL                        // nRF52 PWM default (16MHz)
#define PWM_FREQUENCY 20000UL                            // 20kHz target frequency
#define PWM_COUNTER_TOP (PWM_BASE_CLOCK / PWM_FREQUENCY) // 19.975kHz PWM (-0.125% error)

// Constants for altitude calculation
#define BARO_ALTITUDE_CONSTANT 44307.694f
#define BARO_PRESSURE_EXPONENT 0.190284f

// Quaternion components range from -1.0 to 1.0
// Much smaller gains needed compared to degree/radian based systems
#define KP_PITCH 0.01f // 1.
#define KI_PITCH 0.0f // 3.
#define KD_PITCH 0.0f // 2.

#define KP_ROLL 0.0f // 4.
#define KI_ROLL 0.0f // 6.
#define KD_ROLL 0.0f // 5.

#define KP_YAW 0.0f // 7.
#define KI_YAW 0.0f // 9.
#define KD_YAW 0.0f // 8,

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

// Thresholds for PID
#define PID_OUTPUT_MAX 800
#define PID_OUTPUT_MIN -800

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
  uint16_t motor1;
  uint16_t motor2;
  uint16_t motor3;
  uint16_t motor4;
} ESC;

// PID structure
typedef struct
{
  float setpoint;
  float kp, ki, kd;
  float integral;
  float previous_error;
  float output;
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
extern const DataQuaternion hoverQuaternion;
extern volatile DataPacket rx_packet;
extern DataPacket tx_packet;
extern PID rollPID;
extern PID pitchPID;
extern PID yawPID;
extern PID thrustPID;

// Function declarations
static inline void initialize(void);
static inline void radioInit(void);
static inline void sendDataPacket(void);
static inline void pwmInit(void);
static inline void timerInit(void);
static inline void niclaInit(void);
static inline void imuInit(void);
static inline void quaternionMultiply(DataQuaternion &result, const DataQuaternion &q1, const DataQuaternion &q2);
static inline void quaternionNormalize(DataQuaternion &q);
static inline void setControlInputs(const float thrust, const float roll, const float pitch, const float yaw);
static inline void updateESC(void);
static inline void updatePID(PID &pid, float currentValue);
static inline void updateFlightControl(void);

#endif