#pragma once

#ifndef MAIN_H
#define MAIN_H

#define NODE_ID 1
#define ZONE_ID 0

#include <nrf.h>
#include <Nicla_System.h>
#include <Serial.h>

#include <sensors/Sensor.h>
#include <sensors/SensorQuaternion.h>
#include <sensors/SensorXYZ.h>

#ifdef Mode
#undef Mode
#endif

#include <vl53l4cx_class.h>
#define VL53L4CX_ADDR 0x52

#define LPF_BARO 0.30f
#define HPF_BARO (1.0f - LPF_BARO)
#define LPF_ENV 0.10f
#define HPF_ENV (1.0f - LPF_ENV)
#define LPF_DISTANCE 0.25f
#define HPF_DISTANCE (1.0f - LPF_DISTANCE)
#define ONE_SECOND_IN_US 1000000.0f
#define HZ_TO_US(Hz) (ONE_SECOND_IN_US / (Hz))
#define INV_SEA_LEVEL_PRESSURE (1.0f / 1013.25f)
#define PA_TO_HPA 0.01f
#define TEMP_OFFSET 5.6f
#define PWM_BASE_CLOCK 16000000UL
#define PWM_FREQUENCY 20000UL
#define PWM_TOP (PWM_BASE_CLOCK / PWM_FREQUENCY)

#define BARO_ALTITUDE_CONSTANT 44307.694f
#define BARO_PRESSURE_EXPONENT 0.190284f

#define KP_PITCH 0.0f
#define KI_PITCH 0.0f
#define KD_PITCH 0.0f

#define KP_ROLL 0.0f
#define KI_ROLL 0.0f
#define KD_ROLL 0.0f

#define KP_YAW 0.0f
#define KI_YAW 0.0f
#define KD_YAW 0.0f

#define MOTOR1_PIN 11
#define MOTOR2_PIN 28
#define MOTOR3_PIN 27
#define MOTOR4_PIN 29

#define ROLL_SETPOINT 0.0f
#define PITCH_SETPOINT 0.0f
#define YAW_SETPOINT 0.0f
#define THRUST_SETPOINT 0.0f

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

#define SERIAL_BAUDRATE 115200

// UICR CUSTOMER area layout
#define UICR_BLOCK_SIZE 32 // 32 words (128 bytes)
#define UICR_BYTE_COUNT (UICR_BLOCK_SIZE * 4)
#define UICR_WRITE_LIMIT 5 // Erase required every 5 writes

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
  // Thrust and setpoints
  uint16_t thrust;
  uint16_t distance;
  float setpoint_pitch;
  float setpoint_roll;
  float setpoint_yaw;

  // PID values for each axis/gain
  float pid_pitch_p;
  float pid_pitch_i;
  float pid_pitch_d;
  float pid_roll_p;
  float pid_roll_i;
  float pid_roll_d;
  float pid_yaw_p;
  float pid_yaw_i;
  float pid_yaw_d;

  // Other telemetry
  float roll;
  float pitch;
  float yaw;
  float pressure;
  float altitude;
  float humidity;
  float temperature;

  bool armed;
  uint8_t _pad[3]; // for alignment
} Fcu;

typedef struct __attribute__((aligned(4), packed))
{
  uint16_t m1;
  uint16_t m2;
  uint16_t m3;
  uint16_t m4;
} Esc;

typedef struct __attribute__((aligned(4), packed))
{
  float setpoint;
  float kp, ki, kd;
  float integral;
  float previous_value;
  float output;
} Pid;

typedef struct __attribute__((packed))
{
  uint8_t node;
  uint8_t zone;
  uint8_t type;
  uint8_t data[252];
} DataPacket;

typedef struct __attribute__((aligned(4), packed))
{
  float roll_kp, roll_ki, roll_kd;
  float pitch_kp, pitch_ki, pitch_kd;
  float yaw_kp, yaw_ki, yaw_kd;
  uint32_t write_count;
  uint32_t _pad[(UICR_BYTE_COUNT - 40) / 4];
} FlashBlock;

extern Fcu fcu;
extern Esc esc;
extern FlashBlock flashData;
extern const DataQuaternion HoverQuaternion;
extern Pid rollPid;
extern Pid pitchPid;
extern Pid yawPid;
extern volatile DataPacket rxPacket;
extern DataPacket txPacket;

void setup(void);
void loop(void);

static inline void rcuInit(void);
static inline void sendDataPacket(void);
static inline void pwmInit(void);
static inline void clkInit(void);
static inline void sysInit(void);
static inline void imuInit(void);
static inline void tofInit(void);
static inline void quaternionMultiply(DataQuaternion &r, const DataQuaternion &q1, const DataQuaternion &q2);
static inline void quaternionNormalize(DataQuaternion &q);
static inline void updateEsc(void);
static inline void resetState(void);
static inline void updatePid(Pid &pid, const float value);
static inline void updateFlightControl(void);
static inline void parseDataPacket(void);
static inline void handlePidPacket(void);
static inline void handleSetpointPacket(void);
static inline void handleThrustPacket(void);
static inline void extractFloatFromData(float &value, const uint8_t index);
static inline void checkUsbAndCharge(void);

// UICR Flash functions
static inline void eraseFlashData(void);
static inline void saveFlashData(void);
static inline void loadFlashData(void);

#endif