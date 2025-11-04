#pragma once

#ifndef MAIN_H
#define MAIN_H

#define NODE_ID 1
#define ZONE_ID 0

#include <nrf.h>
#include <nrf_delay.h>
#include <Nicla_System.h>

#include <common.h>
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

#define MOTOR1_PIN 11 // M1: Front-left (CCW)
#define MOTOR2_PIN 28 // M2: Front-right (CW)
#define MOTOR3_PIN 27 // M3: Rear-left (CW)
#define MOTOR4_PIN 29 // M4: Rear-right (CCW)

#define HZ_TO_US(Hz) ((uint32_t)(1000000.0f / (Hz)))
#define VL53L4CX_I2C_SPEED 400000 // 400kHz

#define MOTOR_MIN 0
#define MOTOR_MAX 800

#define I_MIN -200.0f
#define I_MAX 200.0f

#define PID_MIN -400.0f
#define PID_MAX 400.0f

#define PID_FREQUENCY 211.0f
#define PID_PERIOD (1.0f / PID_FREQUENCY)

#define GAIN_MAX 1000.0f
#define GAIN_MIN 0.0f

#define SETPOINT_MAX 1.0f
#define SETPOINT_MIN -1.0f

#define EMA_ALPHA 0.3f
#define EMA_BETA (1.0f - EMA_ALPHA)

#define TEMPERATURE_OFFSET -3.8f
#define DISTANCE_OFFSET -20

#define SCHEDULER_TASK_COUNT 6
#define PACKET_TYPE_COUNT 4

#define TYPE_PID 0
#define TYPE_SETPOINT 1
#define TYPE_THRUST 2
#define TYPE_FLASH 3
#define TYPE_TELEMETRY 4

#define PID_DEPTH 3
#define PID_FILE_WORDS (PID_DEPTH * 3)
#define PID_FILE_BYTES (PID_FILE_WORDS * 4)

#define FCU_STATUS_ACTIVE (1U << 0)
#define FCU_STATUS_POFWARN (1U << 1)

#define FCU_IS_ACTIVE(status) (status & FCU_STATUS_ACTIVE)
#define FCU_SET_ACTIVE(status) (status |= FCU_STATUS_ACTIVE)
#define FCU_CLEAR_ACTIVE(status) (status &= ~FCU_STATUS_ACTIVE)
#define FCU_UPDATE_ACTIVE(status, cond) ((cond) ? FCU_SET_ACTIVE(status) : FCU_CLEAR_ACTIVE(status))

#define FCU_IS_POFWARN(status) (status & FCU_STATUS_POFWARN)
#define FCU_SET_POFWARN(status) (status |= FCU_STATUS_POFWARN)
#define FCU_CLEAR_POFWARN(status) (status &= ~FCU_STATUS_POFWARN)
#define FCU_UPDATE_POFWARN(status, cond) ((cond) ? FCU_SET_POFWARN(status) : FCU_CLEAR_POFWARN(status))

#define FCU_LANDING_STEP(thrust, threshold, step, status) \
  ((thrust) >= (threshold) ? ((thrust) -= (step)) : FCU_CLEAR_ACTIVE(status))

#define FCU_UPDATE_GAIN(gain_array, axis, gain, value, min, max) \
  (gain_array[(axis) % PID_DEPTH][(gain) % PID_DEPTH] = constrain((value), (min), (max)))

#define FCU_UPDATE_SETPOINT(setpoint_array, axis, value, min, max) \
  (setpoint_array[(axis) % PID_DEPTH] = constrain((value), (min), (max)))

#define FCU_UPDATE_THRUST(status, thrust, value, min, max) \
  (thrust = constrain(FCU_IS_POFWARN(status) ? ((value) < (thrust) ? (value) : (thrust)) : (value), (min), (max)))

#define FLASH_SCK_PIN 3
#define FLASH_MOSI_PIN 4
#define FLASH_MISO_PIN 5
#define FLASH_CS_PIN 26
#define FLASH_WREN_CMD 0x06
#define FLASH_WRDI_CMD 0x04
#define FLASH_READ_CMD 0x0B
#define FLASH_WRITE_CMD 0x02
#define FLASH_RDSR_CMD 0x05
#define FLASH_SE_CMD 0x20
#define FLASH_RDID_CMD 0x9F
#define FLASH_FCU_ADDR 0x000000
#define FLASH_CS_LOW() (NRF_P0->OUTCLR = (1UL << FLASH_CS_PIN))
#define FLASH_CS_HIGH() (NRF_P0->OUTSET = (1UL << FLASH_CS_PIN))

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
  float pid_gain[PID_DEPTH][PID_DEPTH];
  float pid_setpoint[PID_DEPTH];
  float pressure;
  float humidity;
  float temperature;
  float battery;
  uint16_t thrust;
  uint16_t distance;
  uint8_t status;
  uint8_t reserved[183];
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
  float integral, output, prev;
} Pid;

static_assert(sizeof(Pid) == 12, "Pid struct must be 12 bytes (3 words)");

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
  float pid_gain[PID_DEPTH][PID_DEPTH];
  uint32_t reserved[53];
} Flash;

static_assert(sizeof(Flash) == 248, "Flash struct must be 248 bytes (62 words)");

// Global utility instances
static Fcu fcu = {0};
static Esc esc = {0x8000, 0x8000, 0x8000, 0x8000};
static volatile Rcu received_packet = {0};
static Pid pid_state[3] = {0};
static Flash flash = {0};

// Task function prototypes
static inline void task_imu_update(void);
static inline void task_fcu_update(void);
static inline void task_esc_update(void);
static inline void task_tof_update(void);
static inline void task_tel_update(void);
static inline void task_pof_update(void);

// Scheduler task array
static Task tasks[SCHEDULER_TASK_COUNT] = {
    {"IMU", task_imu_update, HZ_TO_US(401), 0},
    {"FCU", task_fcu_update, HZ_TO_US(211), 0},
    {"ESC", task_esc_update, HZ_TO_US(101), 0},
    {"TOF", task_tof_update, HZ_TO_US(31), 0},
    {"TEL", task_tel_update, HZ_TO_US(3), 0},
    {"POF", task_pof_update, HZ_TO_US(2), 0}};

// Handler function prototypes
static inline void handle_pid_update(void);
static inline void handle_setpoint_update(void);
static inline void handle_thrust_update(void);
static inline void handle_flash_update(void);

static void (*const handle_type[PACKET_TYPE_COUNT])(void) = {
    handle_pid_update,
    handle_setpoint_update,
    handle_thrust_update,
    handle_flash_update,
};

// Utility function prototypes
static inline void pid_calculate(const float sp, const float pv, const float Kp, const float Ki,
                                 const float Kd, float *I, float *_pv, float *out);
static inline void quaternion_multiply(DataQuaternion *result, const DataQuaternion *q1, const DataQuaternion *q2);
static inline void quaternion_normalize(DataQuaternion *q);
static inline void flash_read(void);

static void flash_spim_init(void);
static uint8_t flash_read_status(void);
static bool flash_write_enable(void);
static bool flash_wait_ready(void);
static inline bool flash_erase(const uint32_t addr);
static inline bool flash_write(void);

#endif // MAIN_H