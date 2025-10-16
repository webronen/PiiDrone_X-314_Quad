#include "main.h"

void setup(void)
{
  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  while (!NRF_CLOCK->EVENTS_HFCLKSTARTED)
    __NOP();

  NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
  NRF_TIMER0->PRESCALER = 4;
  NRF_TIMER0->TASKS_START = 1;

  NRF_P0->PIN_CNF[MOTOR1_PIN] = ((GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                 (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));
  NRF_P0->PIN_CNF[MOTOR2_PIN] = ((GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                 (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));
  NRF_P0->PIN_CNF[MOTOR3_PIN] = ((GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                 (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));
  NRF_P0->PIN_CNF[MOTOR4_PIN] = ((GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                 (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));

  NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_START_Msk);
  NRF_RADIO->PACKETPTR = (uint32_t)&received_packet;
  NRF_RADIO->TXPOWER = RADIO_TXPOWER_TXPOWER_Pos4dBm;

  NRF_RADIO->PCNF1 = (sizeof(Rcu) << RADIO_PCNF1_MAXLEN_Pos) |
                     (sizeof(Rcu) << RADIO_PCNF1_STATLEN_Pos) |
                     (2 << RADIO_PCNF1_BALEN_Pos) |
                     (RADIO_PCNF1_WHITEEN_Enabled << RADIO_PCNF1_WHITEEN_Pos);

  NRF_RADIO->BASE0 = 0x0000BABE;
  NRF_RADIO->PREFIX0 = 0x41 << RADIO_PREFIX0_AP0_Pos;
  NRF_RADIO->RXADDRESSES = RADIO_RXADDRESSES_ADDR0_Msk;

  NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos) |
                      (RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos);

  NRF_RADIO->CRCPOLY = 0x0000AAAA;
  NRF_RADIO->CRCINIT = 0x12345678;

  NRF_RADIO->DATAWHITEIV = 0x55;

  NRF_RADIO->MODECNF0 = (RADIO_MODECNF0_DTX_B0 << RADIO_MODECNF0_DTX_Pos) |
                        (RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos);

  NRF_RADIO->TASKS_RXEN = 1;

  NRF_PWM0->COUNTERTOP = PWM_TOP;
  NRF_PWM0->PRESCALER = PWM_PRESCALER_PRESCALER_DIV_1;
  NRF_PWM0->DECODER = PWM_DECODER_LOAD_Individual;
  NRF_PWM0->SEQ[0].PTR = (uint32_t)&esc;
  NRF_PWM0->SEQ[0].CNT = (sizeof(Esc) / sizeof(uint16_t));
  NRF_PWM0->SEQ[0].REFRESH = PWM_SEQ_REFRESH_CNT_Continuous;
  NRF_PWM0->PSEL.OUT[0] = MOTOR1_PIN;
  NRF_PWM0->PSEL.OUT[1] = MOTOR2_PIN;
  NRF_PWM0->PSEL.OUT[2] = MOTOR3_PIN;
  NRF_PWM0->PSEL.OUT[3] = MOTOR4_PIN;
  NRF_PWM0->ENABLE = PWM_ENABLE_ENABLE_Enabled;
  NRF_PWM0->TASKS_SEQSTART[0] = 1;

  // Set power failure threshold to 2.7V and enable power failure detection for battery monitoring and early warning
  NRF_POWER->POFCON = (POWER_POFCON_THRESHOLD_V27 << POWER_POFCON_THRESHOLD_Pos) |
                      POWER_POFCON_POF_Enabled;

  nicla::begin(false);
  nicla::setBatteryNTCEnabled(false);
  nicla::disableCharging();
  nicla::disableLDO();
  nicla::enable3V3LDO();
  
  // Set ILIM to 350mA (Default 50mA) and disable UVLO (Default 3.0V)
  uint8_t pmic_status = nicla::_pmic.readByte(BQ25120A_ADDRESS, BQ25120A_ILIM_UVLO_CTRL);
  pmic_status = (pmic_status & ~0x3F) | 0x3F;
  nicla::_pmic.writeByte(BQ25120A_ADDRESS, BQ25120A_ILIM_UVLO_CTRL, pmic_status);
  
  sensortec.begin();
  
  accelerometer.begin(ACCELEROMETER_HZ, ACCELEROMETER_LATENCY);
  accelerometer.setRange(ACCELEROMETER_RANGE);
  gyroscope.begin(GYROSCOPE_HZ, GYROSCOPE_LATENCY);
  gyroscope.setRange(GYROSCOPE_RANGE);

  // TODO: Calibrate magnetometer, before using it.
  // magnetometer.begin(MAGNETOMETER_HZ, MAGNETOMETER_LATENCY);
  // magnetometer.setRange(MAGNETOMETER_RANGE);

  quaternion.begin(QUATERNION_HZ, QUATERNION_LATENCY);

  pressure.begin(PRESSURE_HZ, PRESSURE_LATENCY);
  humidity.begin(HUMIDITY_HZ, HUMIDITY_LATENCY);
  temperature.begin(TEMPERATURE_HZ, TEMPERATURE_LATENCY);

  Wire.begin();
  Wire.setClock(400000);

  vl53l4cx.VL53L4CX_SetDeviceAddress(VL53L4CX_DEFAULT_DEVICE_ADDRESS);
  vl53l4cx.VL53L4CX_WaitDeviceBooted();
  vl53l4cx.VL53L4CX_DataInit();
  vl53l4cx.VL53L4CX_SetDistanceMode(VL53L4CX_DISTANCEMODE_MEDIUM);
  vl53l4cx.VL53L4CX_SetMeasurementTimingBudgetMicroSeconds(33000);

  // Set ROI to 4x4 centered
  VL53L4CX_UserRoi_t roi = {
      .TopLeftX = 6,
      .TopLeftY = 6,
      .BotRightX = 9,
      .BotRightY = 9};

  vl53l4cx.VL53L4CX_SetUserROI(&roi);
  vl53l4cx.VL53L4CX_StartMeasurement();

  flash_read();
}

void loop(void)
{
  // Capture current timer value
  NRF_TIMER0->TASKS_CAPTURE[0] = 1;
  const uint32_t loop_time_us = NRF_TIMER0->CC[0];

  // Static variables to track last received packet time and landing rate time
  static uint32_t last_packet_us = loop_time_us;
  static uint32_t landing_rate_us = loop_time_us;

  // Handle received RCU packets
  if (NRF_RADIO->EVENTS_CRCOK)
  {
    NRF_RADIO->EVENTS_CRCOK = 0;
    last_packet_us = loop_time_us + HZ_TO_US(0.1f);
    rcu_read();
  }

  // Run periodic (scheduled) tasks
  scheduler_run(loop_time_us);

  /**
   * Automatic landing sequence
   *
   * If FCU is active, and no new RCU packet has been received for 10 seconds, initiate landing sequence
   * by gradually reducing thrust to zero at a rate of 10 units per second. If a new packet is received same
   * time, landing sequence is aborted. When thrust reaches zero, FCU active bit is cleared.
   */
  if ((fcu.status & 0x01) && loop_time_us >= last_packet_us && loop_time_us >= landing_rate_us)
  {
    landing_rate_us = loop_time_us + HZ_TO_US(1);
    memset(fcu.pid_setpoint, 0, sizeof(fcu.pid_setpoint));
    (fcu.thrust >= 10) ? (fcu.thrust -= 10) : (fcu.status &= ~0x01);
  }
}

static inline void scheduler_run(const uint32_t loop_time_us)
{
  for (uint8_t i = 0; i < SCHEDULER_TASK_COUNT; i++)
  {
    if (loop_time_us >= tasks[i].previous_us)
    {
#ifdef DEBUG
      DEBUG_FUNC_TIME_START();
#endif

      tasks[i].previous_us += tasks[i].interval_us;
      tasks[i].task();

#ifdef DEBUG
      DEBUG_FUNC_TIME_END(tasks[i].name);
#endif
    }
  }
}

static inline void task_imu_update(void)
{
  // Update data on FIFO buffer
  sensortec.update();
}

static inline void task_fcu_update(void)
{
  fcu.pressure = EMA_ALPHA * pressure._value + EMA_BETA * fcu.pressure;
  const float _temperature = temperature._value + TEMPERATURE_OFFSET;
  fcu.temperature = EMA_ALPHA * _temperature + EMA_BETA * fcu.temperature;
  fcu.humidity = EMA_ALPHA * humidity._value + EMA_BETA * fcu.humidity;

  static const DataQuaternion hover_quaternion = {0.0f, 0.0f, 0.0f, 1.0f, 0.0f};
  const DataQuaternion conjugate = {-quaternion._data.x, -quaternion._data.y,
                                    -quaternion._data.z, quaternion._data.w};

  DataQuaternion error;
  quaternion_multiply(&error, &hover_quaternion, &conjugate);

  pid_update(fcu.pid_setpoint[0], error.x, fcu.pid_gain[0][0], fcu.pid_gain[0][1], fcu.pid_gain[0][2],
             &pid_state[0].integral, &pid_state[0].prev, &pid_state[0].output);

  pid_update(fcu.pid_setpoint[1], error.y, fcu.pid_gain[1][0], fcu.pid_gain[1][1], fcu.pid_gain[1][2],
             &pid_state[1].integral, &pid_state[1].prev, &pid_state[1].output);

  pid_update(fcu.pid_setpoint[2], error.z, fcu.pid_gain[2][0], fcu.pid_gain[2][1], fcu.pid_gain[2][2],
             &pid_state[2].integral, &pid_state[2].prev, &pid_state[2].output);
}

static inline void task_esc_update(void)
{
  if (!(fcu.status & 0x01))
  {
    fcu.thrust = 0;
    memset(fcu.pid_setpoint, 0, sizeof(fcu.pid_setpoint));
    memset(pid_state, 0, sizeof(pid_state));
  }

  esc.m1 = 0x8000 | (uint16_t)constrain(fcu.thrust + pid_state[0].output - pid_state[1].output - pid_state[2].output, THRUST_MIN, THRUST_MAX);
  esc.m2 = 0x8000 | (uint16_t)constrain(fcu.thrust - pid_state[0].output - pid_state[1].output + pid_state[2].output, THRUST_MIN, THRUST_MAX);
  esc.m3 = 0x8000 | (uint16_t)constrain(fcu.thrust + pid_state[0].output + pid_state[1].output + pid_state[2].output, THRUST_MIN, THRUST_MAX);
  esc.m4 = 0x8000 | (uint16_t)constrain(fcu.thrust - pid_state[0].output + pid_state[1].output - pid_state[2].output, THRUST_MIN, THRUST_MAX);

  NRF_PWM0->TASKS_SEQSTART[0] = 1;
}

static inline void task_tof_update(void)
{
  static uint8_t ready = 0;
  if (vl53l4cx.VL53L4CX_GetMeasurementDataReady(&ready) == VL53L4CX_ERROR_NONE && ready)
  {
    static VL53L4CX_MultiRangingData_t data;
    if (vl53l4cx.VL53L4CX_GetMultiRangingData(&data) == VL53L4CX_ERROR_NONE &&
        data.NumberOfObjectsFound > 0 &&
        data.RangeData[0].RangeStatus == 0)
    {
      fcu.distance = EMA_ALPHA * data.RangeData[0].RangeMilliMeter + EMA_BETA * fcu.distance;
    }
    vl53l4cx.VL53L4CX_ClearInterruptAndStartMeasurement();
  }
}

static inline void task_tel_update(void)
{
  memcpy(transmit_packet.data, &fcu, sizeof(Fcu));

  while (!NRF_RADIO->EVENTS_END)
    __NOP();

  NRF_RADIO->EVENTS_END = 0;
  NRF_RADIO->TASKS_DISABLE = 1;

  while (NRF_RADIO->STATE)
    __NOP();

  NRF_RADIO->PACKETPTR = (uint32_t)&transmit_packet;
  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk;
  NRF_RADIO->TASKS_TXEN = 1;

  while (NRF_RADIO->STATE)
    __NOP();

  NRF_RADIO->PACKETPTR = (uint32_t)&received_packet;
  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_START_Msk;
  NRF_RADIO->TASKS_RXEN = 1;
}

static inline void task_pof_update(void)
{
  fcu.battery = nicla::getCurrentBatteryVoltage();

  NRF_POWER->EVENTS_POFWARN ? (fcu.status |= 0x02) : (fcu.status &= ~0x02);
  NRF_POWER->EVENTS_POFWARN = 0;
}

static inline void rcu_read(void)
{
  static void (*const handle[PACKET_TYPE_COUNT])(void) = {
      handle_pid_update,
      handle_setpoint_update,
      handle_thrust_update,
      handle_flash_write,
  };

  if (received_packet.node == NODE_ID && received_packet.zone == ZONE_ID)
    handle[received_packet.type % PACKET_TYPE_COUNT]();
}

static inline void pid_update(const float setpoint, const float value, const float kp, const float ki,
                              const float kd, float *integral, float *prev_value, float *output)
{
  const float error = setpoint - value;
  const float derivative = -(value - *prev_value) * PID_LOOP_HZ;
  const float outputNoI = (kp * error) + (kd * derivative);

  *integral += error * PID_LOOP_PERIOD * ((outputNoI <= PID_MAX) && (outputNoI >= PID_MIN));
  const float iLimit = PID_MAX / (ki + __FLT_EPSILON__);
  *integral = constrain(*integral, -iLimit, iLimit);

  *output = outputNoI + (ki * (*integral));
  *output = constrain(*output, PID_MIN, PID_MAX);

  *prev_value = value;
}

static inline void quaternion_multiply(DataQuaternion *r, const DataQuaternion *q1, const DataQuaternion *q2)
{
  r->w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
  r->x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
  r->y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
  r->z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;

  quaternion_normalize(r);
}

static inline void quaternion_normalize(DataQuaternion *q)
{
  const float mag = q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z;
  const float inv = 1.0f / __builtin_sqrtf(mag + __FLT_EPSILON__);

  q->w *= inv;
  q->x *= inv;
  q->y *= inv;
  q->z *= inv;
}

static inline void flash_read(void)
{
  // TODO: Implement loading PID gains from flash memory
  return;
}

static inline void handle_pid_update(void)
{
  const uint8_t axis = received_packet.data[0];
  const uint8_t gain = received_packet.data[1];

  float pid_gain = 0.0f;
  memcpy(&pid_gain, (const void *)&received_packet.data[2], sizeof(pid_gain));
  fcu.pid_gain[axis % PID_DEPTH][gain % PID_DEPTH] = constrain(pid_gain, GAIN_MIN, GAIN_MAX);
}

static inline void handle_setpoint_update(void)
{
  const uint8_t axis = received_packet.data[0];

  float pid_setpoint = 0.0f;
  memcpy(&pid_setpoint, (const void *)&received_packet.data[1], sizeof(pid_setpoint));
  fcu.pid_setpoint[axis % PID_DEPTH] = constrain(pid_setpoint, SETPOINT_MIN, SETPOINT_MAX);
}

static inline void handle_thrust_update(void)
{
  uint16_t thrust = 0;
  memcpy(&thrust, (const void *)&received_packet.data[0], sizeof(thrust));
  fcu.thrust = constrain(thrust, THRUST_MIN, THRUST_MAX);
  (fcu.thrust > THRUST_MIN) ? (fcu.status |= 0x01) : (fcu.status &= ~0x01);
}

static inline void handle_flash_write(void)
{
  // TODO: Implement saving PID gains to flash memory
  return;
}