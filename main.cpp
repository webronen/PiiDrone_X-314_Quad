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

  uint8_t pmic_status = nicla::_pmic.readByte(BQ25120A_ADDRESS, BQ25120A_ILIM_UVLO_CTRL);
  pmic_status = (pmic_status & ~0x3F) | 0x3F; // Set ILIM to 350mA (Default ILIM 50mA) and disable UVLO (Default UVLO 3.0V)
  nicla::_pmic.writeByte(BQ25120A_ADDRESS, BQ25120A_ILIM_UVLO_CTRL, pmic_status);

  sensortec.begin();

  accelerometer.begin(ACCELEROMETER_HZ, ACCELEROMETER_LATENCY);
  accelerometer.setRange(ACCELEROMETER_RANGE);
  gyroscope.begin(GYROSCOPE_HZ, GYROSCOPE_LATENCY);
  gyroscope.setRange(GYROSCOPE_RANGE);

  magnetometer.begin(0, 0);
  magnetometer.setRange(MAGNETOMETER_RANGE);

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

  // Load PID gains from flash
  memcpy(fcu.pid_gain, (const void *)NRF_UICR->CUSTOMER, sizeof(fcu.pid_gain));
}

void loop(void)
{
  // Capture current timer value and store in time_us
  NRF_TIMER0->TASKS_CAPTURE[0] = 1;
  const uint32_t time_us = NRF_TIMER0->CC[0];

  static uint32_t packet_time_us = time_us;
  static uint32_t landing_time_us = time_us;

  // Check if a packet has been received
  if (NRF_RADIO->EVENTS_CRCOK)
  {
    NRF_RADIO->EVENTS_CRCOK = 0;
    packet_time_us = time_us + HZ_TO_US(0.1f);
    read_rcu();
  }

  // Run periodic (scheduled) tasks
  run_scheduler_tasks(time_us);

  /**
   * Automatic landing sequence
   *
   * If FCU is active, and no new RCU packet has been received for 10 seconds, initiate landing sequence
   * by gradually reducing thrust to zero at a rate of 10 units per second. If a new packet is received same
   * time, landing sequence is aborted. When thrust reaches zero, FCU active bit is cleared.
   */
  if ((fcu.status & 0x01) &&
      time_us >= packet_time_us &&
      time_us >= landing_time_us)
  {
    landing_time_us = time_us + HZ_TO_US(1);
    memset(fcu.pid_setpoint, 0, sizeof(fcu.pid_setpoint));

    // If thrust is greater than or equal to 10 units, decrease it by 10 units, else clear the active bit
    (fcu.thrust >= 10) ? (fcu.thrust -= 10) : (fcu.status &= ~0x01);
  }
}

static inline void run_scheduler_tasks(const uint32_t time_us)
{
  for (uint8_t i = 0; i < SCHEDULER_TASK_COUNT; i++)
  {
    if (time_us >= tasks[i].previous_us)
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

static inline void task_update_imu(void)
{
  // Update BHY2 sensor data
  sensortec.update();
}

static inline void task_update_fcu(void)
{
  fcu.pressure = EMA_ALPHA * pressure._value + EMA_BETA * fcu.pressure;
  const float _temperature = temperature._value + TEMPERATURE_OFFSET;
  fcu.temperature = EMA_ALPHA * _temperature + EMA_BETA * fcu.temperature;
  fcu.humidity = EMA_ALPHA * humidity._value + EMA_BETA * fcu.humidity;

  const DataQuaternion conjugate = {-quaternion._data.x, -quaternion._data.y,
                                    -quaternion._data.z, quaternion._data.w};
  DataQuaternion error;
  multiply_quaternion(error, hover_quaternion, conjugate);

  update_pid(fcu.pid_setpoint[0], error.x, fcu.pid_gain[0][0], fcu.pid_gain[0][1], fcu.pid_gain[0][2],
             &pid_state[0].integral, &pid_state[0].prev, &pid_state[0].output);

  update_pid(fcu.pid_setpoint[1], error.y, fcu.pid_gain[1][0], fcu.pid_gain[1][1], fcu.pid_gain[1][2],
             &pid_state[1].integral, &pid_state[1].prev, &pid_state[1].output);

  update_pid(fcu.pid_setpoint[2], error.z, fcu.pid_gain[2][0], fcu.pid_gain[2][1], fcu.pid_gain[2][2],
             &pid_state[2].integral, &pid_state[2].prev, &pid_state[2].output);
}

static inline void task_update_esc(void)
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

static inline void task_update_tof(void)
{
  uint8_t ready = 0;
  if (vl53l4cx.VL53L4CX_GetMeasurementDataReady(&ready) == VL53L4CX_ERROR_NONE && ready)
  {
    VL53L4CX_MultiRangingData_t data;
    if (vl53l4cx.VL53L4CX_GetMultiRangingData(&data) == VL53L4CX_ERROR_NONE &&
        data.NumberOfObjectsFound > 0 &&
        data.RangeData[0].RangeStatus == 0)
    {
      fcu.distance = EMA_ALPHA * data.RangeData[0].RangeMilliMeter + EMA_BETA * fcu.distance;
    }
    vl53l4cx.VL53L4CX_ClearInterruptAndStartMeasurement();
  }
}

static inline void task_send_rcu(void)
{
  // Prepare telemetry packet, copy FCU data to transmit packet
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

static inline void multiply_quaternion(DataQuaternion &r, const DataQuaternion &q1, const DataQuaternion &q2)
{
  r.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
  r.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
  r.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
  r.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;

  normalize_quaternion(r);
}

static inline void normalize_quaternion(DataQuaternion &q)
{
  const float mag = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
  const float inv = 1.0f / __builtin_sqrtf(mag + __FLT_EPSILON__);

  q.w *= inv;
  q.x *= inv;
  q.y *= inv;
  q.z *= inv;
}

static inline void task_handle_pof(void)
{
  // Update battery voltage for telemetry
  fcu.battery = nicla::getCurrentBatteryVoltage();

  // Handle power failure event
  if (NRF_POWER->EVENTS_POFWARN)
  {
    NRF_POWER->EVENTS_POFWARN = 0; // Clear event flag
    static bool led_state = false;
    led_state = !led_state;
    nicla::leds.setColorRed(led_state ? 255 : 0);
    fcu.status |= 0x02; // Set power failure warning bit for telemetry
  }
  else
  {
    nicla::leds.setColorRed(0);
    fcu.status &= ~0x02; // Clear power failure warning bit for telemetry
  }
}

static inline void update_pid(const float setpoint, const float value, const float kp, const float ki,
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

static inline void read_rcu(void)
{
  static void (*const request_table[REQUEST_HANDLER_COUNT])(void) = {
      handle_pid_request,
      handle_setpoint_request,
      handle_thrust_request,
      handle_save_request,
  };

  if (received_packet.node == NODE_ID &&
      received_packet.zone == ZONE_ID &&
      received_packet.type < REQUEST_HANDLER_COUNT)
  {
    request_table[received_packet.type]();
  }
}

static inline void handle_pid_request(void)
{
  const uint8_t axis = received_packet.data[0];
  const uint8_t gain = received_packet.data[1];

  if (axis < 3 && gain < 3)
  {
    float axis_gain = 0.0f;
    memcpy(&axis_gain, (const void *)&received_packet.data[2], sizeof(axis_gain));
    fcu.pid_gain[axis][gain] = constrain(axis_gain, GAIN_MIN, GAIN_MAX);
  }
}

static inline void handle_setpoint_request(void)
{
  const uint8_t axis = received_packet.data[0];

  if (axis < 3)
  {
    float setpoint = 0.0f;
    memcpy(&setpoint, (const void *)&received_packet.data[1], sizeof(setpoint));
    fcu.pid_setpoint[axis] = constrain(setpoint, SETPOINT_MIN, SETPOINT_MAX);
  }
}

static inline void handle_thrust_request(void)
{
  uint16_t thrust = 0;
  memcpy(&thrust, (const void *)&received_packet.data[0], sizeof(thrust));
  fcu.thrust = constrain(thrust, THRUST_MIN, THRUST_MAX);

  // If thrust is greater than THRUST_MIN, set the active bit, else clear it
  (fcu.thrust > THRUST_MIN) ? (fcu.status |= 0x01) : (fcu.status &= ~0x01);
}

static inline void handle_save_request(void)
{
  // Enable erase mode
  NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een;
  while (!NRF_NVMC->READY)
    __NOP(); // Wait until erase mode is set

  // Erase UICR
  NRF_NVMC->ERASEUICR = NVMC_ERASEUICR_ERASEUICR_Erase;
  while (!NRF_NVMC->READY)
    __NOP(); // Wait until erase is complete

  // Enable write mode
  NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
  while (!NRF_NVMC->READY)
    __NOP(); // Wait until write mode is set

  // Write PID gains to flash
  const uint32_t *gain = (const uint32_t *)fcu.pid_gain;
  for (uint8_t i = 0; i < sizeof(fcu.pid_gain) / 4; i++)
  {
    // Write each word to the UICR
    NRF_UICR->CUSTOMER[i] = gain[i];
    while (!NRF_NVMC->READY)
      __NOP(); // Wait until write is complete, before writing the next word
  }

  // Reset the system to apply changes
  NVIC_SystemReset();
}