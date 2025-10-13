#include "main.h"

void setup(void)
{
  // Start high-frequency clock for peripherals
  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  while (!NRF_CLOCK->EVENTS_HFCLKSTARTED)
    __NOP();

  // Configure 32-bit timer with prescaler
  NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
  NRF_TIMER0->PRESCALER = 4;
  NRF_TIMER0->TASKS_START = 1;

  // Configure motor GPIOs for high-drive output
  NRF_P0->PIN_CNF[MOTOR1_PIN] = ((GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));
  NRF_P0->PIN_CNF[MOTOR2_PIN] = ((GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));
  NRF_P0->PIN_CNF[MOTOR3_PIN] = ((GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));
  NRF_P0->PIN_CNF[MOTOR4_PIN] = ((GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));

  // Configure radio for packet reception and transmission
  NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_START_Msk);
  NRF_RADIO->PACKETPTR = (uint32_t)&received_packet;
  NRF_RADIO->TXPOWER = RADIO_TXPOWER_TXPOWER_Pos4dBm;

  NRF_RADIO->PCNF1 = (sizeof(DataPacket) << RADIO_PCNF1_MAXLEN_Pos) |
                     (sizeof(DataPacket) << RADIO_PCNF1_STATLEN_Pos) |
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

  // Configure PWM for motor control
  NRF_PWM0->COUNTERTOP = PWM_TOP;
  NRF_PWM0->PRESCALER = PWM_PRESCALER_PRESCALER_DIV_1;
  NRF_PWM0->DECODER = PWM_DECODER_LOAD_Individual;
  NRF_PWM0->SEQ[0].PTR = (uint32_t)&esc.m1;
  NRF_PWM0->SEQ[0].CNT = (sizeof(Esc) / sizeof(uint16_t));
  NRF_PWM0->SEQ[0].REFRESH = PWM_SEQ_REFRESH_CNT_Continuous;
  NRF_PWM0->PSEL.OUT[0] = MOTOR1_PIN;
  NRF_PWM0->PSEL.OUT[1] = MOTOR2_PIN;
  NRF_PWM0->PSEL.OUT[2] = MOTOR3_PIN;
  NRF_PWM0->PSEL.OUT[3] = MOTOR4_PIN;
  NRF_PWM0->ENABLE = PWM_ENABLE_ENABLE_Enabled;
  NRF_PWM0->TASKS_SEQSTART[0] = 1;

  // Configure Power-Fail Comparator to 2.7V for early warning of power loss.
  NRF_POWER->POFCON = (POWER_POFCON_THRESHOLD_V27 << POWER_POFCON_THRESHOLD_Pos) | POWER_POFCON_POF_Enabled;

  // Initialize Nicla power and battery management
  nicla::begin(false);
  nicla::setBatteryNTCEnabled(false);
  nicla::disableCharging();
  nicla::disableLDO();
  nicla::enable3V3LDO();

  // Configure PMIC (power management IC) for current and voltage limits
  uint8_t pmic_status = nicla::_pmic.readByte(BQ25120A_ADDRESS, BQ25120A_ILIM_UVLO_CTRL);
  pmic_status = (pmic_status & ~0x3F) | 0x3F; // Set ILIM to 350mA (Max) and disable UVLO (Default is 50mA and UVLO 3.0V enabled)
  nicla::_pmic.writeByte(BQ25120A_ADDRESS, BQ25120A_ILIM_UVLO_CTRL, pmic_status);

  // Initialize IMU and sensors
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

  // Initialize ToF (Time-of-Flight) distance sensor
  Wire.begin();
  Wire.setClock(400000);

  vl53l4cx.VL53L4CX_SetDeviceAddress(VL53L4CX_DEFAULT_DEVICE_ADDRESS);
  vl53l4cx.VL53L4CX_WaitDeviceBooted();
  vl53l4cx.VL53L4CX_DataInit();
  vl53l4cx.VL53L4CX_SetDistanceMode(VL53L4CX_DISTANCEMODE_MEDIUM);
  vl53l4cx.VL53L4CX_SetMeasurementTimingBudgetMicroSeconds(33000);

  VL53L4CX_UserRoi_t roi = {
      .TopLeftX = 6,
      .TopLeftY = 6,
      .BotRightX = 9,
      .BotRightY = 9};

  vl53l4cx.VL53L4CX_SetUserROI(&roi);
  vl53l4cx.VL53L4CX_StartMeasurement();

  // Load configuration/state from flash memory
  load_user_flash();
}

void loop(void)
{
  // Capture current time in microseconds
  NRF_TIMER0->TASKS_CAPTURE[0] = 1;
  const uint32_t global_time_us = NRF_TIMER0->CC[0];

  // Variables for auto landing sequence, if connection to RCU is lost
  static uint32_t packet_time_us = global_time_us;
  static uint32_t landing_time_us = global_time_us;

  // Check for received radio packet and read if available
  if (NRF_RADIO->EVENTS_CRCOK)
  {
    NRF_RADIO->EVENTS_CRCOK = 0;
    packet_time_us = global_time_us + HZ_TO_US(0.1f);
    read_radio_packet();
  }

  // Run simple scheduler for periodic tasks
  run_scheduler_tasks(global_time_us);

  /**
   * Auto landing sequence:
   * If no packet received from RCU for 10 seconds and FCU is active, start landing sequence.
   * Decrease thrust by 10 every second until thrust is less than 10, then set FCU to inactive.
   * If packet is received from RCU anytime during landing sequence, return to normal operation.
   */
  if (fcu.active && global_time_us >= packet_time_us && global_time_us >= landing_time_us)
  {
    landing_time_us = global_time_us + HZ_TO_US(1);
    fcu.roll_setpoint = fcu.pitch_setpoint = fcu.yaw_setpoint = 0.0f;
    (fcu.thrust >= 10) ? (fcu.thrust -= 10) : (fcu.active = false);
  }
}

static inline void run_scheduler_tasks(const uint32_t global_time_us)
{
  // Run all scheduled tasks whose time has come
  for (uint8_t i = 0; i < SCHEDULER_TASK_COUNT; i++)
  {
    if (global_time_us >= tasks[i].previous_us)
    {
#ifdef DEBUG
      DEBUG_FUNC_TIME_START();
#endif // DEBUG

      tasks[i].previous_us += tasks[i].interval_us;
      tasks[i].task();

#ifdef DEBUG
      DEBUG_FUNC_TIME_END(tasks[i].name);
#endif // DEBUG
    }
  }
}

static void update_inertial_measurement_unit(void)
{
  sensortec.update();
}

static void update_flight_control_unit(void)
{
  // Exponential Moving Average (EMA) filter for sensor readings
  fcu.pressure = EMA_ALPHA * pressure._value + EMA_BETA * fcu.pressure;

  const float _temperature = temperature._value + TEMPERATURE_CORRECTION;
  fcu.temperature = EMA_ALPHA * _temperature + EMA_BETA * fcu.temperature;

  fcu.humidity = EMA_ALPHA * humidity._value + EMA_BETA * fcu.humidity;

  // Calculate attitude error quaternion (desired - measured)
  const DataQuaternion conjugate = {-quaternion._data.x, -quaternion._data.y, -quaternion._data.z, quaternion._data.w};
  DataQuaternion error;
  multiply_quaternion(error, hover_quaternion, conjugate);

  // Update PID controllers for roll, pitch, and yaw
  update_pid(fcu.roll_setpoint, error.x, fcu.roll_p, fcu.roll_i, fcu.roll_d, &roll_pid.integral, &roll_pid.prev, &roll_pid.output);
  update_pid(fcu.pitch_setpoint, error.y, fcu.pitch_p, fcu.pitch_i, fcu.pitch_d, &pitch_pid.integral, &pitch_pid.prev, &pitch_pid.output);
  update_pid(fcu.yaw_setpoint, error.z, fcu.yaw_p, fcu.yaw_i, fcu.yaw_d, &yaw_pid.integral, &yaw_pid.prev, &yaw_pid.output);
}

static void update_motor_speed(void)
{
  if (!fcu.active)
  {
    fcu.thrust = 0;
    fcu.roll_setpoint = fcu.pitch_setpoint = fcu.yaw_setpoint = 0.0f;
    memset(&roll_pid, 0, sizeof(Pid));
    memset(&pitch_pid, 0, sizeof(Pid));
    memset(&yaw_pid, 0, sizeof(Pid));
  }

  // Mix PID outputs and thrust to generate PWM for each motor
  esc.m1 = 0x8000 | (uint16_t)constrain(fcu.thrust + roll_pid.output - pitch_pid.output - yaw_pid.output, THRUST_MIN, THRUST_MAX);
  esc.m2 = 0x8000 | (uint16_t)constrain(fcu.thrust - roll_pid.output - pitch_pid.output + yaw_pid.output, THRUST_MIN, THRUST_MAX);
  esc.m3 = 0x8000 | (uint16_t)constrain(fcu.thrust + roll_pid.output + pitch_pid.output + yaw_pid.output, THRUST_MIN, THRUST_MAX);
  esc.m4 = 0x8000 | (uint16_t)constrain(fcu.thrust - roll_pid.output + pitch_pid.output - yaw_pid.output, THRUST_MIN, THRUST_MAX);

  __DMB(); // Data Memory Barrier before starting PWM
  NRF_PWM0->TASKS_SEQSTART[0] = 1;
}

static void update_altitude_sensor(void)
{
  // Read and filter distance from ToF sensor if new data is ready
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

static void send_radio_packet(void)
{
  memcpy(transmit_packet.data, &fcu, sizeof(Fcu));

  // Wait for previous radio transmission to finish
  while (!NRF_RADIO->EVENTS_END)
    __NOP();

  NRF_RADIO->EVENTS_END = 0;
  NRF_RADIO->TASKS_DISABLE = 1;

  // Wait for radio to be fully disabled
  while (NRF_RADIO->STATE)
    __NOP();

  // Set up for TX
  NRF_RADIO->PACKETPTR = (uint32_t)&transmit_packet;
  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk;
  NRF_RADIO->TASKS_TXEN = 1;

  // Wait for TX to finish
  while (NRF_RADIO->STATE)
    __NOP();

  // Restore RX mode
  NRF_RADIO->PACKETPTR = (uint32_t)&received_packet;
  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_START_Msk;
  NRF_RADIO->TASKS_RXEN = 1;
}

static inline void multiply_quaternion(DataQuaternion &r, const DataQuaternion &q1, const DataQuaternion &q2)
{
  // Hamilton product: r = q1 * q2
  // This combines two rotations represented by quaternions q1 and q2.
  r.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
  r.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
  r.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
  r.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;

  // Normalize the resulting quaternion to maintain unit length
  normalize_quaternion(r);
}

static inline void normalize_quaternion(DataQuaternion &q)
{
  // Compute the squared magnitude (norm) of the quaternion
  const float mag = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;

  // Compute the inverse of the magnitude, adding epsilon to avoid division by zero
  const float inv = 1.0f / __builtin_sqrtf(mag + __FLT_EPSILON__);

  // Scale each component to normalize the quaternion (unit length)
  q.w *= inv;
  q.x *= inv;
  q.y *= inv;
  q.z *= inv;
}

static void handle_power_failure(void)
{
  // Deactivate red LED if no power-fail event
  nicla::leds.setColorRed(0);

  // Read and store current battery voltage
  fcu.battery = nicla::getCurrentBatteryVoltage();

  // Read Power-Fail Warning event status
  fcu.power_failure = NRF_POWER->EVENTS_POFWARN;

  // Check if Power-Fail Warning event has occurred
  if (fcu.power_failure)
  {
    // Toggle red led to indicate power failure event at 2 Hz
    static bool led_state = false;
    led_state = !led_state;
    nicla::leds.setColorRed(led_state * 255);
  }

  // Clear the power-fail event flag
  NRF_POWER->EVENTS_POFWARN = 0;
}

static inline void update_pid(const float setpoint, const float value, const float kp, const float ki,
                              const float kd, float *integral, float *prev_value, float *output)
{
  // Standard PID controller with anti-windup, integral auto-scaling, and output limiting
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

static inline void read_radio_packet(void)
{
  // Ignore packets not addressed to this node/zone
  if (received_packet.node == NODE_ID && received_packet.zone == ZONE_ID)
  {
    switch (received_packet.type)
    {
    case TYPE_PID:
      handle_pid_packet();
      break;
    case TYPE_SETPOINT:
      handle_setpoint_packet();
      break;
    case TYPE_THRUST:
      handle_thrust_packet();
      break;
    case TYPE_SAVE:
      fcu.active = false;
      save_user_flash();
      break;
    case TYPE_LOAD:
      load_user_flash();
      break;
    }
  }
}

static inline void handle_pid_packet(void)
{
  const uint8_t axis = received_packet.data[0];
  const uint8_t gain = received_packet.data[1];

  if (axis < 3 && gain < 3)
  {
    float value = 0.0f;
    extract_float_bytes(value, 2);
    value = constrain(value, GAIN_MIN, GAIN_MAX);

    // Update the selected PID gain for the specified axis
    switch (axis)
    {
    case AXIS_PITCH:
      switch (gain)
      {
      case GAIN_KP:
        fcu.pitch_p = value;
        break;
      case GAIN_KI:
        fcu.pitch_i = value;
        break;
      case GAIN_KD:
        fcu.pitch_d = value;
        break;
      }
      break;
    case AXIS_ROLL:
      switch (gain)
      {
      case GAIN_KP:
        fcu.roll_p = value;
        break;
      case GAIN_KI:
        fcu.roll_i = value;
        break;
      case GAIN_KD:
        fcu.roll_d = value;
        break;
      }
      break;
    case AXIS_YAW:
      switch (gain)
      {
      case GAIN_KP:
        fcu.yaw_p = value;
        break;
      case GAIN_KI:
        fcu.yaw_i = value;
        break;
      case GAIN_KD:
        fcu.yaw_d = value;
        break;
      }
      break;
    }
  }
}

static inline void handle_setpoint_packet(void)
{
  const uint8_t axis = received_packet.data[0];

  if (axis < 3)
  {
    float value = 0.0f;
    extract_float_bytes(value, 1);
    value = constrain(value, SETPOINT_MIN, SETPOINT_MAX);

    // Update the setpoint for the specified axis
    switch (axis)
    {
    case AXIS_PITCH:
      fcu.pitch_setpoint = value;
      break;
    case AXIS_ROLL:
      fcu.roll_setpoint = value;
      break;
    case AXIS_YAW:
      fcu.yaw_setpoint = value;
      break;
    }
  }
}

static inline void handle_thrust_packet(void)
{
  // Combine two bytes to form a unsigned 16-bit thrust value using little-endian byte order
  const uint16_t thrust = (received_packet.data[1] << 8) | received_packet.data[0];
  fcu.thrust = constrain(thrust, THRUST_MIN, THRUST_MAX);

  // Activate or deactivate FCU based on thrust
  fcu.active = (fcu.thrust > THRUST_MIN);
}

static inline void extract_float_bytes(float &value, const uint8_t index)
{
  // Extract a float from received_packet.data starting at index using little-endian byte order
  uint8_t *bytes = (uint8_t *)&value;
  bytes[0] = received_packet.data[index];
  bytes[1] = received_packet.data[index + 1];
  bytes[2] = received_packet.data[index + 2];
  bytes[3] = received_packet.data[index + 3];
}

static inline void erase_user_flash(void)
{
  NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een;
  while (!NRF_NVMC->READY)
    __NOP();

  NRF_NVMC->ERASEUICR = NVMC_ERASEUICR_ERASEUICR_Erase;
  while (!NRF_NVMC->READY)
    __NOP();
}

static inline void save_user_flash(void)
{
  erase_user_flash();

  NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
  while (!NRF_NVMC->READY)
    __NOP();

  // Write FCU struct to flash word by word
  const uint32_t *data = (const uint32_t *)&fcu;
  for (uint8_t i = 0; i < FLASH_BLOCK_WORDS; i++)
  {
    NRF_UICR->CUSTOMER[i] = data[i];
    while (!NRF_NVMC->READY)
      __NOP();
  }

  NVIC_SystemReset(); // Reset to reload configuration
}

static inline void load_user_flash(void)
{
  // Load FCU struct from flash
  uint32_t *data = (uint32_t *)&fcu;
  for (uint8_t i = 0; i < FLASH_BLOCK_WORDS; i++)
    data[i] = NRF_UICR->CUSTOMER[i];
}