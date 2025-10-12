#include "main.h"

void setup(void)
{
  // CLOCK
  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  while (!NRF_CLOCK->EVENTS_HFCLKSTARTED)
    __NOP();

  // TIMER
  NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
  NRF_TIMER0->PRESCALER = 4;
  NRF_TIMER0->TASKS_START = 1;

  // GPIO
  NRF_P0->PIN_CNF[MOTOR1_PIN] = ((GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));
  NRF_P0->PIN_CNF[MOTOR2_PIN] = ((GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));
  NRF_P0->PIN_CNF[MOTOR3_PIN] = ((GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));
  NRF_P0->PIN_CNF[MOTOR4_PIN] = ((GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));

  // RADIO
  NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_START_Msk);
  NRF_RADIO->PACKETPTR = (uint32_t)&rxPacket;
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

  // PWM
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

  // NICLA
  nicla::begin(false);
  nicla::setBatteryNTCEnabled(false);
  nicla::disableCharging();
  nicla::disableLDO();
  nicla::enable3V3LDO();

  // IMU
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

  // TOF
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

  // FLASH
  loadFlash();
}

void loop(void)
{
  NRF_TIMER0->TASKS_CAPTURE[0] = 1;
  const uint32_t loopTime = NRF_TIMER0->CC[0];

  static uint32_t lastSensorUpdateTime = loopTime;
  static uint32_t lastPidUpdateTime = loopTime;
  static uint32_t lastMotorUpdateTime = loopTime;
  static uint32_t lastPacketSendTime = loopTime;
  static uint32_t lastPacketReceiveTime = loopTime;
  static uint32_t startLandingTime = loopTime;

  if (NRF_RADIO->EVENTS_CRCOK)
  {
    NRF_RADIO->EVENTS_CRCOK = 0;
    lastPacketReceiveTime = loopTime + HZ_TO_US(0.1f);
    readRCU();
  }

  if (loopTime >= lastSensorUpdateTime)
  {
    lastSensorUpdateTime += HZ_TO_US(401);
    sensortec.update();
  }

  if (loopTime >= lastPidUpdateTime)
  {
    lastPidUpdateTime += HZ_TO_US(211);
    updateFCU();
  }

  if (loopTime >= lastMotorUpdateTime)
  {
    lastMotorUpdateTime += HZ_TO_US(101);
    updateESC();
  }

  if (loopTime >= lastPacketSendTime)
  {
    lastPacketSendTime += HZ_TO_US(2);

    memcpy(txPacket.data, &fcu, sizeof(Fcu));
    sendRCU();
  }

  if (fcu.active && loopTime >= lastPacketReceiveTime && loopTime >= startLandingTime)
  {
    startLandingTime = loopTime + HZ_TO_US(1);

    fcu.roll_setpoint = fcu.pitch_setpoint = fcu.yaw_setpoint = 0.0f;

    if (fcu.thrust >= 10)
      fcu.thrust -= 10;
    else
      fcu.active = false;
  }

  fcu.battery = nicla::getCurrentBatteryVoltage();
}

static inline void sendRCU(void)
{
  while (!NRF_RADIO->EVENTS_END)
    __NOP();

  NRF_RADIO->EVENTS_END = 0;
  NRF_RADIO->TASKS_DISABLE = 1;

  while (NRF_RADIO->STATE)
    __NOP();

  NRF_RADIO->PACKETPTR = (uint32_t)&txPacket;
  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk;
  NRF_RADIO->TASKS_TXEN = 1;

  while (NRF_RADIO->STATE)
    __NOP();

  NRF_RADIO->PACKETPTR = (uint32_t)&rxPacket;
  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_START_Msk;
  NRF_RADIO->TASKS_RXEN = 1;
}

static inline void multiplyQuaternion(DataQuaternion &r, const DataQuaternion &q1, const DataQuaternion &q2)
{
  r.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
  r.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
  r.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
  r.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;

  normalizeQuaternion(r);
}

static inline void normalizeQuaternion(DataQuaternion &q)
{
  const float mag = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
  const float inv = 1.0f / __builtin_sqrtf(mag + __FLT_EPSILON__);

  q.w *= inv;
  q.x *= inv;
  q.y *= inv;
  q.z *= inv;
}

static inline void updateESC(void)
{
  if (!fcu.active)
  {
    fcu.thrust = 0;
    fcu.roll_setpoint = fcu.pitch_setpoint = fcu.yaw_setpoint = 0.0f;
    memset(&roll_pid, 0, sizeof(Pid));
    memset(&pitch_pid, 0, sizeof(Pid));
    memset(&yaw_pid, 0, sizeof(Pid));
  }

  esc.m1 = 0x8000 | (uint16_t)constrain(fcu.thrust + roll_pid.output - pitch_pid.output - yaw_pid.output, THRUST_MIN, THRUST_MAX);
  esc.m2 = 0x8000 | (uint16_t)constrain(fcu.thrust - roll_pid.output - pitch_pid.output + yaw_pid.output, THRUST_MIN, THRUST_MAX);
  esc.m3 = 0x8000 | (uint16_t)constrain(fcu.thrust + roll_pid.output + pitch_pid.output + yaw_pid.output, THRUST_MIN, THRUST_MAX);
  esc.m4 = 0x8000 | (uint16_t)constrain(fcu.thrust - roll_pid.output + pitch_pid.output - yaw_pid.output, THRUST_MIN, THRUST_MAX);

  __DMB();
  NRF_PWM0->TASKS_SEQSTART[0] = 1;
}

static inline void updatePID(const float setpoint, const float value, const float kp, const float ki,
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

static inline void updateFCU(void)
{
  fcu.pressure = EMA_ALPHA * pressure._value + EMA_BETA * fcu.pressure;

  const float _temperature = temperature._value + TEMPERATURE_CORRECTION;
  fcu.temperature = EMA_ALPHA * _temperature + EMA_BETA * fcu.temperature;

  fcu.humidity = EMA_ALPHA * humidity._value + EMA_BETA * fcu.humidity;

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

  const DataQuaternion conjugate = {-quaternion._data.x, -quaternion._data.y, -quaternion._data.z, quaternion._data.w};
  DataQuaternion error;
  multiplyQuaternion(error, HoverQuaternion, conjugate);

  updatePID(fcu.roll_setpoint, error.x, fcu.roll_p, fcu.roll_i, fcu.roll_d, &roll_pid.integral, &roll_pid.prev, &roll_pid.output);
  updatePID(fcu.pitch_setpoint, error.y, fcu.pitch_p, fcu.pitch_i, fcu.pitch_d, &pitch_pid.integral, &pitch_pid.prev, &pitch_pid.output);
  updatePID(fcu.yaw_setpoint, error.z, fcu.yaw_p, fcu.yaw_i, fcu.yaw_d, &yaw_pid.integral, &yaw_pid.prev, &yaw_pid.output);
}

static inline void readRCU(void)
{
  if (rxPacket.node != NODE_ID || rxPacket.zone != ZONE_ID)
    return;

  switch (rxPacket.type)
  {
  case TYPE_PID:
    handlePID();
    break;
  case TYPE_SETPOINT:
    handleSetpoint();
    break;
  case TYPE_THRUST:
    handleThrust();
    break;
  case TYPE_SAVE:
    fcu.active = false;
    saveFlash();
    break;
  case TYPE_LOAD:
    loadFlash();
    break;
  }
}

static inline void handlePID(void)
{
  const uint8_t axis = rxPacket.data[0];
  const uint8_t gain = rxPacket.data[1];

  if (axis >= 3 || gain >= 3)
    return;

  float value = 0.0f;
  extractFloat(value, 2);
  value = constrain(value, GAIN_MIN, GAIN_MAX);

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

static inline void handleSetpoint(void)
{
  const uint8_t axis = rxPacket.data[0];

  if (axis >= 3)
    return;

  float value = 0.0f;
  extractFloat(value, 1);
  value = constrain(value, SETPOINT_MIN, SETPOINT_MAX);

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

static inline void handleThrust(void)
{
  const uint16_t thrust = (rxPacket.data[1] << 8) | rxPacket.data[0];
  fcu.thrust = constrain(thrust, THRUST_MIN, THRUST_MAX);

  if (fcu.thrust > THRUST_MIN)
    fcu.active = true;
  else
    fcu.active = false;
}

static inline void extractFloat(float &value, const uint8_t index)
{
  uint8_t *bytes = (uint8_t *)&value;
  bytes[0] = rxPacket.data[index];
  bytes[1] = rxPacket.data[index + 1];
  bytes[2] = rxPacket.data[index + 2];
  bytes[3] = rxPacket.data[index + 3];
}

static inline void eraseFlash(void)
{
  NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een;
  while (!NRF_NVMC->READY)
    __NOP();

  NRF_NVMC->ERASEUICR = NVMC_ERASEUICR_ERASEUICR_Erase;
  while (!NRF_NVMC->READY)
    __NOP();
}

static inline void saveFlash(void)
{
  eraseFlash();

  NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
  while (!NRF_NVMC->READY)
    __NOP();

  for (uint8_t i = 0; i < FLASH_BLOCK_WORDS; i++)
  {
    NRF_UICR->CUSTOMER[i] = ((const uint32_t *)&fcu)[i];
    while (!NRF_NVMC->READY)
      __NOP();
  }

  NVIC_SystemReset();
}

static inline void loadFlash(void)
{
  for (uint8_t i = 0; i < FLASH_BLOCK_WORDS; i++)
    ((uint32_t *)&fcu)[i] = NRF_UICR->CUSTOMER[i];
}