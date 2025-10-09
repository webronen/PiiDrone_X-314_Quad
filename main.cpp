#include "main.h"

Fcu fcu = {
    .thrust = 0,
    .distance = 0,
    .setpoint_pitch = 0.0f,
    .setpoint_roll = 0.0f,
    .setpoint_yaw = 0.0f,
    .pid_pitch_p = 0.0f,
    .pid_pitch_i = 0.0f,
    .pid_pitch_d = 0.0f,
    .pid_roll_p = 0.0f,
    .pid_roll_i = 0.0f,
    .pid_roll_d = 0.0f,
    .pid_yaw_p = 0.0f,
    .pid_yaw_i = 0.0f,
    .pid_yaw_d = 0.0f,
    .roll = 0.0f,
    .pitch = 0.0f,
    .yaw = 0.0f,
    .pressure = 1013.25f,
    .altitude = 0.0f,
    .humidity = 50.0f,
    .temperature = 25.0f,
    .armed = false,
    ._pad = {0}};

Esc esc = {
    .m1 = 0x8000,
    .m2 = 0x8000,
    .m3 = 0x8000,
    .m4 = 0x8000};

FlashBlock flashData = {0};

const DataQuaternion HoverQuaternion = {
    .x = 0.0f,
    .y = 0.0f,
    .z = 0.0f,
    .w = 1.0f};

Pid rollPid = {.setpoint = ROLL_SETPOINT, .kp = KP_ROLL, .ki = KI_ROLL, .kd = KD_ROLL};
Pid pitchPid = {.setpoint = PITCH_SETPOINT, .kp = KP_PITCH, .ki = KI_PITCH, .kd = KD_PITCH};
Pid yawPid = {.setpoint = YAW_SETPOINT, .kp = KP_YAW, .ki = KI_YAW, .kd = KD_YAW};

volatile DataPacket rxPacket;
DataPacket txPacket;

void setup(void)
{
  sysInit();
  rcuInit();
  clkInit();
  pwmInit();
  imuInit();
  tofInit();

  loadFlashData();
  pitchPid.kp = flashData.pitch_kp;
  pitchPid.ki = flashData.pitch_ki;
  pitchPid.kd = flashData.pitch_kd;
  rollPid.kp = flashData.roll_kp;
  rollPid.ki = flashData.roll_ki;
  rollPid.kd = flashData.roll_kd;
  yawPid.kp = flashData.yaw_kp;
  yawPid.ki = flashData.yaw_ki;
  yawPid.kd = flashData.yaw_kd;
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
    parseDataPacket();
  }

  if (loopTime >= lastSensorUpdateTime)
  {
    lastSensorUpdateTime += HZ_TO_US(401);
    sensortec.update();
  }

  if (loopTime >= lastPidUpdateTime)
  {
    lastPidUpdateTime += HZ_TO_US(211);
    updateFlightControl();
  }

  if (loopTime >= lastMotorUpdateTime)
  {
    lastMotorUpdateTime += HZ_TO_US(101);
    updateEsc();
  }

  if (loopTime >= lastPacketSendTime)
  {
    lastPacketSendTime += HZ_TO_US(2);

    txPacket.node = NODE_ID;
    txPacket.zone = ZONE_ID;
    txPacket.type = TYPE_TELEMETRY;

    fcu.pid_pitch_p = pitchPid.kp;
    fcu.pid_pitch_i = pitchPid.ki;
    fcu.pid_pitch_d = pitchPid.kd;
    fcu.pid_roll_p = rollPid.kp;
    fcu.pid_roll_i = rollPid.ki;
    fcu.pid_roll_d = rollPid.kd;
    fcu.pid_yaw_p = yawPid.kp;
    fcu.pid_yaw_i = yawPid.ki;
    fcu.pid_yaw_d = yawPid.kd;

    fcu.setpoint_pitch = pitchPid.setpoint;
    fcu.setpoint_roll = rollPid.setpoint;
    fcu.setpoint_yaw = yawPid.setpoint;

    memcpy(txPacket.data, &fcu, sizeof(Fcu));
    sendDataPacket();
  }

  if (fcu.armed && loopTime >= lastPacketReceiveTime)
  {
    if (loopTime >= startLandingTime)
    {
      startLandingTime = loopTime + HZ_TO_US(1);
      fcu.setpoint_pitch = 0.0f;
      fcu.setpoint_roll = 0.0f;
      fcu.setpoint_yaw = 0.0f;

      if (fcu.thrust >= 10)
        fcu.thrust -= 10;
      else
        fcu.armed = false;
    }
  }

  checkUsbAndCharge();
}

static inline void checkUsbAndCharge()
{
  const uint8_t status = nicla::_pmic.getStatusRegister();
  const bool usbPresent = ((status >> 2) & 0x01);
  const bool chargeDone = (((status >> 6) & 0x03) == 2);

  if (usbPresent && !chargeDone)
    nicla::enableCharging(300);
  else
    nicla::disableCharging();

  if (chargeDone && usbPresent)
    nicla::leds.setColorRed();
  else
    nicla::leds.setColorRed(0);
}

static inline void rcuInit(void)
{
  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  while (!NRF_CLOCK->EVENTS_HFCLKSTARTED)
    __NOP();

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
}

static inline void sendDataPacket(void)
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

static inline void pwmInit(void)
{
  NRF_P0->PIN_CNF[MOTOR1_PIN] = ((GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));
  NRF_P0->PIN_CNF[MOTOR2_PIN] = ((GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));
  NRF_P0->PIN_CNF[MOTOR3_PIN] = ((GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));
  NRF_P0->PIN_CNF[MOTOR4_PIN] = ((GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));

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
}

static inline void clkInit(void)
{
  NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
  NRF_TIMER0->PRESCALER = 4;
  NRF_TIMER0->TASKS_START = 1;
}

static inline void sysInit()
{
  nicla::begin(false);
  nicla::setBatteryNTCEnabled(false);
  nicla::disableCharging();
  nicla::disableLDO();
  nicla::enable3V3LDO();
}

static inline void imuInit(void)
{
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
}

static inline void tofInit(void)
{
  Wire.begin();
  Wire.setClock(400000);

  vl53l4cx.VL53L4CX_SetDeviceAddress(VL53L4CX_ADDR);
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
}

static inline void quaternionMultiply(DataQuaternion &r, const DataQuaternion &q1, const DataQuaternion &q2)
{
  r.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
  r.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
  r.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
  r.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;

  quaternionNormalize(r);
}

static inline void quaternionNormalize(DataQuaternion &q)
{
  const float mag = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
  const float inv = 1.0f / __builtin_sqrtf(mag + __FLT_EPSILON__);

  q.w *= inv;
  q.x *= inv;
  q.y *= inv;
  q.z *= inv;
}

static inline void updateEsc(void)
{
  esc.m1 = 0x8000;
  esc.m2 = 0x8000;
  esc.m3 = 0x8000;
  esc.m4 = 0x8000;

  if (!fcu.armed)
  {
    resetState();
    /*
      This is bad practice today. However, i have always wanted to use this legendary statement.
      I am sorry for using it. Please forgive me. If math shows up, I `goto` somewhere else.

      Math? Nah. I prefer my logic nonlinear and my jumps unconditional.

      "goto - because structured programming is just a suggestion.."
    */
    goto UPDATE_ESC_END;
  }

  esc.m1 |= (uint16_t)constrain(fcu.thrust + rollPid.output - pitchPid.output - yawPid.output, THRUST_MIN, THRUST_MAX);
  esc.m2 |= (uint16_t)constrain(fcu.thrust - rollPid.output - pitchPid.output + yawPid.output, THRUST_MIN, THRUST_MAX);
  esc.m3 |= (uint16_t)constrain(fcu.thrust + rollPid.output + pitchPid.output + yawPid.output, THRUST_MIN, THRUST_MAX);
  esc.m4 |= (uint16_t)constrain(fcu.thrust - rollPid.output + pitchPid.output - yawPid.output, THRUST_MIN, THRUST_MAX);

UPDATE_ESC_END:
  __DMB();
  NRF_PWM0->TASKS_SEQSTART[0] = 1;
}

static inline void resetState(void)
{
  fcu.thrust = 0;
  fcu.roll = 0.0f;
  fcu.pitch = 0.0f;
  fcu.yaw = 0.0f;
  fcu.armed = false;

  rollPid.setpoint = 0.0f;
  pitchPid.setpoint = 0.0f;
  yawPid.setpoint = 0.0f;

  rollPid.integral = 0.0f;
  rollPid.previous_value = 0.0f;
  rollPid.output = 0.0f;

  pitchPid.integral = 0.0f;
  pitchPid.previous_value = 0.0f;
  pitchPid.output = 0.0f;

  yawPid.integral = 0.0f;
  yawPid.previous_value = 0.0f;
  yawPid.output = 0.0f;
}

static inline void updatePid(Pid &pid, const float value)
{
  const float error = pid.setpoint - value;
  const float derivative = -(value - pid.previous_value) * PID_LOOP_HZ;
  const float outputNoI = (pid.kp * error) + (pid.kd * derivative);

  pid.integral += error * PID_LOOP_PERIOD * ((outputNoI <= PID_MAX) && (outputNoI >= PID_MIN));
  const float iLimit = PID_MAX / (pid.ki + __FLT_EPSILON__);
  pid.integral = constrain(pid.integral, -iLimit, iLimit);

  pid.output = outputNoI + (pid.ki * pid.integral);
  pid.output = constrain(pid.output, PID_MIN, PID_MAX);

  pid.previous_value = value;
}

static inline void updateFlightControl(void)
{
  const float newPressure = pressure._value * PA_TO_HPA;
  fcu.pressure = LPF_BARO * newPressure + HPF_BARO * fcu.pressure;

  const float newTemp = (temperature._value - TEMP_OFFSET);
  fcu.temperature = LPF_ENV * newTemp + HPF_ENV * fcu.temperature;
  fcu.humidity = LPF_ENV * humidity._value + HPF_ENV * fcu.humidity;

  const float pressureRatio = fcu.pressure * INV_SEA_LEVEL_PRESSURE;
  fcu.altitude = BARO_ALTITUDE_CONSTANT * (1.0f - __builtin_powf(pressureRatio, BARO_PRESSURE_EXPONENT));

  uint8_t ready = 0;
  if (vl53l4cx.VL53L4CX_GetMeasurementDataReady(&ready) == VL53L4CX_ERROR_NONE && ready)
  {
    VL53L4CX_MultiRangingData_t data;
    if (vl53l4cx.VL53L4CX_GetMultiRangingData(&data) == VL53L4CX_ERROR_NONE &&
        data.NumberOfObjectsFound > 0 && data.RangeData[0].RangeStatus == 0)
    {
      const float newDist = (float)data.RangeData[0].RangeMilliMeter;
      fcu.distance = (uint16_t)(LPF_DISTANCE * newDist + HPF_DISTANCE * fcu.distance);
    }
    vl53l4cx.VL53L4CX_ClearInterruptAndStartMeasurement();
  }

  const DataQuaternion conjugate = {-quaternion._data.x, -quaternion._data.y, -quaternion._data.z, quaternion._data.w};
  DataQuaternion error;
  quaternionMultiply(error, HoverQuaternion, conjugate);

  updatePid(rollPid, error.x);
  updatePid(pitchPid, error.y);
  updatePid(yawPid, error.z);
}

static inline void parseDataPacket(void)
{
  if (rxPacket.node != NODE_ID || rxPacket.zone != ZONE_ID)
  {
    return;
  }

  switch (rxPacket.type)
  {
  case TYPE_PID:
    handlePidPacket();
    break;
  case TYPE_SETPOINT:
    handleSetpointPacket();
    break;
  case TYPE_THRUST:
    handleThrustPacket();
    break;
  case TYPE_SAVE:
    flashData.pitch_kp = pitchPid.kp;
    flashData.pitch_ki = pitchPid.ki;
    flashData.pitch_kd = pitchPid.kd;
    flashData.roll_kp = rollPid.kp;
    flashData.roll_ki = rollPid.ki;
    flashData.roll_kd = rollPid.kd;
    flashData.yaw_kp = yawPid.kp;
    flashData.yaw_ki = yawPid.ki;
    flashData.yaw_kd = yawPid.kd;
    saveFlashData();
    break;
  case TYPE_LOAD:
    loadFlashData();
    pitchPid.kp = flashData.pitch_kp;
    pitchPid.ki = flashData.pitch_ki;
    pitchPid.kd = flashData.pitch_kd;
    rollPid.kp = flashData.roll_kp;
    rollPid.ki = flashData.roll_ki;
    rollPid.kd = flashData.roll_kd;
    yawPid.kp = flashData.yaw_kp;
    yawPid.ki = flashData.yaw_ki;
    yawPid.kd = flashData.yaw_kd;
    break;
  }
}

static inline void handlePidPacket(void)
{
  const uint8_t axis = rxPacket.data[0];
  const uint8_t gain = rxPacket.data[1];

  if (axis >= 3 || gain >= 3)
  {
    return;
  }

  float value = 0.0f;
  extractFloatFromData(value, 2);
  value = constrain(value, GAIN_MIN, GAIN_MAX);

  switch (axis)
  {
  case AXIS_PITCH:
    switch (gain)
    {
    case GAIN_KP:
      pitchPid.kp = value;
      break;
    case GAIN_KI:
      pitchPid.ki = value;
      break;
    case GAIN_KD:
      pitchPid.kd = value;
      break;
    }
    break;
  case AXIS_ROLL:
    switch (gain)
    {
    case GAIN_KP:
      rollPid.kp = value;
      break;
    case GAIN_KI:
      rollPid.ki = value;
      break;
      break;
    case GAIN_KD:
      rollPid.kd = value;
      break;
    }
    break;
  case AXIS_YAW:
    switch (gain)
    {
    case GAIN_KP:
      yawPid.kp = value;
      break;
    case GAIN_KI:
      yawPid.ki = value;
      break;
    case GAIN_KD:
      yawPid.kd = value;
      break;
    }
    break;
  }
}

static inline void handleSetpointPacket(void)
{
  const uint8_t axis = rxPacket.data[0];

  if (axis >= 3)
  {
    return;
  }

  float value = 0.0f;
  extractFloatFromData(value, 1);
  value = constrain(value, SETPOINT_MIN, SETPOINT_MAX);

  switch (axis)
  {
  case AXIS_PITCH:
    pitchPid.setpoint = value;
    break;
  case AXIS_ROLL:
    rollPid.setpoint = value;
    break;
  case AXIS_YAW:
    yawPid.setpoint = value;
    break;
  }
}

static inline void handleThrustPacket(void)
{
  const uint16_t thrust = (rxPacket.data[1] << 8) | rxPacket.data[0];
  fcu.thrust = constrain(thrust, THRUST_MIN, THRUST_MAX);

  if (fcu.thrust > THRUST_MIN)
    fcu.armed = true;
  else
    fcu.armed = false;
}

static inline void extractFloatFromData(float &value, const uint8_t index)
{
  uint8_t *bytes = (uint8_t *)&value;
  bytes[0] = rxPacket.data[index];
  bytes[1] = rxPacket.data[index + 1];
  bytes[2] = rxPacket.data[index + 2];
  bytes[3] = rxPacket.data[index + 3];
}

static inline void eraseFlashData(void)
{
  NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een;
  while (!NRF_NVMC->READY)
    __NOP();

  NRF_NVMC->ERASEUICR = NVMC_ERASEUICR_ERASEUICR_Erase;
  while (!NRF_NVMC->READY)
    __NOP();
}

static inline void saveFlashData(void)
{
  eraseFlashData();

  NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
  while (!NRF_NVMC->READY)
    __NOP();

  uint32_t *data = (uint32_t *)&flashData;
  for (uint8_t i = 0; i < UICR_BLOCK_SIZE; i++)
  {
    NRF_UICR->CUSTOMER[i] = data[i];
    while (!NRF_NVMC->READY)
      __NOP();
  }

  NVIC_SystemReset();
}

static inline void loadFlashData(void)
{
  uint32_t *data = (uint32_t *)&flashData;
  for (uint8_t i = 0; i < UICR_BLOCK_SIZE; i++)
    data[i] = NRF_UICR->CUSTOMER[i];
}