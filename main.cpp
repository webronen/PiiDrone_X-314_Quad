#include "main.h"

FCU fcu = {
    .thrust = 0.0f,       // thrust, range 0 - 800
    .roll = 0.0f,         // 0° roll (level)
    .pitch = 0.0f,        // 0° pitch (level)
    .yaw = 0.0f,          // 0° yaw (north)
    .pressure = 1013.25f, // Sea level pressure reference
    .altitude = 0.0f,     // 0 meters (sea level)
    .humidity = 50.0f,    // 50% RH (typical)
    .temperature = 25.0f  // 25°C (room temperature)
};

ESC esc = {
    .m1 = 0x8000,
    .m2 = 0x8000,
    .m3 = 0x8000,
    .m4 = 0x8000};

const DataQuaternion hoverQuaternion = {
    .x = 0.0f,
    .y = 0.0f,
    .z = 0.0f,
    .w = 1.0f};

PID rollPID = {.setpoint = ROLL_SETPOINT, .kp = KP_ROLL, .ki = KI_ROLL, .kd = KD_ROLL};
PID pitchPID = {.setpoint = PITCH_SETPOINT, .kp = KP_PITCH, .ki = KI_PITCH, .kd = KD_PITCH};
PID yawPID = {.setpoint = YAW_SETPOINT, .kp = KP_YAW, .ki = KI_YAW, .kd = KD_YAW};

volatile DataPacket rx_packet;
DataPacket tx_packet;

void setup(void)
{
  radioInit();
  timerInit();
  pwmInit();
  niclaInit();
  imuInit();

#ifdef DEBUG
  Serial.begin(SERIAL_BAUDRATE);
  while (!Serial)
    __WFE();
#endif
}

void loop(void)
{
  NRF_TIMER0->TASKS_CAPTURE[0] = 1;
  const uint32_t loopTime = NRF_TIMER0->CC[0];

  static uint32_t lastMotorUpdateTime = loopTime;
  static uint32_t lastPidUpdateTime = loopTime;
  static uint32_t lastSensorUpdateTime = loopTime;

  if (NRF_RADIO->EVENTS_CRCOK)
  {
    NRF_RADIO->EVENTS_CRCOK = 0;

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
    updateESC();
  }

  nicla::_pmic.getStatusRegister(); // Reset pmic watchdog
}

static inline void radioInit(void)
{
  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  while (!NRF_CLOCK->EVENTS_HFCLKSTARTED)
    __WFE();

  NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_START_Msk);
  NRF_RADIO->PACKETPTR = (uint32_t)&rx_packet;
  NRF_RADIO->TXPOWER = RADIO_TXPOWER_TXPOWER_Pos4dBm;

  NRF_RADIO->PCNF1 = (sizeof(DataPacket) << RADIO_PCNF1_MAXLEN_Pos) |          // Maximum length of packet payload
                     (sizeof(DataPacket) << RADIO_PCNF1_STATLEN_Pos) |         // Static length in number of bytes
                     (2 << RADIO_PCNF1_BALEN_Pos) |                            // Base address length in number of bytes
                     (RADIO_PCNF1_WHITEEN_Enabled << RADIO_PCNF1_WHITEEN_Pos); // Enable packet whitening

  NRF_RADIO->BASE0 = 0x0000BABE;
  NRF_RADIO->PREFIX0 = 0x41 << RADIO_PREFIX0_AP0_Pos;
  NRF_RADIO->RXADDRESSES = RADIO_RXADDRESSES_ADDR0_Msk;

  NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos) |           // CRC length is two bytes and CRC calculation is enabled
                      (RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos); // CRC calculation does not include address field

  NRF_RADIO->CRCPOLY = 0x0000AAAA;
  NRF_RADIO->CRCINIT = 0x12345678;

  NRF_RADIO->DATAWHITEIV = 0x55;

  NRF_RADIO->MODECNF0 = (RADIO_MODECNF0_DTX_B0 << RADIO_MODECNF0_DTX_Pos) | // Transmit 0 when idle
                        (RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos);  // Fast ramp-up

  NRF_RADIO->TASKS_RXEN = 1;
}

static inline void sendDataPacket(void)
{
  while (!NRF_RADIO->EVENTS_END)
    __WFE();

  NRF_RADIO->EVENTS_END = 0;
  NRF_RADIO->TASKS_DISABLE = 1;

  while (NRF_RADIO->STATE)
    __WFE();

  NRF_RADIO->PACKETPTR = (uint32_t)&tx_packet;
  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk;
  NRF_RADIO->TASKS_TXEN = 1;

  while (NRF_RADIO->STATE)
    __WFE();

  NRF_RADIO->PACKETPTR = (uint32_t)&rx_packet;
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
  NRF_PWM0->SEQ[0].CNT = (sizeof(ESC) / sizeof(uint16_t));
  NRF_PWM0->SEQ[0].REFRESH = PWM_SEQ_REFRESH_CNT_Continuous;
  NRF_PWM0->PSEL.OUT[0] = MOTOR1_PIN;
  NRF_PWM0->PSEL.OUT[1] = MOTOR2_PIN;
  NRF_PWM0->PSEL.OUT[2] = MOTOR3_PIN;
  NRF_PWM0->PSEL.OUT[3] = MOTOR4_PIN;
  NRF_PWM0->ENABLE = PWM_ENABLE_ENABLE_Enabled;
  NRF_PWM0->TASKS_SEQSTART[0] = 1;
}

static inline void timerInit(void)
{
  NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
  NRF_TIMER0->PRESCALER = 4;
  NRF_TIMER0->TASKS_START = 1;
}

static inline void niclaInit(void)
{
  nicla::begin(false);
  nicla::setBatteryNTCEnabled(false);
  nicla::disableLDO();
  nicla::enable1V8LDO();
  nicla::enableCharging(300); // Charging current 300 mA (Max)
}

static inline void imuInit(void)
{
  sensortec.begin();

  accelerometer.begin(ACCELEROMETER_HZ, ACCELEROMETER_LATENCY);
  accelerometer.setRange(ACCELEROMETER_RANGE);
  gyroscope.begin(GYROSCOPE_HZ, GYROSCOPE_LATENCY);
  gyroscope.setRange(GYROSCOPE_RANGE);

  // TODO: Magnetometer calibration, before using in quaternion/rotation vector
  // magnetometer.begin(MAGNETOMETER_HZ, MAGNETOMETER_LATENCY);
  magnetometer.begin(0, 0); // Disable magnetometer

  // Initialize 6 DoF quaternion (Acc + Gyro). 9 DoF (Acc + Gyro + Mag) in future, when magnetometer calibrated
  quaternion.begin(QUATERNION_HZ, QUATERNION_LATENCY);

  pressure.begin(PRESSURE_HZ, PRESSURE_LATENCY);
  humidity.begin(HUMIDITY_HZ, HUMIDITY_LATENCY);
  temperature.begin(TEMPERATURE_HZ, TEMPERATURE_LATENCY);
}

static inline void quaternionMultiply(DataQuaternion &r, const DataQuaternion &q1, const DataQuaternion &q2)
{
  // Hamilton product
  r.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
  r.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
  r.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
  r.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
}

static inline void setControlInputs(const float thrust, const float roll, const float pitch, const float yaw)
{
  fcu.thrust = thrust;
  rollPID.setpoint = roll;
  pitchPID.setpoint = pitch;
  yawPID.setpoint = yaw;
}

static inline void updateESC(void)
{
  const uint16_t m1 = (uint16_t)constrain(fcu.thrust + rollPID.output - pitchPID.output - yawPID.output, 0, PID_MAX);
  const uint16_t m2 = (uint16_t)constrain(fcu.thrust - rollPID.output - pitchPID.output + yawPID.output, 0, PID_MAX);
  const uint16_t m3 = (uint16_t)constrain(fcu.thrust + rollPID.output + pitchPID.output + yawPID.output, 0, PID_MAX);
  const uint16_t m4 = (uint16_t)constrain(fcu.thrust - rollPID.output + pitchPID.output - yawPID.output, 0, PID_MAX);

  Serial.print("M1:");
  Serial.print(m1);
  Serial.print(",M2:");
  Serial.print(m2);
  Serial.print(",M3:");
  Serial.print(m3);
  Serial.print(",M4:");
  Serial.println(m4);

  esc.m1 = 0x8000 | m1; // Front Left, CCW
  esc.m2 = 0x8000 | m2; // Front Right, CW
  esc.m3 = 0x8000 | m3; // Rear Left, CW
  esc.m4 = 0x8000 | m4; // Rear Right, CCW

  NRF_PWM0->TASKS_SEQSTART[0] = 1;
}

static inline void updatePID(PID &pid, const float value)
{
  // 1. Error between desired setpoint and measured value
  const float error = pid.setpoint - value;

  // 2. Derivative term on measurement (avoids derivative kick)
  const float derivative = -(value - pid.previous_value) * PID_LOOP_HZ;

  // 3. Proportional + Derivative output (without integral yet)
  const float output_no_i = (pid.kp * error) + (pid.kd * derivative);

  // 4. Conditional integration (branchless)
  pid.integral += error * PID_LOOP_PERIOD * ((output_no_i <= PID_MAX) && (output_no_i >= PID_MIN));

  // 5. Auto-scaled integrator limit (branchless)
  const float i_limit = PID_MAX / (pid.ki + __FLT_EPSILON__);
  pid.integral = constrain(pid.integral, -i_limit, i_limit);

  // 6. Combine P, I, and D terms
  pid.output = output_no_i + (pid.ki * pid.integral);

  // 7. Clamp final output to actuator limits
  pid.output = constrain(pid.output, PID_MIN, PID_MAX);

  // 8. Store for next iteration
  pid.previous_value = value;
}

static inline void updateFlightControl(void)
{
  fcu.pressure = LPF * pressure._value + HPF * fcu.pressure;
  fcu.temperature = LPF * (temperature._value - TEMPERATURE_CORRECTION_FACTOR) + HPF * fcu.temperature;
  fcu.humidity = LPF * humidity._value + HPF * fcu.humidity;

  const float pressure_ratio = fcu.pressure * INV_SEA_LEVEL_PRESSURE;
  const float raw_altitude = BARO_ALTITUDE_CONSTANT * (1.0f - __builtin_powf(pressure_ratio, BARO_PRESSURE_EXPONENT));
  fcu.altitude = LPF * raw_altitude + HPF * fcu.altitude;

  const DataQuaternion conjugate = {-quaternion._data.x, -quaternion._data.y, -quaternion._data.z, quaternion._data.w};
  DataQuaternion error;

  /*
    In this case, the order of the quaternion multiplication does not matter,
    because hoverQuaternion is a unit quaternion. Multiplicative identity.

    quaternionMultiply(error, hoverQuaternion, conjugate);
    quaternionMultiply(error, conjugate, hoverQuaternion);

    Both are valid.
  */
  quaternionMultiply(error, conjugate, hoverQuaternion);

  updatePID(rollPID, error.x);
  updatePID(pitchPID, error.y);
  updatePID(yawPID, error.z);
}

static inline void parseDataPacket(void)
{
  if (rx_packet.node != NODE_ID || rx_packet.zone != ZONE_ID)
  {
    return;
  }

  switch (rx_packet.type)
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
  }
}

static inline void handlePidPacket(void)
{
  const uint8_t axis = rx_packet.data[0];
  const uint8_t gain = rx_packet.data[1];

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
      pitchPID.kp = value;
      break;
    case GAIN_KI:
      pitchPID.ki = value;
      break;
    case GAIN_KD:
      pitchPID.kd = value;
      break;
    }
    break;

  case AXIS_ROLL:
    switch (gain)
    {
    case GAIN_KP:
      rollPID.kp = value;
      break;
    case GAIN_KI:
      rollPID.ki = value;
      break;
    case GAIN_KD:
      rollPID.kd = value;
      break;
    }
    break;

  case AXIS_YAW:
    switch (gain)
    {
    case GAIN_KP:
      yawPID.kp = value;
      break;
    case GAIN_KI:
      yawPID.ki = value;
      break;
    case GAIN_KD:
      yawPID.kd = value;
      break;
    }
    break;
  }
}

static inline void handleSetpointPacket(void)
{
  const uint8_t axis = rx_packet.data[0];

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
    fcu.pitch = value;
    break;

  case AXIS_ROLL:
    fcu.roll = value;
    break;

  case AXIS_YAW:
    fcu.yaw = value;
    break;
  }
}

static inline void handleThrustPacket(void)
{
  float thrust = 0.0f;
  extractFloatFromData(thrust, 0);

  fcu.thrust = constrain(thrust, THRUST_MIN, PID_MAX);
}

static inline void extractFloatFromData(float &value, const uint8_t index)
{
  uint8_t *bytes = (uint8_t *)&value;

  bytes[0] = rx_packet.data[index];
  bytes[1] = rx_packet.data[index + 1];
  bytes[2] = rx_packet.data[index + 2];
  bytes[3] = rx_packet.data[index + 3];
}
