#include "main.h"

FCU flightControlUnit = {0, 0, 0, 0, 1013.25f, 0.0f, 50.0f, 25.0f, -30};
ESC motorController = {0x8000, 0x8000, 0x8000, 0x8000};
DataQuaternion targetQuaternion = {0.0f, 0.0f, 0.0f, 1.0f};

// Initialize PID controllers
PID rollPID = INIT_PID(ROLL_SETPOINT, KP_ROLL, KI_ROLL, KD_ROLL);
PID pitchPID = INIT_PID(PITCH_SETPOINT, KP_PITCH, KI_PITCH, KD_PITCH);
PID yawPID = INIT_PID(YAW_SETPOINT, KP_YAW, KI_YAW, KD_YAW);

volatile DataPacket rx_packet;
DataPacket tx_packet;

void setup(void)
{
  // Hardware init
  initialize();

  // Sensor initialization
  sensortec.begin();

  // Enable required sensors
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

void loop(void)
{
  NRF_TWI0->TASKS_STARTRX = 1;
  NRF_TIMER0->TASKS_CAPTURE[0] = true;
  const uint32_t loopTime = NRF_TIMER0->CC[0];

  static uint32_t lastMotorUpdateTime = loopTime;
  static uint32_t lastPIDUpdateTime = loopTime;
  static uint32_t lastSensorUpdateTime = loopTime;
  // static uint32_t lastDataSendTime = loopTime;

  // if (NRF_RADIO->EVENTS_CRCOK)
  // {
  //   NRF_RADIO->EVENTS_CRCOK = 0;
  // }

  if (loopTime >= lastSensorUpdateTime)
  {
    lastSensorUpdateTime += HZ_TO_US(401);
    sensortec.update();
  }

  if (loopTime >= lastPIDUpdateTime)
  {
    lastPIDUpdateTime += HZ_TO_US(211);
    setControlInputs(0, 0, 0, 80);
    updateFlightControl();
  }

  if (loopTime >= lastMotorUpdateTime)
  {
    lastMotorUpdateTime += HZ_TO_US(101);
    updateESC();
  }
}

static inline void initialize(void)
{
  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    __WFE();

  NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
  NRF_TIMER0->PRESCALER = 4;
  NRF_TIMER0->TASKS_START = 1;

  NRF_P0->PIN_CNF[MOTOR1_PIN] = ((GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));
  NRF_P0->PIN_CNF[MOTOR2_PIN] = ((GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));
  NRF_P0->PIN_CNF[MOTOR3_PIN] = ((GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));
  NRF_P0->PIN_CNF[MOTOR4_PIN] = ((GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));

  NRF_PWM0->COUNTERTOP = 800;
  NRF_PWM0->PRESCALER = PWM_PRESCALER_PRESCALER_DIV_1;
  NRF_PWM0->DECODER = PWM_DECODER_LOAD_Individual;
  NRF_PWM0->SEQ[0].PTR = (uint32_t)&motorController.motor1;
  NRF_PWM0->SEQ[0].CNT = (sizeof(ESC) / sizeof(uint16_t));
  NRF_PWM0->SEQ[0].REFRESH = PWM_SEQ_REFRESH_CNT_Continuous;
  NRF_PWM0->PSEL.OUT[0] = MOTOR1_PIN;
  NRF_PWM0->PSEL.OUT[1] = MOTOR2_PIN;
  NRF_PWM0->PSEL.OUT[2] = MOTOR3_PIN;
  NRF_PWM0->PSEL.OUT[3] = MOTOR4_PIN;
  NRF_PWM0->ENABLE = PWM_ENABLE_ENABLE_Enabled;

  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_START_Msk;
  NRF_RADIO->PACKETPTR = (uint32_t)&rx_packet;
  NRF_RADIO->FREQUENCY = 0 << RADIO_FREQUENCY_FREQUENCY_Pos;
  NRF_RADIO->TXPOWER = RADIO_TXPOWER_TXPOWER_Pos4dBm << RADIO_TXPOWER_TXPOWER_Pos;
  NRF_RADIO->MODECNF0 = (RADIO_MODECNF0_DTX_B0 << RADIO_MODECNF0_DTX_Pos) | (RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos);
  NRF_RADIO->MODE = RADIO_MODE_MODE_Nrf_2Mbit << RADIO_MODE_MODE_Pos;
  NRF_RADIO->PCNF0 = (0 << RADIO_PCNF0_LFLEN_Pos) | (0 << RADIO_PCNF0_S0LEN_Pos) | (0 << RADIO_PCNF0_S1LEN_Pos) | (RADIO_PCNF0_S1INCL_Automatic << RADIO_PCNF0_S1INCL_Pos) | (RADIO_PCNF0_PLEN_8bit << RADIO_PCNF0_PLEN_Pos);
  NRF_RADIO->PCNF1 = (sizeof(DataPacket) << RADIO_PCNF1_MAXLEN_Pos) | (sizeof(DataPacket) << RADIO_PCNF1_STATLEN_Pos) | (2 << RADIO_PCNF1_BALEN_Pos) | (RADIO_PCNF1_ENDIAN_Little << RADIO_PCNF1_ENDIAN_Pos) | (RADIO_PCNF1_WHITEEN_Enabled << RADIO_PCNF1_WHITEEN_Pos);
  NRF_RADIO->BASE0 = 0x0000BABE;
  NRF_RADIO->PREFIX0 = 0x41 << RADIO_PREFIX0_AP0_Pos;
  NRF_RADIO->RXADDRESSES = RADIO_RXADDRESSES_ADDR0_Enabled << RADIO_RXADDRESSES_ADDR0_Pos;
  NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos) | (RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos);
  NRF_RADIO->CRCPOLY = 0x0000AAAA;
  NRF_RADIO->CRCINIT = 0x12345678;
  NRF_RADIO->DATAWHITEIV = 0x55;
  NRF_RADIO->TASKS_RXEN = 1;

  NRF_TWI0->PSELSCL = 16;
  NRF_TWI0->PSELSDA = 15;
  NRF_TWI0->FREQUENCY = TWI_FREQUENCY_FREQUENCY_K400;
  NRF_TWI0->ADDRESS = 0x6A;
  NRF_TWI0->ENABLE = TWI_ENABLE_ENABLE_Enabled;

  nicla::begin();
  nicla::enable3V3LDO();
  nicla::enableCharging(300);

  Serial.begin(SERIAL_BAUDRATE);
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
  const float mag_sq = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
  const float inv_mag = 1.0f / __builtin_sqrtf(mag_sq + __FLT_EPSILON__);

  q.w *= inv_mag;
  q.x *= inv_mag;
  q.y *= inv_mag;
  q.z *= inv_mag;
}

static inline void setControlInputs(float desiredYaw, float desiredPitch, float desiredRoll, float desiredThrust)
{
  flightControlUnit.thrust = desiredThrust;
  rollPID.setpoint = desiredRoll;
  pitchPID.setpoint = desiredPitch;
  yawPID.setpoint = desiredYaw;
}

static inline void updateESC()
{
  const float thrust = flightControlUnit.thrust;
  const float roll = rollPID.output;
  const float pitch = pitchPID.output;
  const float yaw = yawPID.output;

  // Calculate motor outputs with safety clamping
  const uint16_t m1 = (uint16_t)constrain(thrust + roll - pitch - yaw, 0, 800);
  const uint16_t m2 = (uint16_t)constrain(thrust - roll - pitch + yaw, 0, 800);
  const uint16_t m3 = (uint16_t)constrain(thrust + roll + pitch + yaw, 0, 800);
  const uint16_t m4 = (uint16_t)constrain(thrust - roll + pitch - yaw, 0, 800);

  Serial.print("M1:");
  Serial.print(m1);
  Serial.print(",M2:");
  Serial.print(m2);
  Serial.print(",M3:");
  Serial.print(m3);
  Serial.print(",M4:");
  Serial.println(m4);

  //   // Set motor values with PWM control bit
  //   motorController.motor1 = 0x8000 | m1;  // Front Left, CW
  //   motorController.motor2 = 0x8000 | m2;  // Front Right, CCW
  //   motorController.motor3 = 0x8000 | m3;  // Rear Left, CW
  //   motorController.motor4 = 0x8000 | m4;  // Rear Right, CCW

  //   // Ensure memory operations complete before starting PWM sequence
  //   __DMB();
  //   NRF_PWM0->TASKS_SEQSTART[0] = 1;
  //   __DSB();
}

static inline void updatePID(PID &pid, float measured_value)
{
  static const float dt = 1.0f / 211.0f;

  // Calculate error
  const float error = pid.setpoint - measured_value;
  const float proportional = error;
  const float derivative = (error - pid.previous_error) / dt;

  // Calculate output without integral for anti-windup check
  const float output_no_i = (pid.kp * proportional) + (pid.kd * derivative);

  // Anti-windup: only integrate if output wouldn't saturate
  const bool should_integrate = (output_no_i <= 800) && (output_no_i >= -800);
  pid.integral += error * dt * should_integrate;

  // Calculate final output
  pid.output = output_no_i + (pid.ki * pid.integral);
  pid.output = constrain(pid.output, -800, 800);

  // Store for next iteration
  pid.previous_error = error;
}

static inline void updateFlightControl()
{
  flightControlUnit.pressure = LPF * pressure._value + HPF * flightControlUnit.pressure;
  flightControlUnit.temperature = LPF * (temperature._value - TEMPERATURE_CORRECTION_FACTOR) + HPF * flightControlUnit.temperature;
  flightControlUnit.humidity = LPF * humidity._value + HPF * flightControlUnit.humidity;

  const float pressure_ratio = flightControlUnit.pressure * INV_SEA_LEVEL_PRESSURE;
  const float raw_altitude = BARO_ALTITUDE_CONSTANT * (1.0f - powf(pressure_ratio, BARO_PRESSURE_EXPONENT));
  flightControlUnit.altitude = LPF * raw_altitude + HPF * flightControlUnit.altitude;

  DataQuaternion conjugate = {-quaternion._data.x, -quaternion._data.y, -quaternion._data.z, quaternion._data.w};
  DataQuaternion error;
  quaternionMultiply(error, conjugate, targetQuaternion);

  updatePID(rollPID, error.x);
  updatePID(pitchPID, error.y);
  updatePID(yawPID, error.z);
}

static inline void sendRadioData()
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
