#include "main.h"

FCU flightControlUnit = {0, 0, 0, 0, 1013.25f, 0.0f, 50.0f, 25.0f, -30};
ESC motorController = {0};
DataQuaternion targetQuaternion = {0.0f, 0.0f, 0.0f, 1.0f};

PID rollPID = INIT_PID(rollPID, ROLL_SETPOINT, KP_ROLL, KI_ROLL, KD_ROLL, WP_ROLL, WI_ROLL, WD_ROLL);
PID pitchPID = INIT_PID(pitchPID, PITCH_SETPOINT, KP_PITCH, KI_PITCH, KD_PITCH, WP_PITCH, WI_PITCH, WD_PITCH);
PID yawPID = INIT_PID(yawPID, YAW_SETPOINT, KP_YAW, KI_YAW, KD_YAW, WP_YAW, WI_YAW, WD_YAW);
PID thrustPID = INIT_PID(thrustPID, THRUST_SETPOINT, KP_THRUST, KI_THRUST, KD_THRUST, WP_THRUST, WI_THRUST, WD_THRUST);
PID altitudePID = INIT_PID(altitudePID, ALTITUDE_SETPOINT, KP_ALTITUDE, KI_ALTITUDE, KD_ALTITUDE, WP_ALTITUDE, WI_ALTITUDE, WD_ALTITUDE);

volatile DataPacket rx_packet;
DataPacket tx_packet;

void setup(void)
{
  // Memory barrier before hardware init
  __DMB();

  // Hardware init
  initialize();

  // Sensor initialization (ordered by priority)
  sensortec.begin();
  quaternion.begin(QUATERNION_HZ, QUATERNION_LATENCY); // Highest priority
  accelerometer.begin(ACCELEROMETER_HZ, ACCELEROMETER_LATENCY);
  accelerometer.setRange(ACCELEROMETER_RANGE);
  gyroscope.begin(GYROSCOPE_HZ, GYROSCOPE_LATENCY);
  gyroscope.setRange(GYROSCOPE_RANGE);

  // Lower priority sensors
  magnetometer.begin(MAGNETOMETER_HZ, MAGNETOMETER_LATENCY);
  pressure.begin(PRESSURE_HZ, PRESSURE_LATENCY);
  humidity.begin(HUMIDITY_HZ, HUMIDITY_LATENCY);
  temperature.begin(TEMPERATURE_HZ, TEMPERATURE_LATENCY);

  // Ensure all operations complete
  __DSB();
}

void loop(void)
{
  NRF_TWI0->TASKS_STARTRX = 1;
  NRF_TIMER0->TASKS_CAPTURE[0] = true;
  const uint32_t loopTime = NRF_TIMER0->CC[0];

  static uint32_t lastMotorUpdateTime = loopTime;
  static uint32_t lastPIDUpdateTime = loopTime;
  static uint32_t lastSensorUpdateTime = loopTime;
  static uint32_t lastDataSendTime = loopTime;

  if (NRF_RADIO->EVENTS_CRCOK)
  {
    NRF_RADIO->EVENTS_CRCOK = 0;
  }

  if (loopTime - lastSensorUpdateTime >= HZ_TO_US(401))
  {
    lastSensorUpdateTime = loopTime;
    sensortec.update();
  }

  if (loopTime - lastPIDUpdateTime >= HZ_TO_US(211))
  {
    lastPIDUpdateTime = loopTime;
    updateFlightControl();
  }

  if (loopTime - lastMotorUpdateTime >= HZ_TO_US(101))
  {
    lastMotorUpdateTime = loopTime;
    updateESC();
  }

  if (loopTime - lastDataSendTime >= HZ_TO_US(100))
  {
    lastDataSendTime = loopTime;

    tx_packet.node = 0x01;
    tx_packet.zone = 0x01;
    tx_packet.type = TYPE_QUATERNION;
    memcpy(tx_packet.data, &quaternion._data, 4 * sizeof(float));
    sendRadioData();

    tx_packet.type = TYPE_PRESSURE;
    memset(tx_packet.data, 0, sizeof(tx_packet.data));
    memcpy(tx_packet.data, &flightControlUnit.pressure, sizeof(float));
    sendRadioData();

    tx_packet.type = TYPE_TEMPERATURE;
    memset(tx_packet.data, 0, sizeof(tx_packet.data));
    memcpy(tx_packet.data, &flightControlUnit.temperature, sizeof(float));
    sendRadioData();

    tx_packet.type = TYPE_ALTITUDE;
    memset(tx_packet.data, 0, sizeof(tx_packet.data));
    memcpy(tx_packet.data, &flightControlUnit.altitude, sizeof(float));
    sendRadioData();

    tx_packet.type = TYPE_HUMIDITY;
    memset(tx_packet.data, 0, sizeof(tx_packet.data));
    memcpy(tx_packet.data, &flightControlUnit.humidity, sizeof(float));
    sendRadioData();
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

  NRF_PWM0->COUNTERTOP = PID_OUTPUT_MAX;
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

  NRF_P0->PIN_CNF[MOTOR1_PIN] = ((GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));
  NRF_P0->PIN_CNF[MOTOR2_PIN] = ((GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));
  NRF_P0->PIN_CNF[MOTOR3_PIN] = ((GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));
  NRF_P0->PIN_CNF[MOTOR4_PIN] = ((GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));
  NRF_P0->OUTCLR = (1 << MOTOR1_PIN) | (1 << MOTOR2_PIN) | (1 << MOTOR3_PIN) | (1 << MOTOR4_PIN);

  NRF_TWI0->PSELSCL = 16;
  NRF_TWI0->PSELSDA = 15;
  NRF_TWI0->FREQUENCY = TWI_FREQUENCY_FREQUENCY_K400;
  NRF_TWI0->ADDRESS = 0x6A;
  NRF_TWI0->ENABLE = TWI_ENABLE_ENABLE_Enabled;
}

static inline void quaternionMultiply(DataQuaternion &r, const DataQuaternion &q1, const DataQuaternion &q2)
{
  r.w = __builtin_fmaf(-q1.x, q2.x, __builtin_fmaf(-q1.y, q2.y, __builtin_fmaf(-q1.z, q2.z, q1.w * q2.w)));
  r.x = __builtin_fmaf(q1.w, q2.x, __builtin_fmaf(q1.x, q2.w, __builtin_fmaf(q1.y, q2.z, -q1.z * q2.y)));
  r.y = __builtin_fmaf(q1.w, q2.y, __builtin_fmaf(-q1.x, q2.z, __builtin_fmaf(q1.y, q2.w, q1.z * q2.x)));
  r.z = __builtin_fmaf(q1.w, q2.z, __builtin_fmaf(q1.x, q2.y, __builtin_fmaf(-q1.y, q2.x, q1.z * q2.w)));

  quaternionNormalize(r);
}

static inline void quaternionNormalize(DataQuaternion &q)
{
  const float norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  const float inv_norm = 1.0f / __builtin_sqrtf(norm_sq + __FLT_EPSILON__);
  q.x *= inv_norm;
  q.y *= inv_norm;
  q.z *= inv_norm;
  q.w *= inv_norm;
}

static inline void setControlInputs(float desiredYaw, float desiredPitch, float desiredRoll, float desiredThrust)
{
  yawPID.setpoint = desiredYaw;
  pitchPID.setpoint = desiredPitch;
  rollPID.setpoint = desiredRoll;
  thrustPID.setpoint = desiredThrust;
}

static inline void updateESC()
{
  const float t = thrustPID.output, r = rollPID.output, p = pitchPID.output, y = yawPID.output;
  motorController.motor1 = (uint16_t)__builtin_fmaxf(0, __builtin_fminf(PID_OUTPUT_MAX, t - r - p + y));
  motorController.motor2 = (uint16_t)__builtin_fmaxf(0, __builtin_fminf(PID_OUTPUT_MAX, t + r - p - y));
  motorController.motor3 = (uint16_t)__builtin_fmaxf(0, __builtin_fminf(PID_OUTPUT_MAX, t + r + p + y));
  motorController.motor4 = (uint16_t)__builtin_fmaxf(0, __builtin_fminf(PID_OUTPUT_MAX, t - r + p - y));

  __DMB();
  NRF_PWM0->TASKS_SEQSTART[0] = 1;
  __DSB();
}

static inline void updatePID(PID &pid, float cv)
{
  const float e = pid.setpoint - cv, ae = __builtin_fabsf(e);
  pid.error = e * (1.0f - __builtin_fminf(1.0f, ae * (1.0f / THRESHOLD_ERROR)));

  const float p = pid.wp * pid.kp * pid.error;
  const float raw_d = cv - pid.lastMeasurement;
  pid.derivative = __builtin_fmaf(LPF, raw_d, HPF * pid.derivative);
  const float d = pid.wd * pid.kd * pid.derivative;

  const bool no_windup = (pid.output < PID_OUTPUT_MAX) && (pid.output > PID_OUTPUT_MIN);
  pid.integral += no_windup ? pid.error : 0.0f;
  const float i = pid.wi * pid.ki * pid.integral;

  pid.output = __builtin_fmaf(p, 1.0f, __builtin_fmaf(i, 1.0f, d));
  pid.lastMeasurement = cv;
}

static inline void updateFlightControl()
{
  flightControlUnit.pressure = __builtin_fmaf(LPF, pressure._value, HPF * flightControlUnit.pressure);
  flightControlUnit.temperature = __builtin_fmaf(LPF, temperature._value - TEMPERATURE_CORRECTION_FACTOR, HPF * flightControlUnit.temperature);
  flightControlUnit.humidity = __builtin_fmaf(LPF, humidity._value, HPF * flightControlUnit.humidity);

  const float pr = flightControlUnit.pressure * INV_SEA_LEVEL_PRESSURE;
  flightControlUnit.altitude = __builtin_fmaf(LPF, BARO_ALTITUDE_CONSTANT * (1.0f - __builtin_powf(pr, BARO_PRESSURE_EXPONENT)), HPF * flightControlUnit.altitude);

  DataQuaternion ci = {-quaternion._data.x, -quaternion._data.y, -quaternion._data.z, quaternion._data.w};
  DataQuaternion ce;
  quaternionMultiply(ce, ci, targetQuaternion);

  updatePID(rollPID, ce.x);
  updatePID(pitchPID, ce.y);
  updatePID(yawPID, ce.z);
  updatePID(altitudePID, flightControlUnit.altitude);

  thrustPID.setpoint = (__builtin_fabsf(thrustPID.setpoint) > THRESHOLD_THRUST) ? thrustPID.setpoint : altitudePID.output;
  updatePID(thrustPID, flightControlUnit.thrust);
  flightControlUnit.thrust = thrustPID.output;
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