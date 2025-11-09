#include "main.h"

void setup(void)
{
  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  while (!NRF_CLOCK->EVENTS_HFCLKSTARTED)
    ;

  NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
  NRF_TIMER0->PRESCALER = 4;
  NRF_TIMER0->TASKS_START = 1;

  NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_START_Msk);
  NRF_RADIO->PACKETPTR = (uint32_t)&received_packet;
  NRF_RADIO->TXPOWER = RADIO_TXPOWER_TXPOWER_Pos4dBm;

  NRF_RADIO->PCNF1 = (sizeof(Rcu) << RADIO_PCNF1_MAXLEN_Pos) | (sizeof(Rcu) << RADIO_PCNF1_STATLEN_Pos) |
                     (2 << RADIO_PCNF1_BALEN_Pos) | (RADIO_PCNF1_WHITEEN_Enabled << RADIO_PCNF1_WHITEEN_Pos);

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

  NRF_P0->PIN_CNF[MOTOR1_PIN] = ((GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                 (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));
  NRF_P0->PIN_CNF[MOTOR2_PIN] = ((GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                 (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));
  NRF_P0->PIN_CNF[MOTOR3_PIN] = ((GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                 (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));
  NRF_P0->PIN_CNF[MOTOR4_PIN] = ((GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                 (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));

  NRF_PWM0->COUNTERTOP = MOTOR_MAX;
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

  NRF_POWER->POFCON = (POWER_POFCON_THRESHOLD_V27 << POWER_POFCON_THRESHOLD_Pos) | POWER_POFCON_POF_Enabled;

  nicla::begin(false);
  nicla::setBatteryNTCEnabled(false);
  nicla::disableCharging();
  nicla::disableLDO();
  nicla::enable3V3LDO();
  uint8_t pmic_status = nicla::_pmic.readByte(BQ25120A_ADDRESS, BQ25120A_ILIM_UVLO_CTRL);
  pmic_status = (pmic_status & ~0x3F) | 0x3F;
  nicla::_pmic.writeByte(BQ25120A_ADDRESS, BQ25120A_ILIM_UVLO_CTRL, pmic_status);

  sensortec.begin();
  accelerometer.begin(ACCELEROMETER_HZ, ACCELEROMETER_LATENCY);
  accelerometer.setRange(ACCELEROMETER_RANGE);
  gyroscope.begin(GYROSCOPE_HZ, GYROSCOPE_LATENCY);
  gyroscope.setRange(GYROSCOPE_RANGE);

  magnetometer.begin(MAGNETOMETER_HZ, MAGNETOMETER_LATENCY);
  magnetometer.setRange(MAGNETOMETER_RANGE);
  quaternion.begin(QUATERNION_HZ, QUATERNION_LATENCY);
  pressure.begin(PRESSURE_HZ, PRESSURE_LATENCY);
  humidity.begin(HUMIDITY_HZ, HUMIDITY_LATENCY);
  temperature.begin(TEMPERATURE_HZ, TEMPERATURE_LATENCY);

  Wire.begin();
  Wire.setClock(VL53L4CX_I2C_SPEED);

  vl53l4cx.VL53L4CX_SetDeviceAddress(VL53L4CX_DEFAULT_DEVICE_ADDRESS);
  vl53l4cx.VL53L4CX_WaitDeviceBooted();
  vl53l4cx.VL53L4CX_DataInit();
  vl53l4cx.VL53L4CX_SetDistanceMode(VL53L4CX_DISTANCEMODE_MEDIUM);
  vl53l4cx.VL53L4CX_SetMeasurementTimingBudgetMicroSeconds(33000);
  vl53l4cx.VL53L4CX_SetUserROI(&vl53l4cx_UserRoi);
  vl53l4cx.VL53L4CX_StartMeasurement();
}

void loop(void)
{
  // Capture current timer value for this loop iteration
  NRF_TIMER0->TASKS_CAPTURE[0] = 1;
  const uint32_t loop_start_us = NRF_TIMER0->CC[0];

  // Async packet loss and landing (target = now + interval, Recovery from delays)
  static uint32_t last_packet_us = loop_start_us;
  static uint32_t last_landing_us = loop_start_us;

  // Check if async timeouts occurred
  const bool packet_timeout = loop_start_us >= last_packet_us;
  const bool landing_timeout = loop_start_us >= last_landing_us;

  // Update packet timeout to prevent packet loss landing during auto-tuning
  if (!auto_tune_complete)
    last_packet_us = loop_start_us + HZ_TO_US(0.1f);

  // Handle received radio packets before strict periodic tasks
  if (NRF_RADIO->EVENTS_CRCOK)
  {
    // Clear packet received event, so next packet can be detected
    NRF_RADIO->EVENTS_CRCOK = 0;

    // Update async packet target time
    last_packet_us = loop_start_us + HZ_TO_US(0.1f);

    // Process received packet if addressed to this node and zone
    if (received_packet.node == NODE_ID && received_packet.zone == ZONE_ID)
      handle_type[received_packet.type % PACKET_TYPE_COUNT]();
  }

  // Strict periodic tasks (target += interval, Cannot recover from delays)
  for (uint8_t i = 0; i < SCHEDULER_TASK_COUNT; i++)
  {
    if (loop_start_us >= tasks[i].previous_us)
    {
      // Update strict periodic target time
      tasks[i].previous_us += tasks[i].interval_us;
      tasks[i].task();
    }
  }

  // Async landing step for packet loss or power-fail warning
  if (FCU_IS_ACTIVE(fcu.status) && (packet_timeout || FCU_IS_POFWARN(fcu.status)) && landing_timeout)
  {
    // Update async landing target time
    last_landing_us = loop_start_us + HZ_TO_US(1);
    // Decrement thrust smoothly (units per target time)
    FCU_LANDING_STEP(fcu.thrust, 10, 10, fcu.status);
  }
}

static inline void task_imu_update(void)
{
  sensortec.update();
}

static inline void task_fcu_update(void)
{
  fcu.pressure += (pressure._value - fcu.pressure) * ENV_ALPHA;
  fcu.temperature += ((temperature._value + TEMPERATURE_OFFSET) - fcu.temperature) * ENV_ALPHA;
  fcu.humidity += (humidity._value - fcu.humidity) * ENV_ALPHA;

  static const DataQuaternion hover_quaternion = {0.0f, 0.0f, 0.0f, 1.0f, 0.0f};
  const DataQuaternion conjugate = {-quaternion._data.x, -quaternion._data.y,
                                    -quaternion._data.z, quaternion._data.w};

  DataQuaternion error;
  quaternion_multiply(&error, &hover_quaternion, &conjugate);

  // Ziegler-Nichols Auto-Tune System
  if (!auto_tune_complete)
  {
    if (!thrust_at_hover)
    {
      thrust_at_hover = pid_thrust_to_hover();
      memset(fcu.pid_setpoint, 0, sizeof(fcu.pid_setpoint));
    }
    else
    {
      static float *const error_ptr[3] = {&error.x, &error.y, &error.z};
      const float current_error = *error_ptr[tuning_axis];

      if (pid_auto_tune_step(tuning_axis, current_error))
      {
        tuning_axis++;

        if (tuning_axis >= 3)
        {
          auto_tune_complete = true;
          thrust_at_hover = false;
          tuning_axis = 0;
          memset(fcu.pid_setpoint, 0, sizeof(fcu.pid_setpoint));
          // TODO: store the tuned PID gains to non-volatile memory
        }
      }
    }
  }

  pid_calculate(fcu.pid_setpoint[0], error.x, fcu.pid_gain[0][0], fcu.pid_gain[0][1], fcu.pid_gain[0][2],
                &pid_state[0].I, &pid_state[0].Df, &pid_state[0].pv, &pid_state[0].out);

  pid_calculate(fcu.pid_setpoint[1], error.y, fcu.pid_gain[1][0], fcu.pid_gain[1][1], fcu.pid_gain[1][2],
                &pid_state[1].I, &pid_state[1].Df, &pid_state[1].pv, &pid_state[1].out);

  pid_calculate(fcu.pid_setpoint[2], error.z, fcu.pid_gain[2][0], fcu.pid_gain[2][1], fcu.pid_gain[2][2],
                &pid_state[2].I, &pid_state[2].Df, &pid_state[2].pv, &pid_state[2].out);
}

static inline void task_esc_update(void)
{
  if (!FCU_IS_ACTIVE(fcu.status))
  {
    fcu.thrust = 0;
    memset(fcu.pid_setpoint, 0, sizeof(fcu.pid_setpoint));
    memset(pid_state, 0, sizeof(pid_state));
  }

  const float m1 = fcu.thrust + pid_state[0].out - pid_state[1].out - pid_state[2].out;
  const float m2 = fcu.thrust - pid_state[0].out - pid_state[1].out + pid_state[2].out;
  const float m3 = fcu.thrust + pid_state[0].out + pid_state[1].out + pid_state[2].out;
  const float m4 = fcu.thrust - pid_state[0].out + pid_state[1].out - pid_state[2].out;

  esc.m1 = 0x8000 | (uint16_t)constrain(m1, MOTOR_MIN, MOTOR_MAX);
  esc.m2 = 0x8000 | (uint16_t)constrain(m2, MOTOR_MIN, MOTOR_MAX);
  esc.m3 = 0x8000 | (uint16_t)constrain(m3, MOTOR_MIN, MOTOR_MAX);
  esc.m4 = 0x8000 | (uint16_t)constrain(m4, MOTOR_MIN, MOTOR_MAX);

  NRF_PWM0->TASKS_SEQSTART[0] = 1;
}

static inline void task_tof_update(void)
{
  static uint8_t ready;
  static VL53L4CX_MultiRangingData_t data;

  if (vl53l4cx.VL53L4CX_GetMeasurementDataReady(&ready) == VL53L4CX_ERROR_NONE && ready)
  {
    if (vl53l4cx.VL53L4CX_GetMultiRangingData(&data) == VL53L4CX_ERROR_NONE &&
        data.NumberOfObjectsFound > 0 &&
        data.RangeData[0].RangeStatus == 0)
    {
      fcu.distance += (data.RangeData[0].RangeMilliMeter - fcu.distance) * ENV_ALPHA;
    }
    vl53l4cx.VL53L4CX_ClearInterruptAndStartMeasurement();
  }
}

static inline void task_tel_update(void)
{
  static Rcu transmit_packet = {NODE_ID, ZONE_ID, TYPE_TELEMETRY, {0}};
  memcpy(transmit_packet.data, &fcu, sizeof(Fcu));

  while (!NRF_RADIO->EVENTS_END)
    ;

  NRF_RADIO->EVENTS_END = 0;
  NRF_RADIO->TASKS_DISABLE = 1;

  while (NRF_RADIO->STATE)
    ;

  NRF_RADIO->PACKETPTR = (uint32_t)&transmit_packet;
  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk;
  NRF_RADIO->TASKS_TXEN = 1;

  while (NRF_RADIO->STATE)
    ;

  NRF_RADIO->PACKETPTR = (uint32_t)&received_packet;
  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_START_Msk;
  NRF_RADIO->TASKS_RXEN = 1;
}

static inline void task_pof_update(void)
{
  fcu.battery = nicla::getCurrentBatteryVoltage();
  FCU_UPDATE_POFWARN(fcu.status, NRF_POWER->EVENTS_POFWARN);
  NRF_POWER->EVENTS_POFWARN = 0;
}

static inline void pid_calculate(const float sp, const float pv, const float Kp, const float Ki,
                                 const float Kd, float *_I, float *_D, float *_pv, float *out)
{
  const float P = sp - pv;
  const float D = -(pv - *_pv) * PID_LOOP_HZ;

  *_D += (D - *_D) * D_ALPHA;

  float I = *_I + P * PID_LOOP_PERIOD;
  I = constrain(I, PID_I_MIN, PID_I_MAX);

  *out = Kp * P + Ki * (I) + Kd * (*_D);
  const bool update_integral = (*out > PID_OUT_MIN && *out < PID_OUT_MAX);
  *_I = update_integral ? I : *_I;

  *out = Kp * P + Ki * (*_I) + Kd * (*_D);
  *out = constrain(*out, PID_OUT_MIN, PID_OUT_MAX);
  *_pv = pv;
}

static inline void quaternion_multiply(DataQuaternion *r, const DataQuaternion *q1, const DataQuaternion *q2)
{
  r->x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
  r->y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
  r->z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;
  r->w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;

  quaternion_normalize(r);
}

static inline void quaternion_normalize(DataQuaternion *q)
{
  const float mag = q->x * q->x + q->y * q->y + q->z * q->z + q->w * q->w;
  const float inv = 1.0f / __builtin_sqrtf(mag + __FLT_EPSILON__);

  q->x *= inv;
  q->y *= inv;
  q->z *= inv;
  q->w *= inv;
}

static inline void handle_pid_tune(void)
{
  auto_tune_complete = false;
  thrust_at_hover = false;
  tuning_axis = 0;
  fcu.thrust = 0;

  memset(fcu.pid_setpoint, 0, sizeof(fcu.pid_setpoint));
  memset(pid_state, 0, sizeof(pid_state));

  FCU_UPDATE_ACTIVE(fcu.status, true);
}

static inline void handle_pid_update(void)
{
  const uint8_t axis = received_packet.data[0];
  const uint8_t gain = received_packet.data[1];

  float pid_gain;
  memcpy(&pid_gain, (const void *)&received_packet.data[2], sizeof(pid_gain));

  FCU_UPDATE_GAIN(fcu.pid_gain, axis, gain, pid_gain, PID_GAIN_MIN, PID_GAIN_MAX);
}

static inline void handle_setpoint_update(void)
{
  const uint8_t axis = received_packet.data[0];

  float pid_setpoint;
  memcpy(&pid_setpoint, (const void *)&received_packet.data[1], sizeof(pid_setpoint));

  FCU_UPDATE_SETPOINT(fcu.pid_setpoint, axis, pid_setpoint, SETPOINT_MIN, SETPOINT_MAX);
}

static inline void handle_thrust_update(void)
{
  float thrust;
  memcpy(&thrust, (const void *)&received_packet.data[0], sizeof(thrust));

  if (!auto_tune_complete)
  {
    auto_tune_complete = true;
    thrust_at_hover = false;
    tuning_axis = 0;
  }

  FCU_UPDATE_THRUST(fcu.status, fcu.thrust, thrust, THRUST_MIN, THRUST_MAX);
  FCU_UPDATE_ACTIVE(fcu.status, (fcu.thrust > THRUST_MIN));
}

// Ziegler-Nichols Auto-Tune System

// Clean thrust ramp with smoothstep interpolation
static inline bool pid_thrust_to_hover(void)
{
  static uint32_t start_time = 0;

  if (fcu.thrust == 0)
    start_time = NRF_TIMER0->CC[0];

  const uint32_t elapsed = NRF_TIMER0->CC[0] - start_time;
  float x = (float)elapsed / S_TO_US(PID_THRUST_TO_HOVER_S);
  x = constrain(x, 0.0f, 1.0f);
  const float y = (x * x * (3.0f - 2.0f * x));
  fcu.thrust = constrain(y * 50.0f, THRUST_MIN, THRUST_HOVER); // Assume hover at 50 units for safe initial logic testing

  return (elapsed >= S_TO_US(PID_THRUST_TO_HOVER_S));
}

// Single-phase Ziegler-Nichols auto-tune per axis
static inline bool pid_auto_tune_step(const uint8_t axis, const float err)
{
  if (axis >= 3)
    return true;

  static struct
  {
    uint32_t first_cross, last_change, last_adj;
    uint8_t crosses;
    float setpoint;
    bool active;
  } tune[3] = {0};

  static float last_err[3] = {0};
  const uint32_t now = NRF_TIMER0->CC[0];

  // Initialize tuning for this axis
  if (!tune[axis].active)
  {
    tune[axis].setpoint = 10.0f * DEG_TO_RAD;
    tune[axis].last_change = now + HZ_TO_US(0.5f);
    tune[axis].last_adj = now + HZ_TO_US(2.0f);
    tune[axis].crosses = 0;
    tune[axis].first_cross = 0;
    tune[axis].active = true;

    fcu.pid_setpoint[axis] = tune[axis].setpoint;
    fcu.pid_gain[axis][0] = 0.5f; // Start P
    fcu.pid_gain[axis][1] = 0.0f; // Zero I
    fcu.pid_gain[axis][2] = 0.0f; // Zero D
  }

  // Square wave excitation - 0.5Hz (2 second period)
  if (now >= tune[axis].last_change)
  {
    tune[axis].setpoint = -tune[axis].setpoint;
    fcu.pid_setpoint[axis] = tune[axis].setpoint;
    tune[axis].last_change = now + HZ_TO_US(0.5f);
  }

  // Zero crossing detection with noise threshold
  if (last_err[axis] * err < 0.0f && __builtin_fabsf(err) > 0.01f)
  {
    if (tune[axis].crosses++ == 0)
    {
      tune[axis].first_cross = now;
    }
    else if (tune[axis].crosses >= 4)
    {
      // Calculate Ziegler-Nichols parameters
      const uint32_t total_time = now - tune[axis].first_cross;
      const float num_periods = (tune[axis].crosses - 1) / 2.0f;
      const float Tu = (float)total_time / 1000000.0f / num_periods;
      const float Ku = fcu.pid_gain[axis][0];

      // Apply Ziegler-Nichols formulas with constraints
      fcu.pid_gain[axis][0] = constrain(0.6f * Ku, 0.1f, 8.0f);
      fcu.pid_gain[axis][1] = constrain(1.2f * Ku / Tu, 0.01f, 5.0f);
      fcu.pid_gain[axis][2] = constrain(0.075f * Ku * Tu, 0.001f, 2.0f);

      fcu.pid_setpoint[axis] = 0.0f;
      tune[axis].active = false;
      return true;
    }
  }

  last_err[axis] = err;

  // Increase P gain until oscillation detected
  if (tune[axis].crosses < 4 && now >= tune[axis].last_adj)
  {
    fcu.pid_gain[axis][0] += 0.2f;
    tune[axis].last_adj = now + HZ_TO_US(2.0f);

    // Fallback if no oscillation detected
    if (fcu.pid_gain[axis][0] > 8.0f)
    {
      fcu.pid_gain[axis][0] = 2.0f;
      fcu.pid_gain[axis][1] = 0.5f;
      fcu.pid_gain[axis][2] = 0.1f;
      fcu.pid_setpoint[axis] = 0.0f;
      tune[axis].active = false;
      return true;
    }
  }

  return false;
}