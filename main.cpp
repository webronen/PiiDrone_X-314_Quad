/**
 * FCU-Centered Non-blocking Event Polling Approach:
 *
 * This firmware uses a non-blocking, polling-based architecture where all flight control unit (FCU) state updates,
 * event handling, and safety checks are performed centrally in the main control loop. Instead of relying on
 * interrupt service routines (ISRs), all hardware event flags—such as radio packet reception, power-fail warnings,
 * and sensor updates—are checked and processed sequentially within the loop, without blocking the execution flow.
 *
 * Benefits:
 * - Centralized State Management: All critical state (thrust, setpoints, PID gains, status flags) is updated in one place,
 *   making system behavior predictable and easy to reason about.
 * - Non-blocking: The main loop never waits for events, ensuring all tasks and checks run at high frequency.
 * - Simpler Code Flow: Logic is not fragmented across ISRs and callbacks, reducing complexity and risk of subtle bugs.
 * - No Concurrency Issues: All state changes happen in the main loop, avoiding race conditions and data sharing problems.
 * - Deterministic Timing: The order and timing of all actions are controlled, which is important for real-time flight control.
 * - Easier Debugging and Maintenance: The main loop acts as the “center” of the system, making it straightforward to trace and modify behavior.
 * - Full Control: The main loop can prioritize tasks and events as needed, and all state changes are explicit.
 *
 * This approach is well-suited for high-frequency control loops (such as drones), where the loop runs fast enough
 * to respond to events promptly without missing critical updates, while keeping the codebase robust and maintainable.
 */

#include "main.h"

void setup(void)
{
  // Start High Frequency Clock (32MHz) from external crystal, needed for Radio
  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  while (!NRF_CLOCK->EVENTS_HFCLKSTARTED)
    ;

  // Configure timer for timekeeping, Overflow every 4,294 seconds => 71 minutes => 1.19 hours when in 32-bit mode
  NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
  NRF_TIMER0->PRESCALER = 4; // 1MHz timer frequency (1us ticks)
  NRF_TIMER0->TASKS_START = 1;

  // Configure Radio for receiving RCU packets using Nordic's Proprietary 1Mbps protocol at 2.4GHz
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

  // Configure PWM for ESC control (20kHz frequency, 0-800 duty cycle (0-100%))
  NRF_P0->PIN_CNF[MOTOR1_PIN] = ((GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                 (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));
  NRF_P0->PIN_CNF[MOTOR2_PIN] = ((GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                 (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));
  NRF_P0->PIN_CNF[MOTOR3_PIN] = ((GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                 (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));
  NRF_P0->PIN_CNF[MOTOR4_PIN] = ((GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                 (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));

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

  // Configure Power Failure Comparator to 2.7V threshold
  NRF_POWER->POFCON = (POWER_POFCON_THRESHOLD_V27 << POWER_POFCON_THRESHOLD_Pos) | POWER_POFCON_POF_Enabled;

  nicla::begin(false);
  nicla::setBatteryNTCEnabled(false);
  nicla::disableCharging();
  nicla::disableLDO();
  nicla::enable3V3LDO();

  // Set ILIM to 350mA (Default 50mA) and disable UVLO (Default 3.0V) because we use power failure comparator
  uint8_t pmic_status = nicla::_pmic.readByte(BQ25120A_ADDRESS, BQ25120A_ILIM_UVLO_CTRL);
  pmic_status = (pmic_status & ~0x3F) | 0x3F;
  nicla::_pmic.writeByte(BQ25120A_ADDRESS, BQ25120A_ILIM_UVLO_CTRL, pmic_status);

  sensortec.begin();

  accelerometer.begin(ACCELEROMETER_HZ, ACCELEROMETER_LATENCY);
  accelerometer.setRange(ACCELEROMETER_RANGE);
  gyroscope.begin(GYROSCOPE_HZ, GYROSCOPE_LATENCY);
  gyroscope.setRange(GYROSCOPE_RANGE);

  // TODO: Calibrate magnetometer, before using it.
  magnetometer.begin(MAGNETOMETER_HZ, MAGNETOMETER_LATENCY);
  magnetometer.setRange(MAGNETOMETER_RANGE);

  quaternion.begin(QUATERNION_HZ, QUATERNION_LATENCY);

  pressure.begin(PRESSURE_HZ, PRESSURE_LATENCY);
  humidity.begin(HUMIDITY_HZ, HUMIDITY_LATENCY);
  temperature.begin(TEMPERATURE_HZ, TEMPERATURE_LATENCY);

  // Configure external VL53L4CX Time-of-Flight distance sensor using I2C
  Wire.begin();
  Wire.setClock(VL53L4CX_I2C_SPEED);

  vl53l4cx.VL53L4CX_SetDeviceAddress(VL53L4CX_DEFAULT_DEVICE_ADDRESS);
  vl53l4cx.VL53L4CX_WaitDeviceBooted();
  vl53l4cx.VL53L4CX_DataInit();
  vl53l4cx.VL53L4CX_SetDistanceMode(VL53L4CX_DISTANCEMODE_MEDIUM);
  vl53l4cx.VL53L4CX_SetMeasurementTimingBudgetMicroSeconds(33000);

  // Set ROI to 4x4 centered
  VL53L4CX_UserRoi_t roi = {6, 6, 9, 9};
  vl53l4cx.VL53L4CX_SetUserROI(&roi);
  vl53l4cx.VL53L4CX_StartMeasurement();

  // Read PID gains from MX25R1635F flash memory using SPI
  flash_read();
}

void loop(void)
{
  // Capture current timer value
  NRF_TIMER0->TASKS_CAPTURE[0] = 1;
  const uint32_t loop_start_us = NRF_TIMER0->CC[0];

  // Static variables to manage RCU packet timeout and landing sequence
  static uint32_t last_packet_us = loop_start_us;
  static uint32_t last_landing_us = loop_start_us;

  const bool packet_timeout = (int32_t)(loop_start_us - last_packet_us) >= 0;
  const bool landing_timeout = (int32_t)(loop_start_us - last_landing_us) >= 0;

  // Check for received RCU packet
  if (NRF_RADIO->EVENTS_CRCOK)
  {
    NRF_RADIO->EVENTS_CRCOK = 0;
    last_packet_us = loop_start_us + HZ_TO_US(0.1f);
    rcu_read();
  }

  // Run scheduled tasks
  task_run(loop_start_us);

  /**
   * Automatic landing sequence:
   * - Landing is triggered if no new RCU packet is received for 10 seconds, or if a power-fail warning is active.
   * - While landing, thrust is reduced by 10 units every second until it reaches 10 or less.
   * - When thrust drops to 10 or below, the active status bit is cleared and landing stops.
   * - If a new RCU packet arrives or the power-fail warning clears, landing is aborted and normal flight control resumes.
   * - If power-fail warning is active, landing continues and thrust can only be reduced (not increased).
   * - Directional control remains active during landing for safety.
   */
  if ((FCU_IS_ACTIVE(fcu.status) && (packet_timeout || FCU_IS_POFWARN(fcu.status)) && landing_timeout))
  {
    last_landing_us = loop_start_us + HZ_TO_US(1);
    FCU_LANDING_STEP(fcu.thrust, 10, 10, fcu.status);
  }
}

static inline void task_run(const uint32_t loop_start_us)
{
  for (uint8_t i = 0; i < SCHEDULER_TASK_COUNT; i++)
  {
    if ((int32_t)(loop_start_us - tasks[i].previous_us) >= 0)
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
  sensortec.update();
}

static inline void task_fcu_update(void)
{
  fcu.pressure = EMA_ALPHA * pressure._value + EMA_BETA * fcu.pressure;
  fcu.temperature = EMA_ALPHA * (temperature._value + TEMPERATURE_OFFSET) + EMA_BETA * fcu.temperature;
  fcu.humidity = EMA_ALPHA * humidity._value + EMA_BETA * fcu.humidity;

  static const DataQuaternion hover_quaternion = {0.0f, 0.0f, 0.0f, 1.0f, 0.0f};
  const DataQuaternion conjugate = {-quaternion._data.x, -quaternion._data.y,
                                    -quaternion._data.z, quaternion._data.w};

  DataQuaternion error;
  quaternion_multiply(&error, &hover_quaternion, &conjugate);

  pid_calculate(fcu.pid_setpoint[0], error.x, fcu.pid_gain[0][0], fcu.pid_gain[0][1], fcu.pid_gain[0][2],
                &pid_state[0].integral, &pid_state[0].prev, &pid_state[0].output);

  pid_calculate(fcu.pid_setpoint[1], error.y, fcu.pid_gain[1][0], fcu.pid_gain[1][1], fcu.pid_gain[1][2],
                &pid_state[1].integral, &pid_state[1].prev, &pid_state[1].output);

  pid_calculate(fcu.pid_setpoint[2], error.z, fcu.pid_gain[2][0], fcu.pid_gain[2][1], fcu.pid_gain[2][2],
                &pid_state[2].integral, &pid_state[2].prev, &pid_state[2].output);
}

static inline void task_esc_update(void)
{
  if (!FCU_IS_ACTIVE(fcu.status))
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
  static uint8_t ready;
  static VL53L4CX_MultiRangingData_t data;

  if (vl53l4cx.VL53L4CX_GetMeasurementDataReady(&ready) == VL53L4CX_ERROR_NONE && ready)
  {
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
  // Update battery voltage and power-fail status
  fcu.battery = nicla::getCurrentBatteryVoltage();
  FCU_UPDATE_POFWARN(fcu.status, NRF_POWER->EVENTS_POFWARN);
  NRF_POWER->EVENTS_POFWARN = 0;
}

static inline void rcu_read(void)
{
  static void (*const handle[PACKET_TYPE_COUNT])(void) = {
      handle_pid_update,
      handle_setpoint_update,
      handle_thrust_update,
      handle_flash_update,
  };

  if (received_packet.node == NODE_ID &&
      received_packet.zone == ZONE_ID)
    FCU_HANDLE_PACKET(handle, received_packet.type);
}

static inline void pid_calculate(const float setpoint, const float value, const float kp, const float ki,
                                 const float kd, float *integral, float *prev_value, float *output)
{
  const float error = setpoint - value;
  const float derivative = -(value - *prev_value) * PID_LOOP_HZ;
  const float outputNoI = (kp * error) + (kd * derivative);

  // Only integrate if output is within limits (anti-windup)
  *integral += error * PID_LOOP_PERIOD * ((outputNoI <= PID_MAX) && (outputNoI >= PID_MIN));
  const float iLimit = PID_MAX / (ki + __FLT_EPSILON__);
  *integral = constrain(*integral, -iLimit, iLimit);

  *output = outputNoI + (ki * (*integral));
  *output = constrain(*output, PID_MIN, PID_MAX);

  *prev_value = value;
}

static inline void quaternion_multiply(DataQuaternion *r, const DataQuaternion *q1, const DataQuaternion *q2)
{
  // Hamilton product of two quaternions (r = q1 * q2)
  r->w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
  r->x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
  r->y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
  r->z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;

  quaternion_normalize(r);
}

static inline void quaternion_normalize(DataQuaternion *q)
{
  // Normalize quaternion to unit length and avoid division by zero using epsilon
  const float mag = q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z;
  const float inv = 1.0f / __builtin_sqrtf(mag + __FLT_EPSILON__);

  q->w *= inv;
  q->x *= inv;
  q->y *= inv;
  q->z *= inv;
}

static inline void handle_pid_update(void)
{
  const uint8_t axis = received_packet.data[0];
  const uint8_t gain = received_packet.data[1];

  float pid_gain;
  memcpy(&pid_gain, (const void *)&received_packet.data[2], sizeof(pid_gain));

  FCU_UPDATE_GAIN(fcu.pid_gain, axis, gain, pid_gain, GAIN_MIN, GAIN_MAX);
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
  uint16_t thrust;
  memcpy(&thrust, (const void *)&received_packet.data[0], sizeof(thrust));

  FCU_UPDATE_THRUST(fcu.status, fcu.thrust, thrust, THRUST_MIN, THRUST_MAX);
  FCU_UPDATE_ACTIVE(fcu.status, fcu.thrust > THRUST_MIN);
}

static inline void flash_read(void)
{
  // Read PID gains from MX25R1635F flash memory
}

static inline void handle_flash_update(void)
{
  // Write PID gains to MX25R1635F flash memory
}