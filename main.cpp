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

/**
 * System Initialization (setup):
 * - Initializes clocks, timers, radio, GPIO, PWM, and power-fail warning.
 * - Configures all sensors and peripherals required for flight control.
 * - Loads persisted FCU settings from flash memory.
 * - All hardware and sensor interfaces are set up for non-blocking, event-driven operation.
 */

void setup(void)
{
  // Start high-frequency clock (32MHz) and configure 1MHz timer for timekeeping.
  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  while (!NRF_CLOCK->EVENTS_HFCLKSTARTED)
    ;

  NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
  NRF_TIMER0->PRESCALER = 4; // 1MHz timer frequency (1us ticks)
  NRF_TIMER0->TASKS_START = 1;

  // Configure radio for RCU packet reception using proprietary 1Mbps protocol.
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

  // Set up motor pins and configure PWM for ESC control.
  NRF_P0->PIN_CNF[MOTOR1_PIN] = ((GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                 (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));
  NRF_P0->PIN_CNF[MOTOR2_PIN] = ((GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                 (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));
  NRF_P0->PIN_CNF[MOTOR3_PIN] = ((GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                 (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));
  NRF_P0->PIN_CNF[MOTOR4_PIN] = ((GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                 (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos));
  NRF_PWM0->COUNTERTOP = MOTOR_MAX; // Set PWM period to match MOTOR_MAX for ESCs (0-800 range, 20kHz)
  NRF_PWM0->PRESCALER = PWM_PRESCALER_PRESCALER_DIV_1;
  NRF_PWM0->DECODER = PWM_DECODER_LOAD_Individual;
  NRF_PWM0->SEQ[0].PTR = (uint32_t)&esc;
  NRF_PWM0->SEQ[0].CNT = (sizeof(Esc) / sizeof(uint16_t));
  NRF_PWM0->SEQ[0].REFRESH = PWM_SEQ_REFRESH_CNT_Continuous;
  NRF_PWM0->PSEL.OUT[0] = MOTOR1_PIN; // M1: Front-left (CCW)
  NRF_PWM0->PSEL.OUT[1] = MOTOR2_PIN; // M2: Front-right (CW)
  NRF_PWM0->PSEL.OUT[2] = MOTOR3_PIN; // M3: Rear-left (CW)
  NRF_PWM0->PSEL.OUT[3] = MOTOR4_PIN; // M4: Rear-right (CCW)
  NRF_PWM0->ENABLE = PWM_ENABLE_ENABLE_Enabled;
  NRF_PWM0->TASKS_SEQSTART[0] = 1;

  // Load persisted FCU settings from flash memory. Run this before any other SPI initialization.
  flash_read();

  // Configure power-fail warning and PMIC settings.
  NRF_POWER->POFCON = (POWER_POFCON_THRESHOLD_V27 << POWER_POFCON_THRESHOLD_Pos) | POWER_POFCON_POF_Enabled;
  nicla::begin(false);
  nicla::setBatteryNTCEnabled(false);
  nicla::disableCharging();
  nicla::disableLDO();
  nicla::enable3V3LDO();
  uint8_t pmic_status = nicla::_pmic.readByte(BQ25120A_ADDRESS, BQ25120A_ILIM_UVLO_CTRL);
  pmic_status = (pmic_status & ~0x3F) | 0x3F; // Set to maximum current limit and disable UVLO
  nicla::_pmic.writeByte(BQ25120A_ADDRESS, BQ25120A_ILIM_UVLO_CTRL, pmic_status);

  // Initialize all sensors (IMU, magnetometer, quaternion, pressure, humidity, temperature, ToF).
  sensortec.begin();
  accelerometer.begin(ACCELEROMETER_HZ, ACCELEROMETER_LATENCY);
  accelerometer.setRange(ACCELEROMETER_RANGE);
  gyroscope.begin(GYROSCOPE_HZ, GYROSCOPE_LATENCY);
  gyroscope.setRange(GYROSCOPE_RANGE);

  // TODO: Calibrate magnetometer hard-iron offsets
  magnetometer.begin(MAGNETOMETER_HZ, MAGNETOMETER_LATENCY);
  magnetometer.setRange(MAGNETOMETER_RANGE);
  quaternion.begin(QUATERNION_HZ, QUATERNION_LATENCY);
  pressure.begin(PRESSURE_HZ, PRESSURE_LATENCY);
  humidity.begin(HUMIDITY_HZ, HUMIDITY_LATENCY);
  temperature.begin(TEMPERATURE_HZ, TEMPERATURE_LATENCY);

  // Initialize VL53L4CX Time-of-Flight sensor over I2C.
  Wire.begin();
  Wire.setClock(VL53L4CX_I2C_SPEED);

  vl53l4cx.VL53L4CX_SetDeviceAddress(VL53L4CX_DEFAULT_DEVICE_ADDRESS);
  vl53l4cx.VL53L4CX_WaitDeviceBooted();
  vl53l4cx.VL53L4CX_DataInit();
  vl53l4cx.VL53L4CX_SetDistanceMode(VL53L4CX_DISTANCEMODE_MEDIUM);
  vl53l4cx.VL53L4CX_SetMeasurementTimingBudgetMicroSeconds(33000);

  VL53L4CX_UserRoi_t roi = {6, 6, 9, 9}; // Set ROI to 4x4 centered
  vl53l4cx.VL53L4CX_SetUserROI(&roi);
  vl53l4cx.VL53L4CX_StartMeasurement();
}

/**
 * FCU-Centered Non-blocking Event Polling Loop:
 * - Polls hardware event flags and updates FCU state in the main loop.
 * - Handles RCU packet reception, scheduled tasks, and automatic landing sequence.
 * - Ensures all control logic, safety checks, and state updates are performed centrally.
 * - Maintains deterministic timing and avoids blocking or concurrency issues.
 * - Designed for high-frequency, real-time flight control.
 */

void loop(void)
{
  // Capture current timer value for loop timing.
  NRF_TIMER0->TASKS_CAPTURE[0] = 1;
  const uint32_t loop_start_us = NRF_TIMER0->CC[0];

  // Manage RCU packet timeout and landing sequence timing.
  static uint32_t last_packet_us = loop_start_us;
  static uint32_t last_landing_us = loop_start_us;

  const bool packet_timeout = (int32_t)(loop_start_us - last_packet_us) >= 0;
  const bool landing_timeout = (int32_t)(loop_start_us - last_landing_us) >= 0;

  // Check for received RCU packet and handle it.
  if (NRF_RADIO->EVENTS_CRCOK)
  {
    NRF_RADIO->EVENTS_CRCOK = 0;
    last_packet_us = loop_start_us + HZ_TO_US(0.1f);

    // Only handle packet if it is addressed to this node and zone.
    if (received_packet.node == NODE_ID &&
        received_packet.zone == ZONE_ID)
      handle_type[received_packet.type % PACKET_TYPE_COUNT](); // Round-robin array safety
  }

  // Run all scheduled tasks (IMU, FCU, ESC, telemetry, ToF, power-fail).
  for (uint8_t i = 0; i < SCHEDULER_TASK_COUNT; i++)
  {
    if ((int32_t)(loop_start_us - tasks[i].previous_us) >= 0)
    {
      tasks[i].previous_us += tasks[i].interval_us;
      tasks[i].task();
    }
  }

  /**
   * Automatic landing sequence:
   * - Landing is triggered if no new RCU packet is received for 10 seconds, or if a power-fail warning is active.
   * - While landing, thrust is reduced by 10 units every second until it reaches 10 or less.
   * - When thrust drops to 10 or below, the active status bit is cleared and landing stops.
   * - If a new RCU packet arrives or the power-fail warning clears, landing is aborted and normal flight control resumes.
   * - If power-fail warning is active, landing continues and thrust can only be reduced (not increased).
   * - Directional control remains active during landing for safety.
   */

  // If landing conditions are met, reduce thrust and manage landing state.
  if ((FCU_IS_ACTIVE(fcu.status) && (packet_timeout || FCU_IS_POFWARN(fcu.status)) && landing_timeout))
  {
    last_landing_us = loop_start_us + HZ_TO_US(1);
    FCU_LANDING_STEP(fcu.thrust, 10, 10, fcu.status);
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
  // If FCU is not active, reset thrust and PID states
  if (!FCU_IS_ACTIVE(fcu.status))
  {
    fcu.thrust = 0;
    memset(fcu.pid_setpoint, 0, sizeof(fcu.pid_setpoint));
    memset(pid_state, 0, sizeof(pid_state));
  }

  // Calculate raw motor outputs based on thrust and PID outputs
  const float m1 = fcu.thrust - pid_state[0].output + pid_state[1].output - pid_state[2].output; // M1: Front-right (CCW)
  const float m2 = fcu.thrust + pid_state[0].output + pid_state[1].output + pid_state[2].output; // M2: Front-left (CW)
  const float m3 = fcu.thrust - pid_state[0].output - pid_state[1].output + pid_state[2].output; // M3: Rear-right (CCW)
  const float m4 = fcu.thrust + pid_state[0].output - pid_state[1].output - pid_state[2].output; // M4: Rear-left (CW)

  // Determine only the maximum upper motor output and calculate offset if exceeding MOTOR_MAX.
  const float motor_max = __builtin_fmaxf(__builtin_fmaxf(m1, m2), __builtin_fmaxf(m3, m4));
  const float offset = __builtin_fmax(motor_max - MOTOR_MAX, 0.0f);

  // Apply offset if needed and constrain motor outputs to valid range
  esc.m1 = 0x8000 | (uint16_t)constrain(m1 - offset, MOTOR_MIN, MOTOR_MAX);
  esc.m2 = 0x8000 | (uint16_t)constrain(m2 - offset, MOTOR_MIN, MOTOR_MAX);
  esc.m3 = 0x8000 | (uint16_t)constrain(m3 - offset, MOTOR_MIN, MOTOR_MAX);
  esc.m4 = 0x8000 | (uint16_t)constrain(m4 - offset, MOTOR_MIN, MOTOR_MAX);

  // Start PWM sequence to update motor outputs
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
  fcu.battery = nicla::getCurrentBatteryVoltage();
  FCU_UPDATE_POFWARN(fcu.status, NRF_POWER->EVENTS_POFWARN);
  NRF_POWER->EVENTS_POFWARN = 0;
}

static inline void pid_calculate(const float setpoint, const float value, const float kp, const float ki,
                                 const float kd, float *integral, float *prev_value, float *output)
{
  const float error = setpoint - value;
  const float P = kp * error;

  const float D = kd * (*prev_value - value) * -PID_LOOP_HZ;

  const float i_scaling = constrain(fcu.thrust * MOTOR_MAX_INV, 0.2f, 1.0f);

  const float integral_delta = error * PID_LOOP_PERIOD * i_scaling;
  const float new_integral = *integral + integral_delta;
  const float i_limit = I_TERM_MAX * i_scaling;
  *integral = constrain(new_integral, -i_limit, i_limit);

  const float I = ki * (*integral);
  const float pid_sum = P + I + D;

  *output = constrain(pid_sum, PID_OUT_MIN, PID_OUT_MAX);
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

  FCU_UPDATE_THRUST(fcu.status, fcu.thrust, thrust, MOTOR_MIN, MOTOR_MAX);
  FCU_UPDATE_ACTIVE(fcu.status, fcu.thrust > MOTOR_MIN);
}

static void flash_spim_init(void)
{
  NRF_SPIM0->ENABLE = SPIM_ENABLE_ENABLE_Disabled;
  NRF_P0->PIN_CNF[FLASH_CS_PIN] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
                                  (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                                  (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
                                  (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos);
  NRF_P0->OUTSET = (1UL << FLASH_CS_PIN);
  NRF_SPIM0->PSEL.SCK = FLASH_SCK_PIN;
  NRF_SPIM0->PSEL.MOSI = FLASH_MOSI_PIN;
  NRF_SPIM0->PSEL.MISO = FLASH_MISO_PIN;
  NRF_SPIM0->FREQUENCY = SPIM_FREQUENCY_FREQUENCY_M8;
  NRF_SPIM0->CONFIG = (SPIM_CONFIG_ORDER_MsbFirst << SPIM_CONFIG_ORDER_Pos) |
                      (SPIM_CONFIG_CPHA_Leading << SPIM_CONFIG_CPHA_Pos) |
                      (SPIM_CONFIG_CPOL_ActiveHigh << SPIM_CONFIG_CPOL_Pos);
  NRF_SPIM0->ENABLE = SPIM_ENABLE_ENABLE_Enabled;
}

static inline void handle_flash_update(void)
{
  /**
   * Prepare SPI transaction to write flash memory starting from FLASH_FCU_ADDR
   * 1. Send WRITE ENABLE command
   * 2. Send PAGE PROGRAM command followed by 3-byte (24-bit) address
   * 3. Erase relevant flash sector before writing
   * 4. Wait for write to complete by polling WIP bit in status register
   * 5. Reset system to clear state and reload settings from flash on next boot
   *
   * Note: Writing to flash memory is slow and blocking. This function should be used sparingly,
   * ideally only when settings need to be persisted after significant changes.
   */

  flash_spim_init();
  memcpy(&flash.pid_gain, &fcu.pid_gain, sizeof(fcu.pid_gain));
  flash_erase(FLASH_FCU_ADDR);
  flash_write();

  NRF_SPIM0->ENABLE = SPIM_ENABLE_ENABLE_Disabled;
  NVIC_SystemReset();
}

static inline void flash_read(void)
{
  /**
   * Prepare SPI transaction to read flash memory starting from FLASH_FCU_ADDR
   * 1. Send READ command followed by 3-byte (24-bit) address
   * 2. Read back data into rx_buf
   * 3. Copy rx_buf into flash structure
   * 4. Copy PID gains into FCU structure from flash structure
   * 5. Copy other relevant data as needed
   *
   * Note: The first 5 bytes of rx_buf are command and address bytes
   * we need to skip them to get to the actual data.
   */

  flash_spim_init();

  static uint8_t tx_buf[5] = {
      (uint8_t)FLASH_READ_CMD,
      (uint8_t)((FLASH_FCU_ADDR >> 16) & 0xFF),
      (uint8_t)((FLASH_FCU_ADDR >> 8) & 0xFF),
      (uint8_t)(FLASH_FCU_ADDR & 0xFF),
      0xFF}; // Dummy byte for clocking out data

  static uint8_t rx_buf[sizeof(tx_buf) + sizeof(Flash)] = {0};

  FLASH_CS_LOW();
  nrf_delay_us(1);

  NRF_SPIM0->TXD.PTR = (uint32_t)tx_buf;
  NRF_SPIM0->TXD.MAXCNT = sizeof(tx_buf);
  NRF_SPIM0->RXD.PTR = (uint32_t)rx_buf;
  NRF_SPIM0->RXD.MAXCNT = sizeof(rx_buf);
  NRF_SPIM0->TASKS_START = 1;

  while (!NRF_SPIM0->EVENTS_END)
    ;

  NRF_SPIM0->EVENTS_END = 0;
  FLASH_CS_HIGH();

  memcpy(&flash, &rx_buf[5], sizeof(Flash));
  memcpy(&fcu.pid_gain, &flash.pid_gain, sizeof(fcu.pid_gain));
  NRF_SPIM0->ENABLE = SPIM_ENABLE_ENABLE_Disabled;
}

static uint8_t flash_read_status(void)
{
  static uint8_t tx_buf[1] = {FLASH_RDSR_CMD};
  static uint8_t rx_buf[2] = {0};

  FLASH_CS_LOW();
  nrf_delay_us(1);

  NRF_SPIM0->TXD.PTR = (uint32_t)tx_buf;
  NRF_SPIM0->TXD.MAXCNT = sizeof(tx_buf);
  NRF_SPIM0->RXD.PTR = (uint32_t)rx_buf;
  NRF_SPIM0->RXD.MAXCNT = sizeof(rx_buf);
  NRF_SPIM0->TASKS_START = 1;

  while (!NRF_SPIM0->EVENTS_END)
    ;

  NRF_SPIM0->EVENTS_END = 0;
  FLASH_CS_HIGH();

  const uint8_t status = rx_buf[1];
  return status;
}

static bool flash_write_enable(void)
{
  static uint8_t tx_buf[1] = {FLASH_WREN_CMD};

  FLASH_CS_LOW();
  nrf_delay_us(1);

  NRF_SPIM0->TXD.PTR = (uint32_t)tx_buf;
  NRF_SPIM0->TXD.MAXCNT = sizeof(tx_buf);
  NRF_SPIM0->RXD.PTR = 0;
  NRF_SPIM0->RXD.MAXCNT = 0;
  NRF_SPIM0->TASKS_START = 1;

  while (!NRF_SPIM0->EVENTS_END)
    ;

  NRF_SPIM0->EVENTS_END = 0;
  FLASH_CS_HIGH();

  const uint8_t status = flash_read_status();
  return (status & 0x02) != 0;
}

static bool flash_wait_ready(void)
{
  uint32_t timeout = 1000000;
  while ((flash_read_status() & 0x01) && timeout--)
    ;
  return timeout > 0;
}

static inline bool flash_erase(const uint32_t addr)
{
  if (!flash_write_enable())
    return false;

  static uint8_t tx_buf[4] = {
      (uint8_t)FLASH_SE_CMD,
      (uint8_t)((addr >> 16) & 0xFF),
      (uint8_t)((addr >> 8) & 0xFF),
      (uint8_t)(addr & 0xFF)};

  FLASH_CS_LOW();
  nrf_delay_us(1);

  NRF_SPIM0->TXD.PTR = (uint32_t)tx_buf;
  NRF_SPIM0->TXD.MAXCNT = sizeof(tx_buf);
  NRF_SPIM0->RXD.PTR = 0;
  NRF_SPIM0->RXD.MAXCNT = 0;
  NRF_SPIM0->TASKS_START = 1;

  while (!NRF_SPIM0->EVENTS_END)
    ;

  NRF_SPIM0->EVENTS_END = 0;
  FLASH_CS_HIGH();

  return flash_wait_ready();
}

static inline bool flash_write(void)
{
  if (!flash_write_enable())
    return false;

  static uint8_t tx_buf[4 + sizeof(Flash)] = {
      (uint8_t)FLASH_WRITE_CMD,
      (uint8_t)((FLASH_FCU_ADDR >> 16) & 0xFF),
      (uint8_t)((FLASH_FCU_ADDR >> 8) & 0xFF),
      (uint8_t)(FLASH_FCU_ADDR & 0xFF),
  };
  memcpy(&tx_buf[4], &flash, sizeof(Flash));

  FLASH_CS_LOW();
  nrf_delay_us(1);

  NRF_SPIM0->TXD.PTR = (uint32_t)tx_buf;
  NRF_SPIM0->TXD.MAXCNT = sizeof(tx_buf);
  NRF_SPIM0->RXD.PTR = 0;
  NRF_SPIM0->RXD.MAXCNT = 0;
  NRF_SPIM0->TASKS_START = 1;

  while (!NRF_SPIM0->EVENTS_END)
    ;

  NRF_SPIM0->EVENTS_END = 0;
  FLASH_CS_HIGH();

  return flash_wait_ready();
}
