#include <Adafruit_TinyUSB.h>

#define MBPS_TO_BPS(Mbps) ((Mbps) * 1000000UL)

/**
 * @brief Data packet structure for radio communication
 *
 * Total size: 255 bytes (1+1+1+252)
 * - Packed to prevent compiler padding
 * - Must match radio configuration (PCNF1_MAXLEN = PCNF1_STATLEN = 255)
 */
typedef struct __attribute__((packed))
{
  uint8_t node;      // Node identifier (1 byte)
  uint8_t zone;      // Zone/group identifier (1 byte)
  uint8_t type;      // Packet type identifier (1 byte)
  uint8_t data[252]; // Payload data (252 bytes)
} DataPacket;

// Compile-time size validation
static_assert(sizeof(DataPacket) == 255, "DataPacket size must be exactly 255 bytes");

// Global packet buffers
volatile static DataPacket rx_packet; // Receive packet (volatile for ISR safety)
static DataPacket tx_packet;          // Transmit packet

void setup(void)
{
  // Start high-frequency clock (required for radio)
  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  while (!NRF_CLOCK->EVENTS_HFCLKSTARTED)
    __WFE(); // Wait until 64MHz crystal oscillator stabilizes

  // Initialize USB serial with power-efficient waiting
  Serial.begin(MBPS_TO_BPS(12));
  while (!Serial)
    __WFE(); // Wait For Event (low-power wait for USB connection)

  // Configure radio shortcuts for automatic operation:
  // - READY_START: Automatically start when ready
  // - END_START: Automatically restart after packet
  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_START_Msk;

  // Set up radio parameters:
  NRF_RADIO->PACKETPTR = (uint32_t)&rx_packet;                              // Receive buffer location
  NRF_RADIO->FREQUENCY = 0 << RADIO_FREQUENCY_FREQUENCY_Pos;                // Channel 0 (2400MHz)
  NRF_RADIO->TXPOWER = RADIO_TXPOWER_TXPOWER_Pos8dBm;                       // +8dBm output power
  NRF_RADIO->MODECNF0 = (RADIO_MODECNF0_DTX_B0 << RADIO_MODECNF0_DTX_Pos) | // Transmit 0 when idle
                        (RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos);  // Fast radio ramp-up
  NRF_RADIO->MODE = RADIO_MODE_MODE_Nrf_2Mbit << RADIO_MODE_MODE_Pos;       // 2Mbps data rate

  // Packet configuration:
  NRF_RADIO->PCNF0 = (0 << RADIO_PCNF0_LFLEN_Pos) |                             // No length field
                     (0 << RADIO_PCNF0_S0LEN_Pos) |                             // No S0 field
                     (0 << RADIO_PCNF0_S1LEN_Pos) |                             // No S1 field
                     (RADIO_PCNF0_S1INCL_Automatic << RADIO_PCNF0_S1INCL_Pos) | // Automatic S1 inclusion
                     (RADIO_PCNF0_PLEN_8bit << RADIO_PCNF0_PLEN_Pos);           // 8-bit preamble

  NRF_RADIO->PCNF1 = (sizeof(DataPacket) << RADIO_PCNF1_MAXLEN_Pos) |          // Max packet length
                     (sizeof(DataPacket) << RADIO_PCNF1_STATLEN_Pos) |         // Static payload length
                     (2 << RADIO_PCNF1_BALEN_Pos) |                            // Base address length (2 bytes)
                     (RADIO_PCNF1_ENDIAN_Little << RADIO_PCNF1_ENDIAN_Pos) |   // Little-endian
                     (RADIO_PCNF1_WHITEEN_Enabled << RADIO_PCNF1_WHITEEN_Pos); // Enable whitening

  // Addressing configuration:
  NRF_RADIO->BASE0 = 0x0000BABE;                        // Base address (BABE in hex)
  NRF_RADIO->PREFIX0 = 0x41 << RADIO_PREFIX0_AP0_Pos;   // Address prefix (0x41)
  NRF_RADIO->RXADDRESSES = RADIO_RXADDRESSES_ADDR0_Msk; // Enable address 0

  // CRC configuration:
  NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos) |           // 2-byte CRC
                      (RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos); // Skip address in CRC
  NRF_RADIO->CRCPOLY = 0x0000AAAA;                                               // CRC polynomial (x^16 + x^14 + x^12 + ... + x^2 + 1)
  NRF_RADIO->CRCINIT = 0x12345678;                                               // Initial CRC value
  NRF_RADIO->DATAWHITEIV = 0x55;                                                 // Whitening initialization value

  // Start receiving
  NRF_RADIO->TASKS_RXEN = 1;

  Serial.println("Radio ready!");
}

/**
 * @brief Transmits a packet via radio
 *
 * Switches radio to TX mode, sends packet, then returns to RX mode
 * Uses WFE for power-efficient waiting during radio operations
 */
static inline void sendRadioData()
{
  // Wait for current radio operation to complete
  while (!NRF_RADIO->EVENTS_END)
    __WFE(); // Low-power wait for packet end

  NRF_RADIO->EVENTS_END = 0; // Clear event flag

  // Disable radio before reconfiguration
  NRF_RADIO->TASKS_DISABLE = 1;
  while (NRF_RADIO->STATE)
    __WFE(); // Wait until fully disabled

  // Configure for transmission
  NRF_RADIO->PACKETPTR = (uint32_t)&tx_packet;                                     // Set transmit buffer
  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk; // Auto-start and disable after
  NRF_RADIO->TASKS_TXEN = 1;                                                       // Start transmission

  // Wait for transmission to complete
  while (NRF_RADIO->STATE)
    __WFE();

  // Return to receive mode
  NRF_RADIO->PACKETPTR = (uint32_t)&rx_packet;                                   // Restore receive buffer
  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_START_Msk; // Auto-restart
  NRF_RADIO->TASKS_RXEN = 1;                                                     // Enable receiver
}

/**
 * @brief Main processing loop
 *
 * Handles both radio reception and USB communication
 * Uses power-efficient waiting when idle
 */
void loop(void)
{
  // Highest priority: Handle incoming radio packets
  if (NRF_RADIO->EVENTS_CRCOK)
  {
    NRF_RADIO->EVENTS_CRCOK = 0; // Clear flag FIRST to prevent missed interrupts

    // Forward received packet to USB
    tud_cdc_n_write(0, (uint8_t *)&rx_packet, sizeof(DataPacket));
    tud_cdc_n_write_flush(0); // Ensure immediate transmission

    __SEV(); // Signal event to wake any other waiting cores
  }

  // Secondary priority: Handle USB data for transmission
  if (tud_cdc_n_available(0) >= sizeof(DataPacket))
  {
    // Read complete packet from USB
    tud_cdc_n_read(0, (uint8_t *)&tx_packet, sizeof(DataPacket));

    // Transmit via radio
    sendRadioData();
  }

  // Only wait when no work needs to be done
  if (!NRF_RADIO->EVENTS_CRCOK && (tud_cdc_n_available(0) < sizeof(DataPacket)))
  {
    __WFE(); // Power-efficient wait for next event (radio or USB)
  }
}