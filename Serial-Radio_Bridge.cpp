#include <nrf.h>
#include <Adafruit_TinyUSB.h>

#define MBPS_TO_BPS(Mbps) ((Mbps) * 1000000UL)

typedef struct __attribute__((aligned(1), packed))
{
  uint8_t node;
  uint8_t zone;
  uint8_t type;
  uint8_t data[252];
} DataPacket;

static_assert(sizeof(DataPacket) == 255, "DataPacket struct must be 255 bytes");

volatile static DataPacket received_packet;
static DataPacket transmit_packet;

static inline void radioInit(void);
static inline void send_radio_packet(void);

void setup(void)
{
  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  while (!NRF_CLOCK->EVENTS_HFCLKSTARTED)
    __NOP();

  NRF_RADIO->SHORTS = (RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_START_Msk);
  NRF_RADIO->PACKETPTR = (uint32_t)&received_packet;
  NRF_RADIO->TXPOWER = RADIO_TXPOWER_TXPOWER_Pos8dBm;

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

  Serial.begin(MBPS_TO_BPS(1));
  while (!Serial)
    __NOP();
}

void loop(void)
{
  if (NRF_RADIO->EVENTS_CRCOK)
  {
    NRF_RADIO->EVENTS_CRCOK = 0;

    tud_cdc_n_write(0, (uint8_t *)&received_packet, sizeof(DataPacket));
    tud_cdc_n_write_flush(0);
  }

  if (tud_cdc_n_available(0) >= sizeof(DataPacket))
  {
    tud_cdc_n_read(0, (uint8_t *)&transmit_packet, sizeof(DataPacket));
    send_radio_packet();
  }
}

static inline void send_radio_packet(void)
{
  while (!NRF_RADIO->EVENTS_END)
    __NOP();

  NRF_RADIO->EVENTS_END = 0;
  NRF_RADIO->TASKS_DISABLE = 1;

  while (NRF_RADIO->STATE)
    __NOP();

  NRF_RADIO->PACKETPTR = (uint32_t)&transmit_packet;
  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk;
  NRF_RADIO->TASKS_TXEN = 1;

  while (NRF_RADIO->STATE)
    __NOP();

  NRF_RADIO->PACKETPTR = (uint32_t)&received_packet;
  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_START_Msk;
  NRF_RADIO->TASKS_RXEN = 1;
}