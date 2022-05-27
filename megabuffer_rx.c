/*! ----------------------------------------------------------------------------
 *  @file    node_endhost.c
 *  @brief   Node 1's code condensed as a function
 *
 * @attention
 *
 *
 * @author 2022 UCSB Aerobot Capstone
 */

#include <deca_device_api.h>
#include <deca_regs.h>
#include <deca_spi.h>
#include <port.h>
#include <shared_defines.h>
#include <example_selection.h>
#include <dwm_testing_functions.h>
#include <config_options.h>
#include "dwm_msg_header.h"

extern void test_run_info(unsigned char *data);

bool get_pir_data(void);
void dwm_tx(uint8_t* tx_buffer, int len_tx_buffer); 

/* Example application name */
#define APP_NAME " ENDHOST "
/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
    5,               /* Channel number. */
    DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_EXT, /* PHY header mode. */
    //DWT_PHRMODE_STD, /* PHY header mode. */
    DWT_PHRRATE_STD, /* PHY header rate. */
    (129 + 8 - 8),   /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF,
    DWT_STS_LEN_64,  /* STS length, see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0      /* PDOA mode off */
};

/* Index to access to sequence number of the blink frame in the tx_msg array. */
#define BLINK_FRAME_SN_IDX 1

/* Inter-frame delay period, in milliseconds. */
#define TX_DELAY_MS 500

/* Delay from end of transmission to activation of reception, expressed in UWB microseconds (1 uus is 512/499.2 microseconds). See NOTE 2 below. */
#define TX_TO_RX_DELAY_UUS 60

/* Receive response timeout, expressed in UWB microseconds. See NOTE 4 below. */
//#define RX_RESP_TO_UUS 25000
#define RX_RESP_TO_UUS 25000000

/* Buffer to store received frame. See NOTE 5 below. */
static uint8_t rx_buffer[FRAME_LEN_MAX_EX];
static uint8_t storage_buffer[MAX_FILE_SIZE];

/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
 * temperature. These values can be calibrated prior to taking reference measurements. See NOTE 2 below. */
extern dwt_txconfig_t txconfig_options;
extern dwt_txconfig_t txconfig_options_ch9;
extern dwt_config_t config_options;

/**
 * Application entry point.
 */
int megabuffer_test_rx(uint8_t device_id, uint8_t adj_id)
{
    /* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
    uint32_t status_reg = 0;
    /* Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint. */
    uint16_t frame_len = 0;

    /* Display application name on LCD. */
    test_run_info((unsigned char *)APP_NAME);

    /* Configure SPI rate, DW3000 supports up to 38 MHz */
    port_set_dw_ic_spi_fastrate();

    /* Reset DW IC */
    reset_DWIC(); /* Target specific drive of RSTn line into DW IC low for a period. */

    Sleep(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

    while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */
    { };

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        test_run_info((unsigned char *)"INIT FAILED     ");
        while (1)
        { };
    }

    /* Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards. */
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK) ;

    /* Configure DW IC. See NOTE 5 below. */
    if(dwt_configure(&config_options)) /* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
    {
        test_run_info((unsigned char *)"CONFIG FAILED     ");
        while (1)
        { };
    }

    /* Configure the TX spectrum parameters (power PG delay and PG Count) */
    dwt_configuretxrf(&txconfig_options);
    //dwt_configuretxrf(&txconfig_options_ch9);

    ///* Set delay to turn reception on after transmission of the frame. See NOTE 3 below. */
    //dwt_setrxaftertxdelay(TX_TO_RX_DELAY_UUS);

    /* Set response frame timeout. */
    //dwt_setrxtimeout(RX_RESP_TO_UUS);

    //Declarations
    uint16_t buf_len = 125;

    uint8_t tx_msg[buf_len]; //old: 127-2
    memset(tx_msg,0,sizeof(tx_msg));
    tx_msg[SRC_IDX] = device_id;
    tx_msg[DST_IDX] = adj_id;
    tx_msg[TYPE_IDX] = TYPE_DATA;
    tx_msg[LEN_IDX] = buf_len-5;    // CHANGED from 4 to 5

    // CHANGED: define PIR idx
    tx_msg[PIR_IDX] = 0;
    bool my_pir_data = 0;

    //For storage
    int itr = 0;
    int numbytes = 0;

    FILE *storedFile;
    //storedFile = fopen("megabuffer.jpg","wb+");
    storedFile = fopen("rx_image.jpg","wb+");

    memset(storage_buffer,0,sizeof(storage_buffer));

    int error_counter = 0;

    while (1){
      memset(rx_buffer,0,sizeof(rx_buffer));
      dwt_rxenable(DWT_START_RX_IMMEDIATE);
      while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO )))
      { };
      if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
      {
          /* A frame has been received, copy it to our local buffer. */
          frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_BIT_MASK;
          if (frame_len <= FRAME_LEN_MAX_EX)
          {
              dwt_readrxdata(rx_buffer, frame_len-FCS_LEN, 0); /* No need to read the FCS/CRC. */
          }

          if((rx_buffer[DST_IDX]==device_id) && (rx_buffer[SRC_IDX]==adj_id) && (rx_buffer[TYPE_IDX] == TYPE_LAST))
          {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
            test_run_info((unsigned char *)"Last Message Received\n");

            break; //break out of loop
          }

          if((rx_buffer[DST_IDX]==device_id) && (rx_buffer[SRC_IDX]==adj_id) && (rx_buffer[TYPE_IDX] == TYPE_DATA))
          {
            for(int i=0; i<rx_buffer[LEN_IDX]; i++)
            {
              char tmp[1]={1};
              sprintf(tmp,"%c", (unsigned char)rx_buffer[i+4]);
              test_run_info((unsigned char *)tmp);
            }
            memcpy(&storage_buffer[itr], &rx_buffer[MSG_IDX], rx_buffer[LEN_IDX]);
            //for(int i=0; i<rx_buffer[LEN_IDX]; i++)
            //{
            //  char tmp[1]={1};
            //  sprintf(tmp,"%c", (unsigned char)storage_buffer[i+itr]);
            //  test_run_info((unsigned char *)tmp);
            //}
            itr += rx_buffer[LEN_IDX];

            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
            test_run_info((unsigned char *)"RX Data\n");
          }
          dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
      }
      else
      {
          /* Clear RX error events in the DW IC status register. */
          dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR  | SYS_STATUS_ALL_RX_TO );
          error_counter++;
      }
    }


    fwrite(&storage_buffer, sizeof(uint8_t), sizeof(storage_buffer), storedFile);
    fclose(storedFile);

    test_run_info((unsigned char *)"IMAGE RX --- DONE!");
    printf("%d\n",error_counter);

    return 1;

}
