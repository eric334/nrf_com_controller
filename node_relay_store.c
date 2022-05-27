/*! ----------------------------------------------------------------------------
 *  @file    node_relay.c
 *  @brief   Node X's code condensed as a function
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
#include "dwm_msg_header.h"

extern void test_run_info(unsigned char *data);
bool get_pir_data(void);

/* Example application name */
#define APP_NAME " RELAY "
/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
    5,               /* Channel number. */
    DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
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
#define RX_RESP_TO_UUS 5000

/* Sleep time after echoing a broadcast, expressed in microseconds. */
#define SLEEP_ECHO 50

/* Buffer to store received frame. See NOTE 5 below. */
static uint8_t rx_buffer[FRAME_LEN_MAX_EX];

static uint8_t storage_buffer[MAX_FILE_SIZE];

/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
 * temperature. These values can be calibrated prior to taking reference measurements. See NOTE 2 below. */
extern dwt_txconfig_t txconfig_options;
extern dwt_txconfig_t txconfig_options_ch9;
extern dwt_config_t config_options;

void dwm_tx_storage(uint8_t* tx_msg, int size);

/**
 * Application entry point.
 */
int node_relay_store(uint8_t device_id)
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

    /* Set response frame timeout. */
    dwt_setrxtimeout(RX_RESP_TO_UUS);

    //Declarations
    int direction = 1; //pos: host to robot, neg: robot to host
    uint8_t rx_id = device_id-direction;
    uint8_t tx_id = device_id+direction;
    uint8_t ack_rx[] = {device_id, rx_id, TYPE_ACK_1, 0, 0};
    uint8_t RTS[] = {device_id, tx_id, TYPE_RTS, 0, 0};
    uint8_t CTS[] = {device_id, tx_id, TYPE_CTS, 0, 0};
    uint8_t broadcast[] = {device_id, BROADCAST, TYPE_MOVE, 0, 1, UP};
    uint8_t done_msg[] = {device_id, tx_id, TYPE_LAST, 0, 0};

    //For storage
    int itr = 0;

    uint8_t pir_data = 0;
    bool my_pir_data = 0;

    //// Node 2 to 3 Tests
    //bool hasRTS = 0;
    //if (!hasRTS) {
    //  // Receive RTS
    //  rx_buffer[TYPE_IDX] = TYPE_RTS;
    //  rx_buffer[SRC_IDX] = rx_id;
    //  rx_buffer[DST_IDX] = device_id;
    //  hasRTS = 1;
    //}

    /***RTS/CTS***/
    //Accept RX from either adjacent nodes for RTS/CTS
    /* RX MSG from prev node*/
    // Loop until you receive message from correct sender
    do{
      //Receive RTS/CTS from either direction
      while(1){
        //if (hasRTS) {
        //  break;
        //}

        memset(rx_buffer,0,sizeof(rx_buffer));
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR  | SYS_STATUS_ALL_RX_TO )))
        { };
        if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
        {
          /* A frame has been received, copy it to our local buffer. */
          frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_BIT_MASK;
          if (frame_len <= FRAME_LEN_MAX_EX)
          {
            dwt_readrxdata(rx_buffer, frame_len-FCS_LEN, 0); /* No need to read the FCS/CRC. */
          }

          if(rx_buffer[DST_IDX]==device_id && (rx_buffer[SRC_IDX]==rx_id || rx_buffer[SRC_IDX]==tx_id) && ((rx_buffer[TYPE_IDX]==TYPE_RTS) || (rx_buffer[TYPE_IDX]==TYPE_CTS)))
          {
            test_run_info((unsigned char *)"RTS/CTS received\n");
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

            break; //break out of loop
          } else if (rx_buffer[DST_IDX]==BROADCAST) {
            //Broadcast message received (MOVE or DROP)
            broadcast[TYPE_IDX] = rx_buffer[TYPE_IDX];
            broadcast[MSG_IDX] = rx_buffer[MSG_IDX];
            dwm_tx(broadcast, sizeof(broadcast));
            Sleep(SLEEP_ECHO);
          }

          dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

        } else {
          /* Clear RX error events in the DW IC status register. */
          dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR  | SYS_STATUS_ALL_RX_TO);
        }
      }
      //Set direction of transmission and update rx/tx_id
      direction = rx_buffer[DST_IDX] - rx_buffer[SRC_IDX];
      rx_id = device_id-direction;
      tx_id = device_id+direction;

      //TX RTS/CTS to next node
      if(rx_buffer[TYPE_IDX] == TYPE_CTS){
        test_run_info((unsigned char *)"CTS Received");
        CTS[DST_IDX] = tx_id;
        dwt_writetxdata((sizeof(CTS)+FCS_LEN)-FCS_LEN, CTS, 0); /* Zero offset in TX buffer. */
        dwt_writetxfctrl((sizeof(CTS)+FCS_LEN), 0, 0); /* Zero offset in TX buffer, no ranging. */
        dwt_starttx(DWT_START_TX_IMMEDIATE);
        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK))
        { };
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
      } else {
        test_run_info((unsigned char *)"RTS Received");
        RTS[DST_IDX] = tx_id;
        dwt_writetxdata((sizeof(RTS)+FCS_LEN)-FCS_LEN, RTS, 0); /* Zero offset in TX buffer. */
        dwt_writetxfctrl((sizeof(RTS)+FCS_LEN), 0, 0); /* Zero offset in TX buffer, no ranging. */
        dwt_starttx(DWT_START_TX_IMMEDIATE);
        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK))
        { };
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

        //// Node 2 to 3 Testing
        //hasRTS = 0;

        ////Code inserted for node 2 to 1 testing
        //test_run_info((unsigned char *)"CTS Sent from node 3");
        //CTS[DST_IDX] = rx_id;
        //dwt_writetxdata((sizeof(CTS)+FCS_LEN)-FCS_LEN, CTS, 0); /* Zero offset in TX buffer. */
        //dwt_writetxfctrl((sizeof(CTS)+FCS_LEN), 0, 0); /* Zero offset in TX buffer, no ranging. */
        //dwt_starttx(DWT_START_TX_IMMEDIATE);
        //while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK))
        //{ };
        //dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
        //rx_buffer[TYPE_IDX] = TYPE_CTS;
        //direction *= -1;
      }  
    }while(rx_buffer[TYPE_IDX] != TYPE_CTS);

    //After CTS, set direction as opposite and update tx_id and rx_id
    direction *= -1;
    rx_id = device_id-direction;
    tx_id = device_id+direction;

    uint16_t buf_len = 125;
    uint8_t tx_msg[buf_len]; //old: 127-2
    memset(tx_msg,0,sizeof(tx_msg));
    tx_msg[SRC_IDX] = device_id;
    tx_msg[DST_IDX] = tx_id;
    tx_msg[TYPE_IDX] = TYPE_DATA;
    tx_msg[PIR_IDX] = 0;
    tx_msg[LEN_IDX] = buf_len-4;

    //Data transfer
    bool notFirst = 0;
    do{
      // Testing node 2 to node 3
      //FILE *myFile;
      //myFile = fopen("image_raw_screenshot.jpeg","rb+");
      //memset(rx_buffer,0,sizeof(rx_buffer));
      //uint16_t buf_len = 125;
      //while(fread(&rx_buffer[MSG_IDX], sizeof(uint8_t), buf_len-4, myFile)!=0){
        
        //Sleep(10);

        /* RX MSG from prev node*/
        // Loop until you receive message from correct sender
        while (1){
          memset(rx_buffer,0,sizeof(rx_buffer));
          dwt_rxenable(DWT_START_RX_IMMEDIATE);
          while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR  | SYS_STATUS_ALL_RX_TO)))
          { };
          if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
          {
              /* A frame has been received, copy it to our local buffer. */
              frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_BIT_MASK;
              if (frame_len <= FRAME_LEN_MAX_EX)
              {
                  dwt_readrxdata(rx_buffer, frame_len-FCS_LEN, 0); /* No need to read the FCS/CRC. */
              }
              
              if ((rx_buffer[DST_IDX]==device_id) && (rx_buffer[SRC_IDX]==rx_id) && (rx_buffer[TYPE_IDX]==TYPE_LAST)) { 
               test_run_info((unsigned char *)"LAST MESSAGE!!!\n");
               dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

               break; //break out of loop
              }

              if((rx_buffer[DST_IDX]==device_id) && (rx_buffer[SRC_IDX]==rx_id) && (rx_buffer[TYPE_IDX]==TYPE_DATA))
              {
                //for(int i=0; i<rx_buffer[3]; i++)
                //{
                //  char tmp[1]={1};
                //  sprintf(tmp,"%c", (unsigned char)rx_buffer[i+4]);
                //  test_run_info((unsigned char *)tmp);
                //}
                memcpy(&storage_buffer[itr], &rx_buffer[MSG_IDX], rx_buffer[LEN_IDX]);
                itr += rx_buffer[LEN_IDX];

                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

                pir_data = rx_buffer[PIR_IDX];

                //break; //break out of loop
              }
              /* Clear good RX frame event in the DW IC status register. */
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
              //test_run_info((unsigned char *)"Frame Received");
          }
          else
          {
              ///* Re-TX ACK 2 to prev node*/
              ////FRAME_LENGTH -> (sizeof(ack_rx)+FCS_LEN)
              //ack_rx[SRC_IDX] = device_id;
              //ack_rx[DST_IDX] = rx_id;
              //ack_rx[TYPE_IDX] = TYPE_ACK_2;

              //dwm_tx(ack_rx, sizeof(ack_rx));

              /* Clear RX error events in the DW IC status register. */
              dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);
          }
        }
        /* TX ACK 1 to prev node*/
        //FRAME_LENGTH -> (sizeof(ack_rx)+FCS_LEN)
        ack_rx[SRC_IDX] = device_id;
        ack_rx[DST_IDX] = rx_id;
        ack_rx[TYPE_IDX] = TYPE_ACK_1;

        dwm_tx(ack_rx, sizeof(ack_rx));
        //test_run_info((unsigned char *)ack_rx);
        //test_run_info((unsigned char *)"ACK 1 Sent to Prev\n");

        
        //Update PIR and TX to next node
        my_pir_data = get_pir_data();
        pir_data &= ~(1U << (device_id - 1));
        pir_data |= (my_pir_data << (device_id-1));
        tx_msg[PIR_IDX] = pir_data;
        dwm_tx_storage(tx_msg, itr);
        itr = 0;
        memset(storage_buffer,0,sizeof(storage_buffer));

        // TX Done Message
        for(int i = 0; i < 3; i++) {
          dwm_tx(done_msg, sizeof(done_msg));
        }

        /* RX ACK_1 from next node  - COMMENT OUT FOR 1 -> 2 NODE TEST*/
        // Loop until you receive message from correct sender
        while(1){
          memset(ack_rx,0,sizeof(ack_rx));
          dwt_rxenable(DWT_START_RX_IMMEDIATE);
          while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO )))
          { };
          if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
          {
              /* A frame has been received, copy it to our local buffer. */
              frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_BIT_MASK;
              if (frame_len <= FRAME_LEN_MAX_EX)
              {
                  dwt_readrxdata(ack_rx, frame_len-FCS_LEN, 0); /* No need to read the FCS/CRC. */
              }

              if((ack_rx[DST_IDX]==device_id) && (ack_rx[SRC_IDX]==tx_id) && (ack_rx[TYPE_IDX]==TYPE_ACK_1))
              {
                test_run_info((unsigned char *)"RX ACK 1 from next\n");
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
                break;
              }
              /* Clear good RX frame event in the DW IC status register. */
              dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
              //test_run_info((unsigned char *)"Frame Received");
          }
          else
          {
              /* Clear RX error events in the DW IC status register. */
              dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);
          }
        }

        ///* TX ACK 2 to prev node*/
        ////FRAME_LENGTH -> (sizeof(ack_rx)+FCS_LEN)
        //ack_rx[SRC_IDX] = device_id;
        //ack_rx[DST_IDX] = rx_id;
        //ack_rx[TYPE_IDX] = TYPE_ACK_2;

        //dwm_tx(ack_rx, sizeof(ack_rx));
        ////test_run_info((unsigned char *)ack_rx);
        ////test_run_info((unsigned char *)"TX ACK 2 to prev\n");

      }while(rx_buffer[TYPE_IDX] != TYPE_LAST);

      test_run_info((unsigned char *)"IMAGE TX --- DONE!");

      return 1;

}

void dwm_tx_storage(uint8_t* tx_msg, int size){
  int i = 0;
  while(size > 0){
    memcpy(tx_msg[MSG_IDX], storage_buffer[i], 120);
    size-=120;
    i+=120;
    dwm_tx(tx_msg, sizeof(tx_msg));
  }
  return;
}

//bool get_pir_data(void) {
//  uint32_t PIR_PIN = NRF_GPIO_PIN_MAP(1,2);
//  nrf_gpio_cfg_input(PIR_PIN, NRF_GPIO_PIN_PULLDOWN);
//  if (nrf_gpio_pin_read(PIR_PIN)) {
//    printf("1\n");
//    return 1;
//  } else {
//    printf("0\n");
//    return 0;
//  }
//} 