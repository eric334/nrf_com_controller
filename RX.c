/*! ----------------------------------------------------------------------------
 *  @file    simple_rx.c
 *  @brief   Simple RX example code
 *
 * @attention
 *
 * author Tom Zu
 */

#include <string.h>
#include <deca_device_api.h>
#include <deca_regs.h>
#include <deca_spi.h>
#include <port.h>
#include <example_selection.h>
#include <shared_defines.h>
#include <dwm_testing_functions.h>
#include "dwm_msg_header.h"


//#if defined(TEST_SIMPLE_RX)

extern void test_run_info(unsigned char *data);

/* Example application name */
#define APP_NAME ""

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
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,  /* STS length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0      /* PDOA mode off */
};

/* Buffer to store received frame. See NOTE 1 below. */
static uint8_t rx_buffer[FRAME_LEN_MAX];

/* Index to access to sequence number of the blink frame in the tx_msg array. */
    #define BLINK_FRAME_SN_IDX 1


    /* Inter-frame delay period, in milliseconds. */
    #define TX_DELAY_MS 500

    extern dwt_txconfig_t txconfig_options;

/**
 * Application entry point.
 */
unsigned int * rx(/*unsigned int *return_buffer, */uint8_t tx_id)
{
    //test_run_info("running");
    /* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
    uint32_t status_reg;
    /* Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint. */
    uint16_t frame_len;

    /* Display application name on LCD. */
    test_run_info((unsigned char *)APP_NAME);

    /* Configure SPI rate, DW IC supports up to 38 MHz */
    port_set_dw_ic_spi_fastrate();

    /* Reset DW IC */
    reset_DWIC(); /* Target specific drive of RSTn line into DW IC low for a period. */

    Sleep(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

    while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */
    { };

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        test_run_info((unsigned char *)"INIT FAILED");
        while (1)
        { };
    }
    /* Enabling LEDs here for debug so that for each RX-enable the D2 LED will flash on DW3000 red eval-shield boards. */
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK) ;

    /* Configure DW IC. */
    if(dwt_configure(&config)) /* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
    {
        test_run_info((unsigned char *)"CONFIG FAILED     ");
        while (1)
        { };
    }

    /* Loop forever receiving frames. */
    while (TRUE)
    {
        //test_run_info("running");
        /* TESTING BREAKPOINT LOCATION #1 */

        /* Clear local RX buffer to avoid having leftovers from previous receptions  This is not necessary but is included here to aid reading
         * the RX buffer.
         * This is a good place to put a breakpoint. Here (after first time through the loop) the local status register will be set for last event
         * and if a good receive has happened the data buffer will have the data in it, and frame_len will be set to the length of the RX frame. */
        memset(rx_buffer,0,sizeof(rx_buffer));

        /* Activate reception immediately. See NOTE 2 below. */
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        /* Poll until a frame is properly received or an error/timeout occurs. See NOTE 3 below.
         * STATUS register is 5 bytes long but, as the event we are looking at is in the first byte of the register, we can use this simplest API
         * function to access it. */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR )))
        { };

        //status_reg = status_reg | SYS_STATUS_RXFCG_BIT_MASK;

        if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
        {
            /* A frame has been received, copy it to our local buffer. */
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_BIT_MASK;
            if (frame_len <= FRAME_LEN_MAX)
            {
                dwt_readrxdata(rx_buffer, frame_len, 0); /* No need to read the FCS/CRC. */
            }
  
            uint8_t device_id=2;
          
            if(rx_buffer[1]==device_id && rx_buffer[0]==tx_id)
            {
              ////test_run_info("gotit");
              ////test_run_info((unsigned char *)"message received from node ");
              //printf("%.0d",rx_buffer[SRC_IDX]);
              ////test_run_info((unsigned char *)" to node ");
              //printf("%.0d",rx_buffer[DST_IDX]);
              //printf("%.0d",rx_buffer[TYPE_IDX]);
              ////printf("%.0d",rx_buffer[3]);
              ////test_run_info((unsigned char *)". message: ");
              
              //for(int i=0; i<rx_buffer[LEN_IDX]; i++)
              //{
              //  char tmp[1]={1};
              //  sprintf(tmp,"%.0d", (unsigned char)rx_buffer[i+MSG_IDX]);
              //  test_run_info((unsigned char *)tmp);
              //}

              /*for(int i=0; i<rx_buffer[2]+3; i++)
              {
                return_buffer[i]=rx_buffer[i];
                //char tmp[1]={1};
                //sprintf(tmp,"%c", (unsigned char)rx_buffer[i]);
                //test_run_info((unsigned char *)tmp);
              }*/




              return rx_buffer;
              break;
            }
            else{
               //test_run_info((unsigned char *)"no");
                //printf("%.0d",rx_buffer[0]);
                //printf("%.0d",rx_buffer[1]);
            }

            /* Clear good RX frame event in the DW IC status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

            //test_run_info((unsigned char *)"Frame Received");

        }
        else
        {
            /* Clear RX error events in the DW IC status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        }
    }
}
//#endif