///*********************************************************************
//*                    SEGGER Microcontroller GmbH                     *
//*                        The Embedded Experts                        *
//**********************************************************************
//*                                                                    *
//*            (c) 2014 - 2020 SEGGER Microcontroller GmbH             *
//*                                                                    *
//*           www.segger.com     Support: support@segger.com           *
//*                                                                    *
//**********************************************************************
//*                                                                    *
//* All rights reserved.                                               *
//*                                                                    *
//* Redistribution and use in source and binary forms, with or         *
//* without modification, are permitted provided that the following    *
//* conditions are met:                                                *
//*                                                                    *
//* - Redistributions of source code must retain the above copyright   *
//*   notice, this list of conditions and the following disclaimer.    *
//*                                                                    *
//* - Neither the name of SEGGER Microcontroller GmbH                  *
//*   nor the names of its contributors may be used to endorse or      *
//*   promote products derived from this software without specific     *
//*   prior written permission.                                        *
//*                                                                    *
//* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND             *
//* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,        *
//* INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF           *
//* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE           *
//* DISCLAIMED.                                                        *
//* IN NO EVENT SHALL SEGGER Microcontroller GmbH BE LIABLE FOR        *
//* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR           *
//* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT  *
//* OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;    *
//* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF      *
//* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT          *
//* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE  *
//* USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH   *
//* DAMAGE.                                                            *
//*                                                                    *
//**********************************************************************

//-------------------------- END-OF-HEADER -----------------------------

//File    : main.c
//Purpose : Nordic nRF52840-DK build main entry point for simple exmaples.

///*************************** End of file ****************************/


/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************
*                                                                    *
*            (c) 2014 - 2020 SEGGER Microcontroller GmbH             *
*                                                                    *
*           www.segger.com     Support: support@segger.com           *
*                                                                    *
**********************************************************************
*                                                                    *
* All rights reserved.                                               *
*                                                                    *
* Redistribution and use in source and binary forms, with or         *
* without modification, are permitted provided that the following    *
* conditions are met:                                                *
*                                                                    *
* - Redistributions of source code must retain the above copyright   *
*   notice, this list of conditions and the following disclaimer.    *
*                                                                    *
* - Neither the name of SEGGER Microcontroller GmbH                  *
*   nor the names of its contributors may be used to endorse or      *
*   promote products derived from this software without specific     *
*   prior written permission.                                        *
*                                                                    *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND             *
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,        *
* INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF           *
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE           *
* DISCLAIMED.                                                        *
* IN NO EVENT SHALL SEGGER Microcontroller GmbH BE LIABLE FOR        *
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR           *
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT  *
* OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;    *
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF      *
* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT          *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE  *
* USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH   *
* DAMAGE.                                                            *
*                                                                    *
**********************************************************************

-------------------------- END-OF-HEADER -----------------------------

File    : main.c
Purpose : Nordic nRF52840-DK build main entry point for simple exmaples.

*/

#include <stdio.h>
#include <stdlib.h>
#include <sdk_config.h>
#include <boards.h>
#include <port.h>
#include <deca_spi.h>
#include <examples_defines.h>
#include "dwm_testing_functions.h"
#include "shared_defines.h"
#include "dwm_msg_header.h"
#include <nrf_gpio.h>

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "nrf.h"
#include "nrf_drv_usbd.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_power.h"

#include "app_error.h"
#include "app_util.h"
#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"

#include "boards.h"
#include "bsp.h"
#include "bsp_cli.h"
#include "nrf_cli.h"
#include "nrf_cli_uart.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include <stdio.h>
#include <string.h>
#include <port.h>
#include <dwm_testing_functions.h>

#include <deca_device_api.h>
#include <deca_regs.h>
#include <deca_spi.h>
#include <port.h>
#include <shared_defines.h>

//#include <pthread.h>


#define UNIT_TEST 0
#if NRF_CLI_ENABLED
/**
 * @brief CLI interface over UART
 */
NRF_CLI_UART_DEF(m_cli_uart_transport, 0, 64, 16);
NRF_CLI_DEF(m_cli_uart,
            "uart_cli:~$ ",
            &m_cli_uart_transport.transport,
            '\r',
            4);
#endif

/**@file
 * @defgroup usbd_cdc_acm_example main.c
 * @{
 * @ingroup usbd_cdc_acm_example
 * @brief USBD CDC ACM example
 *
 */

#define LED_USB_RESUME      (BSP_BOARD_LED_0)
#define LED_CDC_ACM_OPEN    (BSP_BOARD_LED_1)
#define LED_CDC_ACM_RX      (BSP_BOARD_LED_2)
#define LED_CDC_ACM_TX      (BSP_BOARD_LED_3)

#define BTN_CDC_DATA_SEND       0
#define BTN_CDC_NOTIFY_SEND     1

#define BTN_CDC_DATA_KEY_RELEASE        (bsp_event_t)(BSP_EVENT_KEY_LAST + 1)

/**
 * @brief Enable power USB detection
 *
 * Configure if example supports USB port connection
 */
#ifndef USBD_POWER_DETECTION
#define USBD_POWER_DETECTION true
#endif


static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event);

#define CDC_ACM_COMM_INTERFACE  0
#define CDC_ACM_COMM_EPIN       NRF_DRV_USBD_EPIN2

#define CDC_ACM_DATA_INTERFACE  1
#define CDC_ACM_DATA_EPIN       NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT      NRF_DRV_USBD_EPOUT1


/**
 * @brief CDC_ACM class instance
 * */
APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm,
                            cdc_acm_user_ev_handler,
                            CDC_ACM_COMM_INTERFACE,
                            CDC_ACM_DATA_INTERFACE,
                            CDC_ACM_COMM_EPIN,
                            CDC_ACM_DATA_EPIN,
                            CDC_ACM_DATA_EPOUT,
                            APP_USBD_CDC_COMM_PROTOCOL_AT_V250
);

// Global Variables Declaration
#define READ_SIZE 1
#define FRAME_LEN 64

static char m_rx_buffer[READ_SIZE];
static char m_tx_buffer[NRF_DRV_USBD_EPSIZE];
static char buffer[FRAME_LEN];

int count = 0;
int pointer = 0;
static bool m_send_flag = 1;

static uint8_t rx_buffer[FRAME_LEN_MAX_EX];
static uint8_t storage_buffer[MAX_FILE_SIZE];

int itr = 0;
int sent_itr = 0;
bool should_send = TRUE;
bool should_send_2 = TRUE;

static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event)
{
    app_usbd_cdc_acm_t const * p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);

    switch (event)
    {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
        {
            bsp_board_led_on(LED_CDC_ACM_OPEN);

            /*Setup first transfer*/
            ret_code_t ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                                   m_rx_buffer,
                                                   READ_SIZE);
            UNUSED_VARIABLE(ret);
            break;
        }
        case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
            bsp_board_led_off(LED_CDC_ACM_OPEN);
            break;
        case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
            bsp_board_led_invert(LED_CDC_ACM_TX);
            break;
        case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
        {
            ret_code_t ret;
            app_usbd_cdc_acm_bytes_stored(p_cdc_acm);
            do
            {
                buffer[pointer]=m_rx_buffer[0];
                /*Get amount of data transfered*/
                //size_t size = app_usbd_cdc_acm_rx_size(p_cdc_acm);
                /* Fetch data until internal buffer is empty */
                ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                            m_rx_buffer,
                                            READ_SIZE);
                pointer++;
            } while (ret == NRF_SUCCESS);
            
            count++;
            printf("%d\n", count);

            if (buffer[0] == 'e' && buffer[1] == 'n' && buffer[2] == 'd') {
              should_send_2 = FALSE;
              printf(buffer);
              printf("end buffer received\n");
            } else {
              // Write Serial Data to Mega Buffer
              memcpy(&storage_buffer[itr], &buffer[0], sizeof(buffer));
              itr += sizeof(buffer);
            }
            
            memset(buffer, '\0', sizeof(buffer));
            pointer=0;
            break;
        }
        default:
            break;
    }
}



static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
    {
        case APP_USBD_EVT_DRV_SUSPEND:
            bsp_board_led_off(LED_USB_RESUME);
            break;
        case APP_USBD_EVT_DRV_RESUME:
            bsp_board_led_on(LED_USB_RESUME);
            break;
        case APP_USBD_EVT_STARTED:
            break;
        case APP_USBD_EVT_STOPPED:
            app_usbd_disable();
            bsp_board_leds_off();
            break;
        case APP_USBD_EVT_POWER_DETECTED:
            NRF_LOG_INFO("USB power detected");

            if (!nrf_drv_usbd_is_enabled())
            {
                app_usbd_enable();
            }
            break;
        case APP_USBD_EVT_POWER_REMOVED:
            NRF_LOG_INFO("USB power removed");
            app_usbd_stop();
            break;
        case APP_USBD_EVT_POWER_READY:
            NRF_LOG_INFO("USB ready");
            app_usbd_start();
            break;
        default:
            break;
    }
}

static void bsp_event_callback(bsp_event_t ev)
{
    ret_code_t ret;
    switch ((unsigned int)ev)
    {
        case CONCAT_2(BSP_EVENT_KEY_, BTN_CDC_DATA_SEND):
        {
            //m_send_flag = 1;
            break;
        }
        
        case BTN_CDC_DATA_KEY_RELEASE :
        {
            //m_send_flag = 0;
            break;
        }

        case CONCAT_2(BSP_EVENT_KEY_, BTN_CDC_NOTIFY_SEND):
        {
            ret = app_usbd_cdc_acm_serial_state_notify(&m_app_cdc_acm,
                                                       APP_USBD_CDC_ACM_SERIAL_STATE_BREAK,
                                                       false);
            UNUSED_VARIABLE(ret);
            break;
        }

        default:
            return; // no implementation needed
    }
}

static void init_bsp(void)
{
    ret_code_t ret;
    ret = bsp_init(BSP_INIT_BUTTONS, bsp_event_callback);
    APP_ERROR_CHECK(ret);
    
    UNUSED_RETURN_VALUE(bsp_event_to_button_action_assign(BTN_CDC_DATA_SEND,
                                                          BSP_BUTTON_ACTION_RELEASE,
                                                          BTN_CDC_DATA_KEY_RELEASE));
    
    /* Configure LEDs */
    bsp_board_init(BSP_INIT_LEDS);
}

#if NRF_CLI_ENABLED
static void init_cli(void)
{
    ret_code_t ret;
    ret = bsp_cli_init(bsp_event_callback);
    APP_ERROR_CHECK(ret);
    nrf_drv_uart_config_t uart_config = NRF_DRV_UART_DEFAULT_CONFIG;
    uart_config.pseltxd = TX_PIN_NUMBER;
    uart_config.pselrxd = RX_PIN_NUMBER;
    uart_config.hwfc    = NRF_UART_HWFC_DISABLED;
    ret = nrf_cli_init(&m_cli_uart, &uart_config, true, true, NRF_LOG_SEVERITY_INFO);
    APP_ERROR_CHECK(ret);
    ret = nrf_cli_start(&m_cli_uart);
    APP_ERROR_CHECK(ret);
}
#endif

#define UNIT_TEST 0
#define LED1_PIN 17

extern example_ptr example_pointer;
extern int unit_test_main(void);
extern void build_examples(void);
extern int image_tx(void);
extern int image_rx(void);
//extern int simple_tx_ack(unsigned int * tx_msg, int size);

/*! ------------------------------------------------------------------------------------------------------------------
* @fn test_run_info()
*
* @brief  This function is simply a printf() call for a string. It is implemented differently on other platforms,
*         but on the nRF52840-DK, a printf() call is .
*
* @param data - Message data, this data should be NULL string.
*
* output parameters
*
* no return value
*/
void test_run_info(unsigned char *data)
{
    printf("%s\n", data);
}

/*********************************************************************
*
*       main()
*
*  Function description
*   Application entry point.
*/


int main(void) {
    uint8_t device_id = 1;
    uint8_t adj_id = 2;
    //should_send = FALSE;

    /* Reset of all peripherals (if attached). */
    build_examples();

    /* Initialize all configured peripherals */
    bsp_board_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS);

    /* Initialise the SPI for nRF52840-DK */
    nrf52840_dk_spi_init();

    /* Configuring interrupt*/
    dw_irq_init();

    /* Small pause before startup */
    nrf_delay_ms(2);

    dwm_init();

    //Declarations
    uint16_t buf_len = FRAME_LEN+5;   // 125
    uint8_t done_msg[] = {device_id, adj_id, TYPE_LAST, 0, 0};
    uint8_t RTS[] = {device_id, adj_id, TYPE_RTS, 0, 0};
    uint8_t CTS[] = {device_id, adj_id, TYPE_CTS, 0, 0};
    uint8_t ack_rx[] = {device_id, adj_id, TYPE_ACK_1, 0, 0};

    uint8_t tx_msg[buf_len];
    memset(tx_msg,0,sizeof(tx_msg));
    tx_msg[SRC_IDX] = device_id;
    tx_msg[DST_IDX] = adj_id;
    tx_msg[TYPE_IDX] = TYPE_DATA;
    tx_msg[LEN_IDX] = buf_len-5;

    bool ack2 = FALSE;

    // Define PIR idx
    tx_msg[PIR_IDX] = 0;
    bool my_pir_data = 0;

    /* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
    uint32_t status_reg = 0;
    /* Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint. */
    uint16_t frame_len = 0;


    ret_code_t ret;
    static const app_usbd_config_t usbd_config = {
        .ev_state_proc = usbd_user_ev_handler
    };

    ret = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(ret);

    ret = nrf_drv_clock_init();
    APP_ERROR_CHECK(ret);
    
    nrf_drv_clock_lfclk_request(NULL);

    while(!nrf_drv_clock_lfclk_is_running())
    {
        /* Just waiting */
    }

    ret = app_timer_init();
    APP_ERROR_CHECK(ret);

    init_bsp();
#if NRF_CLI_ENABLED
    init_cli();
#endif

    app_usbd_serial_num_generate();

    ret = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(ret);
    NRF_LOG_INFO("USBD CDC ACM example started.");

    app_usbd_class_inst_t const * class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
    ret = app_usbd_class_append(class_cdc_acm);
    APP_ERROR_CHECK(ret);

    if (USBD_POWER_DETECTION)
    {
        ret = app_usbd_power_events_enable();
        APP_ERROR_CHECK(ret);
    }
    else
    {
        NRF_LOG_INFO("No USB power detection enabled\r\nStarting USB now");

        app_usbd_enable();
        app_usbd_start();
    }

   
    int tmp_counter = 0;

    while(TRUE) {
      

      if (should_send) {
        // DWM TX
        while (sent_itr < itr) {
          memset(&tx_msg[MSG_IDX],0,sizeof(buffer));
          memcpy(&tx_msg[MSG_IDX], &storage_buffer[sent_itr], sizeof(buffer));

          

          tx_msg[LEN_IDX] = tmp_counter;
          printf("Frame:\n");
          printf("%d\n", tx_msg[LEN_IDX]);
          
          // Double ACK
          dwm_tx(tx_msg,sizeof(tx_msg));
          //printf(tmp_counter);
          //printf("\n");
          
          /* RX_code*/
          while(1){
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

                if((rx_buffer[DST_IDX]==device_id) && (rx_buffer[SRC_IDX]==adj_id) && (rx_buffer[TYPE_IDX]==TYPE_ACK_1))
                {
                  printf("ACK 1 Received\n");
                  tmp_counter ++;
                  //printf(tmp_counter);
                  //printf("\n");

                  //test_run_info((unsigned char *)"ACK Received\n");
                  /* Clear good RX frame event in the DW IC status register. */
                  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

                  // Display PIR
                  // test_run_info((unsigned char *)rx_buffer[PIR_IDX]);

                  break;
                }
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

            }
            else
            {
                // If not received ACK 1, Re TX
                dwm_tx(tx_msg,sizeof(tx_msg));
                printf("Re TX Msg\n");
                /* Clear RX error events in the DW IC status register. */
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);
            }
            
            while (app_usbd_event_queue_process())
            { }
          }

          if(rx_buffer[TYPE_IDX] == TYPE_ACK_2){ack2 = TRUE;}

          /* RX_code*/
          if(!ack2){
            while(1){
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

                  if((rx_buffer[DST_IDX]==device_id) && (rx_buffer[SRC_IDX]==adj_id) && (rx_buffer[TYPE_IDX] == TYPE_ACK_2))
                  {
                    test_run_info((unsigned char *)"A2 received\n");
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

                    // Display PIR
                    // test_run_info((unsigned char *)rx_buffer[PIR_IDX]);

                    break;
                  }

                  

                  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
              }
              else
              {
                  /* Clear RX error events in the DW IC status register. */
                  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);
              }

              if (should_send_2 == FALSE) {break;}
              
              while (app_usbd_event_queue_process())
              { }
            }
          }

          sent_itr += sizeof(buffer);

          while (app_usbd_event_queue_process())
          { }
        }

        if (should_send_2 == FALSE) {
          should_send = FALSE;
          should_send_2 = TRUE;
          
          // Insurance for donw msg
          for(int i = 0; i<3; i++){
            dwm_tx(done_msg,sizeof(done_msg));
            printf("TX Done\n");
          }
          test_run_info((unsigned char *)"TX --- DONE!\n");
        }
        
        
      } else { 
        // DWM RX
        
        FILE *myFile;
        //myFile = fopen("rx_image.jpg","wb+");
        myFile = fopen("rx_image.txt","wb+");

        do{
          /* RX MSG from prev node*/
          // Loop until you receive message from correct sender
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
                  should_send = TRUE;

                  // Display PIR
                  // test_run_info((unsigned char *)rx_buffer[PIR_IDX]);
                  
                  break; //break out of loop
                }

                if((rx_buffer[DST_IDX]==device_id) && (rx_buffer[SRC_IDX]==adj_id) && (rx_buffer[TYPE_IDX] == TYPE_DATA))
                {
                  //if (rx_buffer[MSG_IDX] == 'e' && rx_buffer[MSG_IDX+1] == 'n' && rx_buffer[MSG_IDX+2] == 'd') {
                  //  printf("Received END\n");
                  //  should_send = TRUE;
                  //}
                  test_run_info((unsigned char *)"frame: ");
                  printf("%d",rx_buffer[LEN_IDX]);
                  printf("\n");
                  for(int i=0; i<63; i++)
                  {
                    char tmp[1]={1};
                    sprintf(tmp,"%c", (unsigned char)rx_buffer[i+5]);
                    test_run_info((unsigned char *)tmp);
                    buffer[i]=rx_buffer[i+5];
                    //test_run_info((unsigned char *)rx_buffer[i+3]);
                    //printf("%.0d",rx_buffer[i+3]);
                  }
                  printf("\n");
                  ret = app_usbd_cdc_acm_write(&m_app_cdc_acm, buffer, sizeof(buffer));
                  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
                  test_run_info((unsigned char *)"RX Data\n");

                  /* Here would be some processing or saving of msg*/
                  //fwrite(&rx_buffer[MSG_IDX], sizeof(uint8_t), buf_len-5, myFile);
                  fwrite(&rx_buffer[MSG_IDX], sizeof(uint8_t), 64, myFile);

                  // Display PIR
                  // test_run_info((unsigned char *)rx_buffer[PIR_IDX]);

                  break; //break out of loop
                }
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
            }
            else
            {
                /* Re-TX ACK 2 to adj node*/
                //FRAME_LENGTH -> (sizeof(ack_rx)+FCS_LEN)
                //Sleep(25);
                ack_rx[TYPE_IDX] = TYPE_ACK_2;
                dwm_tx(ack_rx,sizeof(ack_rx));
                //test_run_info((unsigned char *)"Re TX ACK2 ");
                /* Clear RX error events in the DW IC status register. */
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR  | SYS_STATUS_ALL_RX_TO );
            }

            while (app_usbd_event_queue_process())
            {}
          }
        
          

          /* TX ACK 1 to adj node*/
          //FRAME_LENGTH -> (sizeof(ack_rx)+FCS_LEN)
          ack_rx[TYPE_IDX] = TYPE_ACK_1;
          dwm_tx(ack_rx,sizeof(ack_rx));
          //test_run_info((unsigned char *)"TX ACK1\n");

          if (should_send) {break;}
          if (rx_buffer[TYPE_IDX]==TYPE_LAST) {break;}

          /* TX ACK 2 to adj node*/
          //FRAME_LENGTH -> (sizeof(ack_rx)+FCS_LEN)
          //Sleep(25);
          ack_rx[TYPE_IDX] = TYPE_ACK_2;
          dwm_tx(ack_rx,sizeof(ack_rx));
          //test_run_info((unsigned char *)"TX ACK2 ");

        }while(rx_buffer[TYPE_IDX] != TYPE_LAST);
        fclose(myFile);
        test_run_info((unsigned char *)"RX --- DONE!");

        sent_itr = 0;
        itr = 0;
        memset(storage_buffer, '\0', sizeof(storage_buffer));
      }





      

      while (app_usbd_event_queue_process())
      { }

        
#if NRF_CLI_ENABLED
        nrf_cli_process(&m_cli_uart);
#endif

        UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
        /* Sleep CPU only if there was no interrupt since last loop processing */
        __WFE();
    }

}

/*************************** End of file ****************************/








