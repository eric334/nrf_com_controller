/*! ----------------------------------------------------------------------------
 *  @file    node_endhost_prep.c
 *  @brief   Code for Node Endhost when dropping off the nodes
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
#include <dwm_testing_functions.h>
#include <config_options.h>
#include <dwm_msg_header.h>

extern void test_run_info(unsigned char *data);

/* Example application name */
#define APP_NAME " ENDHOST Prep "
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
uint16_t buf_len = 125;

int node_endhost_prep(uint8_t device_id) {
  
  //uint8_t type_msg = ;    // Condition to trigger tx command
  
  //uint8_t tx_msg[buf_len];
  //memset(tx_msg,0,sizeof(tx_msg));
  //tx_msg[TYPE_IDX] = TYPE_DROP;


  //if (has_command_to_send) {
    
  //}

  return 1;
}