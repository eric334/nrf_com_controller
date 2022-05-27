/*! ----------------------------------------------------------------------------
 *  @file    dwm_devid.c
 *  @brief   Set a device id
 *
 *
 * @author Kim Dang
 * Code template from Decawave
 */

#include <deca_device_api.h>
#include <deca_regs.h>
#include <deca_spi.h>
#include <port.h>
#include <shared_defines.h>
#include <string.h>

#include "dwm_testing_functions.h"

extern void test_run_info(unsigned char *data);

/* Example application name */
#define APP_NAME "DEVICE ID ASSIGNMENT"

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

/**
 * Application entry point.
 */
int get_dev_id(void)
{
    int err;
    /* Display application name on LCD. */
    test_run_info((unsigned char *)APP_NAME);

    /* Configure SPI rate, DW3000 supports up to 38 MHz */
    port_set_dw_ic_spi_fastrate();

    /* Reset DW IC */
    reset_DWIC(); /* Target specific drive of RSTn line into DW IC low for a period. */

    Sleep(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

    /* Reads Part ID and Lot ID*/
    uint32_t lotid = dwt_getlotid();
    uint32_t partid = dwt_getpartid();

    //dwt_write32bitoffsetreg(DEV_ID_ID, 0, 0xDECA0001);
    
    printf("Device ID: %X\n", dwt_readdevid());
    printf("Lot ID: %X\n", lotid);
    printf("Part ID: %X\n", partid);

    return DWT_SUCCESS;
}
