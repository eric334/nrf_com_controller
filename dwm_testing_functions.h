/*! ----------------------------------------------------------------------------
 *  @file    dwm_testing_functions.h
 *  @brief   
 *
 *
 * @author Kim Dang
 * Code template from Decawave
 */

#ifndef TESTING_FUNCTIONS_
#define TESTING_FUNCTIONS_

#ifdef __cplusplus
extern "C" {
#endif

int get_dev_id(void);
int set_dev_id(void);
int tx_id_filter(void);
int image_tx(void);
int image_rx(void);
unsigned int * rx(uint8_t tx_id);
int tx(unsigned int *tx_buffer, int size);
void dwm_init(void);
int node_endhost(uint8_t device_id, uint8_t adj_id);
int node_relay(uint8_t device_id);
int node_endhost_prep(uint8_t device_id);
int megabuffer_test_rx(uint8_t device_id, uint8_t adj_id);
int node_endhost_store(uint8_t device_id, uint8_t adj_id);
void dwm_tx_storage(uint8_t* tx_msg, int size);
int node_relay_store(uint8_t device_id);


#ifdef __cplusplus
}
#endif


#endif