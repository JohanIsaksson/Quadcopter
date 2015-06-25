#include "radio.h"

/* Initializes the radio and sets parameters */
void init_radio(radio* r){
  // Initialise the IO and ISR
  vw_set_tx_pin(12);
  vw_set_rx_pin(11);
  vw_set_ptt_pin(10);
  vw_set_ptt_inverted(true); // Required for DR3100
  vw_setup(6000); // Bits per sec

  for (int i = 0; i < VW_MAX_MESSAGE_LEN; i++){
  	r->buffer[i] = 0;
  }
  r->bufpos = 0;

  vw_rx_start();       // Start the receiver PLL running
}

/* Puts a recieved message in the buffer */
bool read_message(radio * r){
	uint8_t temp = VW_MAX_MESSAGE_LEN;
	if (vw_get_message((uint8_t*)(r->buffer), &temp)) // Non-blocking
  {
  	return true;    
  }
  return false; //no message
}