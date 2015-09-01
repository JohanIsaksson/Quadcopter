#include "radio.h"

/* Initializes the radio and sets parameters */
void init_radio(radio* r){
  // Initialise the IO and ISR
  vw_set_tx_pin(12);
  vw_set_rx_pin(11);
  vw_set_ptt_pin(10);
  vw_set_ptt_inverted(true); // Required for DR3100
  vw_setup(2000); // Bits per sec

  for (int i = 0; i < BUFFER_MAX; i++){
  	r->buffer[i] = 0;
  }
  r->bufpos = 0;

  vw_rx_start();       // Start the receiver PLL running
}

/* Puts a recieved message in the buffer */
bool read_message(radio * r){
	uint8_t temp = BUFFER_MAX;
  signed char temp_buf[BUFFER_MAX];
	if (vw_get_message((uint8_t*)(temp_buf), &temp)) // Non-blocking
  {
    for (uint8_t m = 6; m < temp; m++){
      if (temp_buf[m] != 0){
        return false; //bad message
      }
    }

    /* copy buffer and return */
    for (uint8_t n = 0; n < 6; n++){
      r->buffer[n] = temp_buf[n];
    }
  	return true;    
  }
  return false; //no message
}