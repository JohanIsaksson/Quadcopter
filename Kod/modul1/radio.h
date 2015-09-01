#ifndef RADIO_H
#define RADIO_H


#include <VirtualWire.h>

#define BUFFER_MAX 8

/* Radio structure containing necessary info */
struct radio {
	signed char buffer[BUFFER_MAX];
	byte bufpos;
};

typedef struct radio radio;

/* Initializes the radio and sets parameters */
void init_radio(radio* r);

/* Puts a recieved message in the buffer */
bool read_message(radio * r);

#endif