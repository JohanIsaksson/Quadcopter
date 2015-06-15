#include <VirtualWire.h>

struct radio {
	signed char buffer[VW_MAX_MESSAGE_LEN;
	byte bufpos;
};

typedef struct radio radio;

void init_radio(radio* r);

bool read_message(radio * r);