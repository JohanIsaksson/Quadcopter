#include "fixed_point.h"


uint32_t encode_d(double d){
	uint32_t b = 0;
	double msb_fixed = pow(2.0, INTEGER_BITS - 1);
	uint32_t msb_encoded = (uint32_t)pow(2, TOTAL_BITS - 1);

	for(int i=0; i<TOTAL_BITS; i++){
	    if (d >= msb_fixed){
	    	b += msb_encoded;
	    	d -= msb_fixed;
	    }
	    msb_fixed /= 2.0;
	    msb_encoded /= 2;
	}
	return b;
}

double decode_d(uint32_t b){
	double d = 0.0;
	double lsb_fixed = 1.0 / pow(2.0, TOTAL_BITS - INTEGER_BITS);
	for(int i=0; i<TOTAL_BITS; i++){
	    d += ((b >> i) & 0x00000001)*lsb_fixed;
	    lsb_fixed *= 2.0;
	}
	return d;
}
