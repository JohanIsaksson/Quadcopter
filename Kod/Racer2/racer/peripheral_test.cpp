// micro timer

// Enable the TC bus clock
APB0MASK |= (1 << CLK_TC0_APB);

// set control register: (page 652)
//	bits	description				config
//	15:14 	reserved				-
//	13:12 	prescaler sync			sync to prescaled clock
//	11		run in standby			enabled
//	10:8	prescaler				/16
//	7		reserved				-
//	6:5		wave generation			none
//	4		reserved				-
//	3:2		(8, 16, 32) bit mode	8 bit
//	1		enable					enabled
//	0		software reset 			no reset
	
REG_TC0_CTRLA = 0b0001110000000010;

CTRLB = 0b

// capture channel CPTEN0
CTRLC = 0b00010000;

//EVCTRL = (1 << TCEI);

INTENSET = (1 << MC0)

CC = 2;


//TODO: COMPLETE THIS





// external interrupts

// enable EIC
CTRL = 0b00000010;

// enable interrupts on the radio pins
//	arduino		port		interrupt
//	13			PA17		EXTINT1
//	12			PA19		EXTINT3
//	11			PA16		EXTINT0
//	10			PA18		EXTINT2
//	9			PA07		EXTINT7
//	8			PA06		EXTINT6

// set pins to inputs - UGLY
PORT->[g_APinDescription[8].ulPort].PINCFG[g_APinDescription[8].ulPin].bit.PMUXEN = 1;
PORT->[g_APinDescription[9].ulPort].PINCFG[g_APinDescription[9].ulPin].bit.PMUXEN = 1;
PORT->[g_APinDescription[10].ulPort].PINCFG[g_APinDescription[10].ulPin].bit.PMUXEN = 1;
PORT->[g_APinDescription[11].ulPort].PINCFG[g_APinDescription[11].ulPin].bit.PMUXEN = 1;
PORT->[g_APinDescription[12].ulPort].PINCFG[g_APinDescription[12].ulPin].bit.PMUXEN = 1;
PORT->[g_APinDescription[13].ulPort].PINCFG[g_APinDescription[13].ulPin].bit.PMUXEN = 1;

REG_

// enable interrupts
INTENSET = (1 << EXTINT1) | (1 << EXTINT3) | (1 << EXTINT0) | (1 << EXTINT2) | (1 << EXTINT7) | (1 << EXTINT6);

// no filtering and trigger on both edges (0011) for each interrupt
CONFIG0 = 0b00110011001100110011001100110011;

// ISR for external interrupts
void EIC_Handler(){

}