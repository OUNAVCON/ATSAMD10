/*
 * events.c
 *
 *  Created on: Apr 19, 2018
 *      Author: Isaac.Rose
 */
#include "events.h"
#include "samd10d13as.h"

static void initTccToAdcEvent(void);

void init_Events(void){
//enable Event System Clock in PM
//GCLK_EVSYS_Channel0;
GCLK_CLKCTRL = 0x4007; //Enable EVSYS Clock. Channel zero
PM_APBCMASK |= 0x00000002;
initTccToAdcEvent();
}

static void initTccToAdcEvent(void){
//Setup ADC to Receive event - Handled in adc.c
//Setup TCC to fire event on Counter==Period - Handled in tcc.c

//Setup Event Controller

//FROM Page 407
//Single 16 bit write to USER mux with the USER.CHANNEL
EVSYS_USER = 0x010C; //User ADC Start Of Conversion on Ch:0 ((Value is channel +1)
EVSYS_CHANNEL = 0x061A0000; //TCC Generator on Ch:0

}

