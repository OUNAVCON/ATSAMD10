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
GCLK_CLKCTRL = 0x4006; //Enable EVSYS Clock. Channel zero
PM_APBCMASK |= 0x00000002;
initTccToAdcEvent();
}

static void initTccToAdcEvent(void){
//Setup ADC to Receive event
ADC_CTRLA &= 0xFD;
while(ADC_STATUS & 0x80);
ADC_EVCTRL |= 0x01; //Event Starts a new Conversion
ADC_CTRLA |= 0x02;
while(ADC_STATUS & 0x80);
//Setup TCC to fire event on Counter==Period
TCC_CTRLA &= 0xFFFFFFFD;
while(TCC_SYNCBUSY>0);
TCC_EVCTRL |= 0x00000440;
TCC_CTRLA |= 0x00000002;
while(TCC_SYNCBUSY>0);
//Setup Event Controller

//FROM Page 407
//Single 16 bit write to USER mux with the USER.CHANNEL
EVSYS_USER = 0x000C; //User ADC Start Of Conversion on Ch:0
EVSYS_CHANNEL = 0x061B0000; //TCC Generator on Ch:0

}

