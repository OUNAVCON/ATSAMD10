/*
 * tcc.c
 *
 *  Created on: Apr 19, 2018
 *      Author: Isaac.Rose
 */
#include "tcc.h"
#include "samd10d13as.h"

void init_Tcc(void){

    // Enable Bus Clock
    while(GCLK_STATUS & 0x80);
    GCLK_CLKCTRL = 0x4011; //TCC0 uses OSC1 (8mhz)
    while(GCLK_STATUS & 0x80);
    PM_APBCMASK |= 0x00000020; //Enable the clock to TCC0
    // Prescalar CTRLA.PRESCALAR
    TCC_CTRLA = 0x00001000;
    while(TCC_SYNCBUSY > 0);
    // CTRLA.PRESYNC (On reload 0)
    // CTRLBSET.DIR Up-Down
    TCC_CTRLBSET = 0x20; //Enable Re-trigger.
    while(TCC_SYNCBUSY > 0);
    TCC_EVCTRL = 0x00000480;
    while(TCC_SYNCBUSY > 0);
    /*
     * DSBOTTOM
     * POLx=0 output is clear on increment to CC, and set when on decrement to CC.
     * w[3] and w[4] enabled.
     */
    TCC_WAVE  = 0x000C0005;
    while(TCC_SYNCBUSY > 0);
    TCC_PER   = 0x000003FF;
    while(TCC_SYNCBUSY > 0);
//    TCC_DBGCTRL = 0x01; //Enable debug.
    TCC_CC(2) = 0x000001FF; //w[2] & w[3]
    while(TCC_SYNCBUSY > 0);
    TCC_CC(3) = 0x000000FF; //w[4] & w[5]
    while(TCC_SYNCBUSY > 0);
    //Setup the count reaching the period to trigger an ADC Start of conversion event.
    while(TCC_SYNCBUSY > 0);
    TCC_CTRLA |= 0x2; //Enable the peripheral.
    while(TCC_SYNCBUSY > 0);
}
