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
    GCLK_CLKCTRL =0x4111; //TCC0 uses OSC1 (8mhz)
    PM_APBCMASK |= 0x00000020; //Enable the clock to TCC0
    // Prescalar CTRLA.PRESCALAR
    TCC_CTRLA = 0x000003;
    // CTRLA.PRESYNC (On reload 0)
    // CTRLBSET.DIR Up-Down
    TCC_CTRLBSET = 0x20; //Enable Re-trigger.
    /*
     * DSBOTTOM
     * POLx=0 output is clear on increment to CC, and set when on decrement to CC.
     * w[3] and w[4] enabled.
     */
    TCC_WAVE = 0x00060604;
    TCC_PER = 0x0000FFFF;
    TCC_DBGCTRL = 0x01; //Enable debug.
    TCC_CC(3) = 0x0000EFFF;
    TCC_CC(4) = 0x0000EFFF;

    TCC_CTRLA |= 0x2; //Enalble the peripheral.
}
