#include "samd10d13as.h"
#include "clocks.h"

static void DFLL48M_to_core_init(void);

void initClocks(void){
uint32_t osc8Value = 0x00;
//Disable the Watch Dog.


WDT_CTRL = 0x00;
while(WDT_STATUS & 0x80); //make sure that the wdt is disabled.

osc8Value = SYSCTRL_OSC8M;
osc8Value &= 0xFFFFFCFF;
SYSCTRL_OSC8M = osc8Value; //Set the prescalar to 0 which is divide by 1.
while(!(SYSCTRL_PCLKSR & 0x8));//Make sure the oscillator is stable 

while(GCLK_STATUS & 0x80);
//GCLK_GENDIV = clkDiv | 0x300; //Set clock generator divider to 1;

DFLL48M_to_core_init();
}



/******************************************************************************
 * void DFLL48M_to_core_init(void)
 *
 *  FUNCTION:  Initialize the DFLL48M with external 32k crystal as reference
 *              and routes to CLKGEN0 to be used by the core
 ******************************************************************************/
 void DFLL48M_to_core_init(void)
 {


//Generic Clock 0 (denoted here as GCLK_CLKCTRL_ID_DFLL48) is dedicated to the DFLL48M. Enable this
// clock with CLKGEN2 (this is the one taking in the XOSC32K) as source.
// The generic clock must be configured by performing a single 16-bit write to the Generic Clock Control
// register (CLKCTRL)
    GCLK_CLKCTRL = 0x4200;
    while(GCLK_STATUS & 0x80);
//Enable the DFLL48M in open loop mode. Without this step, attempts to go into closed loop mode at 48 MHz
// will result in Processor Reset.  PCLKSR.DFLLRDY must be one before writing to the DFLL Control register
// note that the DFLLRDY bit represents status of register synchronization - NOT clock stability
    while(!(SYSCTRL_PCLKSR & 0x10));
    SYSCTRL_DFLLCTRL = 0x0002;
    while(!(SYSCTRL_PCLKSR & 0x10));
//Load factory calibrated values into DFLLVAL.  This value is located in calibration flash.
// Without loading this value you get ~25 MHz in open loop mode.  Note that this value is loaded AFTER
// setting the DFLL48M ENABLE bit. Attempts to write this register before enabling
// the DFLL48M results in a processor RESET
    uint8_t DFLL48M_Course_Cal = (*((volatile uint8_t*)CALIBRATION+7));
    DFLL48M_Course_Cal &= 0b11111100;
    DFLL48M_Course_Cal >>= 2;
    uint32_t courseCal = ((uint32_t)DFLL48M_Course_Cal<<10);
    SYSCTRL_DFLLVAL = courseCal;

//Set up the Multiplier to be just below the 48MHZ frequency.  1465 * 32KHZ is 46.880MHZ.  The step sizes
// signify the maximum steps for the frequency adjustment
   SYSCTRL_DFLLMUL = 0x000005B9 | 0x1C000000 | 0x00FF0000;

  

//Switch DFLL48M to Closed Loop and enable WAITLOCK
   while(!(SYSCTRL_PCLKSR & 0x10));
   SYSCTRL_DFLLCTRL |= 0x0C06;

   while(!(SYSCTRL_PCLKSR & 0x80)); //Wait for course lock

   while(!(SYSCTRL_PCLKSR & 0x40)); //Wait for fine lock

//Set wait states for 48 MHZ operation
    NVMCTRL_CTRLB  = 0x4;

//Now that DFLL48M is running, switch CLKGEN0 source to it to run the core at 48 MHz.
    while(GCLK_STATUS & 0x80);

   // GCLK_GENCTRL = (uint32_t)(GCLK_GENCTRL_ID(0) | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_IDC | GCLK_GENCTRL_OE);
    GCLK_GENCTRL = 0x000B0700;
}
