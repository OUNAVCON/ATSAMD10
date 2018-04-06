#include "samd10d13as.h"
#include "clocks.h"

void initClocks(void){


//Disable the Watch Dog.
WDT_CTRL = 0x00;

while(GCLK_STATUS & 0x80); //Wait till the close has sync'd which is value 0, if 1 then sync started.
GCLK_GENCTRL = 0x00010600;//src is osc8m & enable and id=0

while(GCLK_STATUS & 0x80); //Wait till the close has sync'd which is value 0, if 1 then sync started.
GCLK_GENCTRL = 0x00010604;

SYSCTRL_OSC8M &= 0xFFFFFCFF; //Set the prescalar to 0 which is divide by 1.
GCLK_GENDIV = 0; //Set clock generator divider to 1;
/*
  1. enable lower power osc32.
  2. Enble clock 1 with LPOSC32 as input.
  3. Enable DFLL with clock 1 as input.
  4. Config DFLL.
  5. Wait for lock.
  6. Set Clock 0 with DFLL as source.
*/

//Init Instructions.
/*
bits(31:24) - reserved(00000000)
bits(23:8) - Div[15:0](0000000000000000)
bits (7:4) - reserved (000)
bits(3:0) - ID[3:0](0001)
*/
//GCLK_GENDIV = 0x00000001;

/*
bits(31:22) - Reserved(0000000000)
bit(21) - RUNSTDBY(0)
bit(20) - DIVSEL(0)
bit(19) - OE(1) Output Enable
bit(18) - OOV(0)
bit(17) - IDC(0)
bit(16) - GENEN(1) Enabled
bit(15:13) - Reserved (000)
bit(12:8) - SRC[4:0] (0x03) OSCULP32K
bit(7:4) - Reserved (0000)
bit(3:0) - ID[3:0] (0001)  Clock Gen 1
*/ 
//GCLK_GENCTRL = 0x00010301;

/*
bit(15) - WRTLOCK(0)
bit(14) - CLKEN(1)
bit(13:12) - Reserved(00)
bit(11:8) - GEN[3:0] (0001)
bit(7:6) - Reserved(00)
bit(5:0) - ID[5:0] (000001)
*/
//GCLK_CLKCTRL = 0x00000000;


/*******************************************************************************
* uint32_t get_DFLL48M_coarse_cal(void)
*
* FUNCTION:  Gets factory calibrated value for "DFLL48M COARSE CAL" from NVM
*               Software Calibration Area
*******************************************************************************/
/*uint32_t get_DFLL48M_coarse_cal(void)
{
    uint32_t tempCalibCoarse;

    tempCalibCoarse = *(uint32_t*)FUSES_DFLL48M_COARSE_CAL_ADDR;
    tempCalibCoarse &= FUSES_DFLL48M_COARSE_CAL_Msk;
    tempCalibCoarse = tempCalibCoarse>>FUSES_DFLL48M_COARSE_CAL_Pos;
    return(tempCalibCoarse);
}


******************************************************************************
 * void DFLL48M_to_core_init(void)
 *
 *  FUNCTION:  Initialize the DFLL48M with external 32k crystal as reference
 *              and routes to CLKGEN0 to be used by the core
 ******************************************************************************
 void DFLL48M_to_core_init(void)
 {
// XOSC32k takes the longest to settle - turn it on first to get it started
    SYSCTRL->XOSC32K.reg = (uint16_t)(SYSCTRL_XOSC32K_STARTUP(2) | SYSCTRL_XOSC32K_RUNSTDBY | SYSCTRL_XOSC32K_AAMPEN
                                | SYSCTRL_XOSC32K_EN32K | SYSCTRL_XOSC32K_XTALEN | SYSCTRL_XOSC32K_ENABLE);

//Set the clock generator 3 divider to 1
    REG_GCLK_GENDIV = GCLK_GENDIV_DIV(1) | GCLK_GENDIV_ID(3);

//Enable the Clock Generator 3 with XOSC32k as source - this will be the reference for the DFLL48M.
    while(GCLK->STATUS.bit.SYNCBUSY);
    GCLK->GENCTRL.reg = (uint32_t)(GCLK_GENCTRL_ID(3) | GCLK_GENCTRL_SRC_XOSC32K| GCLK_GENCTRL_GENEN
                            | GCLK_GENCTRL_IDC | GCLK_GENCTRL_OE);

//Generic Clock 0 (denoted here as GCLK_CLKCTRL_ID_DFLL48) is dedicated to the DFLL48M. Enable this
// clock with CLKGEN3 (this is the one taking in the XOSC32K) as source.
// The generic clock must be configured by performing a single 16-bit write to the Generic Clock Control
// register (CLKCTRL)
    GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_GEN(3) | GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_ID_DFLL48);

//Enable the DFLL48M in open loop mode. Without this step, attempts to go into closed loop mode at 48 MHz
// will result in Processor Reset.  PCLKSR.DFLLRDY must be one before writing to the DFLL Control register
// note that the DFLLRDY bit represents status of register synchronization - NOT clock stability
    while(!SYSCTRL->PCLKSR.bit.DFLLRDY);
    SYSCTRL->DFLLCTRL.reg = (uint16_t)(SYSCTRL_DFLLCTRL_ENABLE);

//Load factory calibrated values into DFLLVAL.  This value is located in calibration flash.
// Without loading this value you get ~25 MHz in open loop mode.  Note that this value is loaded AFTER
// setting the DFLL48M ENABLE bit. Attempts to write this register before enabling
// the DFLL48M results in a processor RESET
    SYSCTRL->DFLLVAL.bit.COARSE = get_DFLL48M_coarse_cal();

//Wait for XOSC32K to stabilize
    while(!SYSCTRL->PCLKSR.bit.XOSC32KRDY);

//Set up the Multiplier to be just below the 48MHZ frequency.  1465 * 32KHZ is 46.880MHZ.  The step sizes
// signify the maximum steps for the frequency adjustment
    SYSCTRL->DFLLMUL.reg = (uint32_t)(SYSCTRL_DFLLMUL_MUL(1465) | SYSCTRL_DFLLMUL_CSTEP(7) | SYSCTRL_DFLLMUL_FSTEP(15));

//Switch DFLL48M to Closed Loop and enable WAITLOCK
    while(!SYSCTRL->PCLKSR.bit.DFLLRDY);
    SYSCTRL->DFLLCTRL.reg |= (uint16_t) (SYSCTRL_DFLLCTRL_MODE | SYSCTRL_DFLLCTRL_WAITLOCK);

//Set wait states for 48 MHZ operation
    NVMCTRL->CTRLB.bit.RWS = 3;

//Now that DFLL48M is running, switch CLKGEN0 source to it to run the core at 48 MHz.
    while(GCLK->STATUS.bit.SYNCBUSY);
    GCLK->GENCTRL.reg = (uint32_t)(GCLK_GENCTRL_ID(0) | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_GENEN
                            | GCLK_GENCTRL_IDC | GCLK_GENCTRL_OE);

}*/

}
