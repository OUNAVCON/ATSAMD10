#include "samd10d13as.h"
#include "clocks.h"

void initClocks(void){
//Disable the Watch Dog.
WDT_CTRL = 0x00;
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
}
