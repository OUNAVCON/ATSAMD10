#include "samd10d13as.h"
#include "gpio.h"

void initGpio(){
    PORT_DIRSET(PORTA) = 0x40 | 0x80; //PA06 (0x40) PA08(0x80)
 //   PORT_DIRSET(PORTA) |= 0x80; //PA08
    PORT_MUX(PORTA,8) = 0x07;//Function H => GCLK_IO[0]
    PORT_PINCFG(PORTA,8) |= 0x1;
}

void toggleLed(){
    PORT_OUTTGL(PORTA) = 0x40;
}
