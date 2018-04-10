#include "samd10d13as.h"
#include "gpio.h"

static void initUsart(void);


void initGpio(){
    PORT_DIRSET(PORTA) = 0x40 | 0x80; //PA06 (0x40) PA08(0x80)
    initUsart();
}

void toggleLed(){
    PORT_OUTTGL(PORTA) = 0x40;
}

static void initUsart(void){
//SERCOM0, PA4 pad[0] and PA5 pad[1]
    PORT_DIRSET(PORTA) |= 0x30; //PA04 and PA05
    PORT_MUX(PORTA,4) |= 0x33; //Upper Nibble is PA5 Lower Nibble is PA4, 0x3 is Sercom0
    PORT_PINCFG(PORTA,4) |= 0x01; //use pin mux value
    PORT_PINCFG(PORTA,5) |= 0x01; //use pin mux value
}
