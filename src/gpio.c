#include "samd10d13as.h"
#include "gpio.h"

static void initUsart(void);
static void initAdcGpio(void);

void initGpio(){
    PORT_DIRSET(PORTA) = 0x40; 
    initUsart();
    initAdcGpio();
}

void toggleLed(){
    PORT_OUTTGL(PORTA) = 0x40;
}

static void initUsart(void){
//SERCOM0, PA4 pad[0] and PA5 pad[1]
    PORT_DIRSET(PORTA) |= 0x10; //PA04 output and PA05 input
    PORT_MUX(PORTA,4) |= 0x33; //Upper Nibble is PA5 Lower Nibble is PA4, 0x3 is Sercom0
    PORT_PINCFG(PORTA,4) |= 0x01; //use pin mux value
    PORT_PINCFG(PORTA,5) |= 0x01; //use pin mux value
}

static void initAdcGpio(void){
    //ADC AIN_5 PA07 mux B
     //   PORT_DIRSET(PORTA) |= 0x80; //PA07
        PORT_MUX(PORTA,7) |= 0x10; //Upper Nibble is PA5 Lower Nibble is PA4, 0x1 is ADC
        PORT_PINCFG(PORTA,7) |= 0x01; //use pin mux value
}
