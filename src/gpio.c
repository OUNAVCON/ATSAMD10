#include "samd10d13as.h"
#include "gpio.h"

static void initUsart(void);
static void initAdcGpio(void);
static void initTccGpio(void);
void initGpio(){
    PORT_DIRSET(PORTA) = 0x40; 
    initUsart();
    initAdcGpio();
    initTccGpio();
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
    //ADC AIN_5 PA07 mux B - Position Voltage
    //ADC AIN_4 PA06 mux B - Motor Current
        PORT_MUX(PORTA,6) |= 0x10; //Upper Nibble is PA7 Lower Nibble is PA6, 0x1 is ADC
        //PORT_PINCFG(PORTA,6) |= 0x01; //use pin mux value
        PORT_PINCFG(PORTA,7) |= 0x01; //use pin mux value

}

static void initTccGpio(void){

       PORT_DIRSET(PORTA) |= 0x0300; //PA08 (TCC_w[2]) & PA09 (TCC_w[3]) //Set Output
       PORT_MUX(PORTA,8) |= 0x44; //Upper Nibble is PA09 Lower Nibble is PA08, 0x4 is ADC (Mux E)
       PORT_PINCFG(PORTA,8) |= 0x01; //use pin mux value
       PORT_PINCFG(PORTA,9) |= 0x01; //use pin mux value
}
