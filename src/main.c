#include "stdint.h"
#include "clocks.h"
#include "adc.h"
#include "events.h"
#include "gpio.h"
#include "tcc.h"
#include "samd10d13as.h"


int main (void){
int i=0;
//uint16_t results;
//uint8_t intEn;
//uint8_t intFlg;
initGpio();
initClocks();
initAdc();
//initSerial()
init_Tcc();


    NVIC_ISER |= 0x8000;
    ADC_INTENSET |= 0x01; //Enable the ADC interrupt.
    init_Events();
		ADC_SWTRIG |= 0x02;

    while(1){
        i++;
        if(i > 2000000){
                i=0;
                toggleLed();
        }
    }
return 0;
}
