#include "stdint.h"
#include "clocks.h"
#include "adc.h"
#include "events.h"
#include "gpio.h"
#include "sercom0.h"
#include "tcc.h"
#include "samd10d13as.h"

//extern volatile uint16_t adcReadings[maxAdcIndex];

extern volatile uint16_t adcReadings[maxAdcIndex];

int main (void){
int i=0;
uint16_t results;
uint8_t data;
//uint8_t intEn;
//uint8_t intFlg;
initGpio();
initClocks();
initAdc();
//initSerial()
initTcc();
initSercom0();

    NVIC_ISER |= 0x8000;
    ADC_INTENSET |= 0x01; //Enable the ADC interrupt.
    init_Events();
	//	ADC_SWTRIG |= 0x02;

    while(1){
        i++;
        if(i > 2000000){
            i=0;
            toggleLed();
	    //results = adcReadings[0];
            //results = adcReadings[1];
            //results++;
//            sendByte(0x55);
        }

       if(SERCOM_INTFLAG(0) & 0x04){
           data = readByte();
           sendByte(data);
       } 
    }
return 0;
}
