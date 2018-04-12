#include "stdint.h"
#include "clocks.h"
#include "adc.h"
#include "gpio.h"
#include "samd10d13as.h"


int main (void){
int i=0;
int16_t results;
initGpio();
initClocks();
initAdc();
//initSerial()
//initPwm



    while(1){
        i++;
        if(i > 2000000){
                i=0;
                toggleLed();
                while(ADC_STATUS & 0x80);
                ADC_INPUTCTRL = 0x00000005;//Select Channel 5 - Write Sync'd
                while(ADC_STATUS & 0x80);
                ADC_SWTRIG |= 0x02; //Write Sync'd
                while(ADC_STATUS & 0x80);
                while(!(ADC_INTFLAG & 0x01)); //Wait for conversion to finish.
                while(ADC_STATUS & 0x80);
                ADC_INTFLAG |= 0x01; //Clear the flag.
                //Read the result.
                while(ADC_STATUS & 0x80);
                results = ADC_RESULT; // Read Sync'd
        }
    }
return 0;
}
