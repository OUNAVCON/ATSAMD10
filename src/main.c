#include "stdint.h"
#include "clocks.h"
#include "adc.h"
#include "gpio.h"
#include "samd10d13as.h"


int main (void){
int i=0;
uint16_t results;
uint8_t intEn;
uint8_t intFlg;
initGpio();
initClocks();
initAdc();
//initSerial()
//initPwm
    NVIC_ISER |= 0x8000;


    while(1){
        i++;
        if(i > 2000000){
                i=0;
                toggleLed();
                
                if(!(ADC_INTENSET & 0x01)){
                    while(ADC_STATUS & 0x80);
                    ADC_INPUTCTRL = 0x0F001805;//Select Channel 5 - Write Sync'd
                    while(ADC_STATUS & 0x80);
                    ADC_SWTRIG |= 0x02; //Write Sync'd
                    while(ADC_STATUS & 0x80);
                    //while(!(ADC_INTFLAG & 0x01)); //Wait for conversion to finish.
                    //while(ADC_STATUS & 0x80);
                    //ADC_INTFLAG |= 0x01; //Clear the flag.
                    ADC_INTENSET |= 0x01;
                    //Read the result.
                    //while(ADC_STATUS & 0x80);
                    //intEn = ADC_INTENSET;
                    //intFlg = ADC_INTFLAG;
                    //results = ADC_RESULT; // Read Sync'd
                    //results = results+1;
                }
        }
    }
return 0;
}
