#include "stdint.h"
#include "clocks.h"
#include "gpio.h"
#include "samd10d13as.h"


int main (void){
int i=0;
/*
 * Calibration data.
 * 34:27 ADC Linearity CALIB offset: 0x28 bits: 0-7
 * 37:35 ADC_Bias_Cal CALIB offset: 0x28 bits: 8-10
 * 44:38 OSC 32k Cal OSC32K offset 0x18, bits: 16-22
 * 63:58 DFLL 48M Course Cal //Loaded into DFLL48M Value offset 0x28
 */
uint16_t ADC_Cal = (*((volatile uint16_t*)CALIBRATION+3));
ADC_Cal &= 0b11111111111000;
ADC_Cal >>= 3;
uint16_t OSC32K_Cal = (*((volatile uint16_t*)CALIBRATION+4));
OSC32K_Cal &= 0b1111111000000;
OSC32K_Cal >>= 6;
//uint8_t DFLL48M_Course_Cal = (*((volatile uint8_t*)CALIBRATION+7));
//DFLL48M_Course_Cal &= 0b11111100;
//DFLL48M_Course_Cal >>= 2;
//PAC0_WPCLR = 0xFFFFFFFF;
//PAC1_WPCLR = 0xFFFFFFFF;
//PAC2_WPCLR = 0xFFFFFFFF;
initGpio();
initClocks();
//initAdc()
//initSerial()
//initPwm



    while(1){
        i++;
	if(i > 2000000){
            i=0;
            toggleLed(); 
	}
    }
return 0;
}
