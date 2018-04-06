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
uint32_t ADC_Cal = (uint32_t)(*(uint32_t*)(CALIBRATION+3));

//PAC0_WPCLR = 0xFFFFFFFF;
//PAC1_WPCLR = 0xFFFFFFFF;
//PAC2_WPCLR = 0xFFFFFFFF;

initClocks();
//initAdc()
//initSerial()
//initPwm
initGpio();


    while(1){
        i++;
//	if(i > 2){
//            i=0;
toggleLed();
//	}
    }
return 0;
}
