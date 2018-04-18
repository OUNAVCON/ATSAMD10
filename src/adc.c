/*
 * adc.c
 *
 *  Created on: Apr 12, 2018
 *      Author: Isaac.Rose
 */
#include "adc.h"
#include "samd10d13as.h"

void initAdc(){
    GCLK_CLKCTRL = 0x4113; //Set ADC to use OSC1, divide by 1
    while(GCLK_STATUS & 0x80);
    ADC_DBGCTRL = 0x01;
    ADC_REFCTRL = 0x02; //Ref is 1/2 VDDANA
    ADC_SAMPCTRL = 0x05;
    while(ADC_STATUS & 0x80);
    ADC_CTRLB = 0x0100; //Write Sync'd, clock/128, 12bit conversion,
    while(ADC_STATUS & 0x80);
	 // ADC_OFFSETCORR = 0x0EFF;

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
    ADC_CALIB = ADC_Cal;
    ADC_CTRLA = 0x02; //Enable the ADC. //Sync Busy.
   while(ADC_STATUS & 0x80);
}

void adc_ISR(void){
    int16_t results = 0;
    ADC_INTENCLR |= 1;
    ADC_INTFLAG |= 1;
    while(ADC_STATUS & 0x80);
    results = ADC_RESULT;
    results++;
}
