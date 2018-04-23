/*
 * adc.c
 *
 *  Created on: Apr 12, 2018
 *      Author: Isaac.Rose
 */
#include "stdint.h"
#include "adc.h"
#include "samd10d13as.h"

static volatile uint16_t adcIndex = 0; //Index of ADC Readings.
static uint8_t adcChannel[maxAdcIndex];
/*
 * [0] - AIN_4 ()
 * [1] - AIN_5 ()
 */
volatile uint16_t adcReadings[maxAdcIndex];

void initAdc(){
		adcChannel[0] = 0x5;
    adcChannel[1] = 0x4;
    GCLK_CLKCTRL = 0x4113; //Set ADC to use OSC1, divide by 1
    while(GCLK_STATUS & 0x80);
    ADC_DBGCTRL = 0x01;
    ADC_REFCTRL = 0x02; //Ref is 1/2 VDDANA
    ADC_SAMPCTRL = 0x05;
    while(ADC_STATUS & 0x80);
    ADC_CTRLB = 0x0100; //Write Sync'd, clock/128, 12bit conversion,
    while(ADC_STATUS & 0x80);
    ADC_EVCTRL = 0x01; //Event Starts a new Conversion
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
    ADC_INPUTCTRL = 0x0F001800 | adcChannel[adcIndex];//Load the first channel to be converted. This will be started once the TCC event is triggered.
}

//ADC triggering is done by event system on TCC TOP (PER reached event)

void adc_ISR(void){
	  uint32_t input = 0;
		uint16_t test = 0;
    while(ADC_STATUS & 0x80);
    test = ADC_RESULT; //Conversion was finished now let's store the results.
		adcReadings[adcIndex] = test;    
    //Setup for the next Channel to be read.
    if(++adcIndex >= maxAdcIndex){ //If the index being incremented pushes it over to - or over the max count then set back to zero.
        adcIndex = 0;
    }
    input =  0x0F001800 | (uint32_t) (adcChannel[adcIndex]);//Select Channel 5 - Write Sync'd
    ADC_INPUTCTRL =  input;
    if(0 != adcIndex){
        while(ADC_STATUS & 0x80);
        ADC_SWTRIG |= 0x02; //Keep taking samples till we get the whole list. The means the index is back to zero.
        while(ADC_STATUS & 0x80);
    } //Now we wait for the next TCC PERiod reached event.

    ADC_INTFLAG |= 1;
}
