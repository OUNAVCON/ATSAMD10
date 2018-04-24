#include "samd10d13as.h"
#include "sercom0.h"



void initSercom0(){
    // Enable Bus Clock
    while(GCLK_STATUS & 0x80);
    GCLK_CLKCTRL = 0x410E; //SERCOM0 Core uses OSC1 (8mhz)
    while(GCLK_STATUS & 0x80);
 //   GCLK_CLKCTRL = 0x410D; //SERCOM slow clock (hz)
 //   while(GCLK_STATUS & 0x80);
    //setup sercom 0 for usart.
    PM_APBCMASK |= 0x00000004;
    SERCOM_CTRLA(0) = 0x50100004;
    while(SERCOM_SYNCBUSY(0) & 0X00000002);
    SERCOM_CTRLB(0) = 0x00030000;
    while(SERCOM_SYNCBUSY(0) & 0X00000004);
    SERCOM_DBGCTRL(0) = 0x01;
    /*
    * BAUD = 416; for 9600
    * BAUD = 208; for 19200
    */
    SERCOM_BAUD(0) = 208;

    while(SERCOM_SYNCBUSY(0) & 0x00000006); //Check for CTRLB //Check for enable
    SERCOM_CTRLA(0) |= 0x00000002; //Enable the sercom.
    while(SERCOM_SYNCBUSY(0) & 0X00000002); //wait till the usart is enabled.
}

void sendByte(uint8_t data){
    SERCOM_DATA(0) = data;
}

uint8_t readByte(void){
    return (SERCOM_DATA(0) & 0x00FF);
}
