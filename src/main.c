#include "stdint.h"
#include "clocks.h"
#include "gpio.h"
#include "samd10d13as.h"

int main (void){
int i=0;

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
