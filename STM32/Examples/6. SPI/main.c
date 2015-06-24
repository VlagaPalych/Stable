#include "stm32f30x.h"
#include "adxl345.h"
#include "ws2812.h"
#include "timer.h"


int main() {
    
    WS2812_Init();
    ADXL345_Init();
	ADXL345_Calibr();
    EXTI_Init();
    
	while(1) {  
        
	}
}
