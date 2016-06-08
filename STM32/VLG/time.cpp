#include "time.h"
#include "stm32f4xx.h"

// < 65536 us
extern "C" void delay_us(uint16_t us) {
    TIM6->PSC = 15;
    TIM6->ARR = us;
    TIM6->EGR = TIM_EGR_UG;
    TIM6->CR1 |= TIM_CR1_OPM | TIM_CR1_CEN;
    while ((TIM6->CR1 & TIM_CR1_CEN)!=0);
}

#define TICK_FREQ (1000u)

volatile uint32_t g_ul_ms_ticks = 0;
static volatile uint32_t TimingDelay = 0;
unsigned long idle_time = 0;
extern uint32_t SystemCoreClock; //16000000=16Mhz (original value)

extern "C" void SysTick_Init() {
    SystemCoreClockUpdate();                               // Update the system clock variable (might not have been set before)
                                                           // With this call, the core clock gets set to 56MHz
    
	if (SysTick_Config (SystemCoreClock / TICK_FREQ)) {     // Setup SysTick Timer for 1 msec interrupts
		while (1);                                          // Handle Error
	}
}

extern "C" void delay_ms(uint32_t nTime)
{
	TimingDelay = nTime;
	while(TimingDelay != 0);
}

extern "C" uint32_t millis()
{
	return g_ul_ms_ticks;
}

extern "C" void TimingDelay_Decrement(void)
{
	if (TimingDelay != 0x00)
		TimingDelay--;
}

extern "C" void TimeStamp_Increment(void)
{
	g_ul_ms_ticks++;
}

extern "C" void SysTick_Handler(void)
{
  	TimingDelay_Decrement();
    TimeStamp_Increment();
}
