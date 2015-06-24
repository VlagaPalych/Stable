#include "stm32f4xx.h"

uint8_t UART2_TX = 2;
uint8_t UART2_RX = 3;

void Delay(void) {
  volatile uint32_t i;
  for (i=0; i != 0x70000; i++);
}
 
void send_to_uart(uint8_t data) {
  while(!(USART2->SR & USART_SR_TC)); 
  USART2->DR=data; 
}
 
int main(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    GPIOA->MODER |= (2 << UART2_TX*2) | (2 << UART2_RX*2);
    GPIOA->OTYPER &= ~((1 << UART2_TX) | (1 << UART2_RX));
    GPIOA->OSPEEDR |= (3 << UART2_TX*2) | (3 << UART2_RX*2);
    GPIOA->PUPDR |= (1 << UART2_TX*2) | (1 << UART2_RX*2);
    GPIOA->AFR[0]|= (7 << UART2_TX*4) | (7 << UART2_RX*4);

    USART2->BRR = 0x683; 
    USART2->CR1 |= USART_CR1_UE | USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE; 
    NVIC_EnableIRQ(USART2_IRQn);
    
  while(1) {
//    send_to_uart(0x11);
//    send_to_uart(0x22);
//    send_to_uart(0x33);
//    send_to_uart(0x44);
//    send_to_uart(0x55);
//    send_to_uart(0x66);
//    send_to_uart(0x77);
//    send_to_uart(0x88);
    //send_to_uart('\n');
//    Delay(); 
  }
}

uint8_t received = 0;

void USART2_IRQHandler() {
    if (USART2->SR & USART_SR_RXNE) {
        received = USART2->DR;
    }
}
