#include "telemetry.h"
#include "string.h"
#include "stm32f30x.h"

uint8_t USART1_TX = 9;  // PA
uint8_t USART1_RX = 10; // PA

Message message;

extern uint8_t Message_Size;
#define MESSAGE_HEADER  0x21

uint8_t tele[100];
uint8_t tele_len;

uint8_t received = 0;

void Message_ToByteArray(Message *message, uint8_t *a) {
    uint8_t i = 0, crc = 0;
    memcpy(a+1, (uint8_t *)message, Message_Size);
    a[0] = MESSAGE_HEADER;
    crc = a[0];
    for (i = 1; i < Message_Size+1; i++) {
        crc ^= a[i];
    }
    a[Message_Size+1] = crc;
}

uint8_t Message_FromByteArray(uint8_t *a, uint8_t n, Message *message) {
    uint8_t i = 0, crc = 0;
    
    crc = a[0];
    for (i = 1; i < Message_Size+1; i++) {
        crc ^= a[i];
    }
    if ((a[0] == MESSAGE_HEADER) && (a[Message_Size+1] == crc)) {
        memcpy((uint8_t *)message, a+1, Message_Size);
        return 1;
    } 
    return 0;
}


void USART1_Init() {
    GPIOA->MODER    |= (2 << USART1_TX*2) | (2 << USART1_RX*2);
    GPIOA->OTYPER   &= ~((1 << USART1_TX) | (1 << USART1_RX));
    GPIOA->OSPEEDR  |= (3 << USART1_TX*2) | (3 << USART1_RX*2);
    GPIOA->PUPDR    |= (1 << USART1_TX*2) | (1 << USART1_RX*2);
    GPIOA->AFR[1]   |= (7 << (USART1_TX-8)*4) | (7 << (USART1_RX-8)*4);

    USART1->BRR     =  0x271;    // 115.2 Kb/s, Fclk = 72MHz
    USART1->CR3     |= USART_CR3_DMAT;
    //USART1->CR2     |= USART_CR2_STOP_1;
    USART1->CR1     |= (1 << 12) | USART_CR1_PCE | USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE; 
    NVIC_EnableIRQ(USART1_IRQn); 
    Telemetry_DMA_Init();
}

void send_to_uart(uint8_t data) {
    while((USART1->ISR & USART_ISR_TXE)==0); 
    USART1->TDR = data; 
}


void USART1_IRQHandler() {
    if (USART1->ISR & USART_ISR_RXNE) {
        USART1->ISR &= ~USART_ISR_RXNE;
        received = USART1->RDR;
    }
}

void Telemetry_DMA_Init() {
    NVIC_EnableIRQ(DMA1_Channel4_IRQn);    
        
    DMA1_Channel4->CPAR = (uint32_t)&(USART1->TDR);
    DMA1_Channel4->CCR = DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_PL | DMA_CCR_TCIE;
}

void Telemetry_DMA_Run() {
    DMA1_Channel4->CMAR = (uint32_t)(tele);
    DMA1_Channel4->CNDTR = tele_len;
    DMA1_Channel4->CCR |= DMA_CCR_EN;
}

void DMA1_Channel4_IRQHandler() {
    if (DMA1->ISR & DMA_ISR_TCIF4) {
        DMA1->IFCR |= DMA_IFCR_CTCIF4 | DMA_IFCR_CHTIF4;
        DMA1_Channel4->CCR &= ~DMA_CCR_EN;
    }
}

void Telemetry_Send(Message *msg) {
    Message_ToByteArray(msg, tele);
    tele_len = Message_Size+2;

    Telemetry_DMA_Run();
}



