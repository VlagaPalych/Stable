#include "telemetry.h"
#include "string.h"
#include "stm32f4xx.h" 

uint8_t USART1_TX = 9;  // PA
uint8_t USART1_RX = 10; // PA

Message message;

extern uint8_t Message_Size;
#define MESSAGE_HEADER  0x21

uint8_t tele[100];
uint8_t tele_len;

uint8_t received = 0;

uint8_t telemetry_on = 0;

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

    USART1->BRR     =  0x8b; //0x271;    // 115.2 Kb/s, Fclk = 72MHz
    USART1->CR3     |= USART_CR3_DMAT;
    //USART1->CR2     |= USART_CR2_STOP_1;
    USART1->CR1     |= (1 << 12) | USART_CR1_PCE | USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE; 
    NVIC_EnableIRQ(USART1_IRQn); 
    NVIC_SetPriority(USART1_IRQn, 0x02); 
    Telemetry_DMA_Init();
}

void USART1_Send(uint8_t data) {
    while((USART1->SR & USART_SR_TXE)==0); 
    USART1->DR = data; 
}


void USART1_IRQHandler() {
    if (USART1->SR & USART_SR_RXNE) {
        USART1->SR &= ~USART_SR_RXNE;
        received = USART1->DR;
        
        if (received == 'h') {
            telemetry_on ^= 1;
        }
    }
}

void Telemetry_DMA_Init() {
    NVIC_EnableIRQ(DMA2_Stream7_IRQn);
    NVIC_SetPriority(DMA2_Stream7_IRQn, 0x02);      
        
    DMA2_Stream7->PAR = (uint32_t)&(USART1->DR);
    DMA2_Stream7->CR = DMA_SxCR_CHSEL_2 | DMA_SxCR_DIR_0 | DMA_SxCR_MINC | DMA_SxCR_PL | DMA_SxCR_TCIE;
}

void Telemetry_DMA_Run() {
    DMA2_Stream7->M0AR = (uint32_t)(tele);
    DMA2_Stream7->NDTR = tele_len;
    DMA2_Stream7->CR |= DMA_SxCR_EN;
}

void DMA2_Stream7_IRQHandler() {
    if (DMA2->HISR & DMA_HISR_TCIF7) {
        DMA2->HIFCR = DMA_HIFCR_CTCIF7 | DMA_HIFCR_CHTIF7;
        DMA2_Stream7->CR &= ~DMA_SxCR_EN;
    }
}

void Telemetry_Send(Message *msg) {
    if (telemetry_on) {
        Message_ToByteArray(msg, tele);
        tele_len = Message_Size+2;

        Telemetry_DMA_Run();
    }
}



