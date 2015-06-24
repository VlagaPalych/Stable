#include "ws2812.h"
#include "stm32f30x.h"
#include "string.h"
#include "timer.h"

uint8_t SPI2_SCK = 13;
uint8_t SPI2_MISO = 14;
uint8_t SPI2_MOSI = 15;
uint8_t SPI2_NSS = 12;

#define LOW     0xF000
#define HIGH    0xFF00
#define RES     0x0000

#define BITS_PER_LED    24
#define SPI_ARRAY_SIZE  LED_NUMBER*BITS_PER_LED

uint32_t COLORS[LED_NUMBER];
uint16_t SPI_ARRAY[SPI_ARRAY_SIZE];
    
void SPI2_GPIO_Init() {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	
	GPIOB->MODER 	|= /*(2 << SPI2_SCK*2) | (2 << SPI2_MISO*2) |*/ (2 << SPI2_MOSI*2) /*| (1 << SPI2_NSS*2)*/;
	GPIOB->OSPEEDR 	|= /*(3 << SPI2_SCK*2) | (3 << SPI2_MISO*2) |*/ (3 << SPI2_MOSI*2) /*| (3 << SPI2_NSS*2)*/;
	GPIOB->AFR[1] 	|= /*(5 << (SPI2_SCK-8)*4) | (5 << (SPI2_MISO-8)*4) |*/ (5 << (SPI2_MOSI-8)*4);                         // AF5
	GPIOB->OTYPER	&= ~(/*(1 << SPI2_SCK) | (1 << SPI2_MISO) |*/ (1 << SPI2_MOSI) /*| (1 << SPI2_NSS)*/); 
}

void SPI2_Init() {
    SPI2_GPIO_Init();
    
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
	
	SPI2->CR1 = 0;
	SPI2->CR2 = SPI_CR2_DS_3 |SPI_CR2_DS_2 |SPI_CR2_DS_1 | SPI_CR2_DS_0;        // 16 bits
	
    SPI2->CR1 |= SPI_CR1_BIDIMODE;
    SPI2->CR1 |= SPI_CR1_BIDIOE;
	SPI2->CR1 |= SPI_CR1_BR_0; 							                        // baudrate = Fpclk / 4
	SPI2->CR1 |= SPI_CR1_CPOL;													// polarity
	SPI2->CR1 |= SPI_CR1_CPHA;													// phase	
	SPI2->CR1 &= ~(SPI_CR1_LSBFIRST);										    // MSBFIRST		
	SPI2->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;										// Software slave management		
	//SPI1->CR2 |= SPI_CR2_SSOE;
    SPI2->CR2 |= SPI_CR2_TXDMAEN;
	SPI2->CR1 |= SPI_CR1_MSTR;													// Master configuration	
	SPI2->CR1 |= SPI_CR1_SPE;                                                   // Enable SPI
}

void WS2812_Convert(uint32_t color, uint16_t *out) {
    int i = 0; 
    for (i = 0; i < BITS_PER_LED; i++) {
        if (color & (1 << (BITS_PER_LED - i - 1))) {
            out[i] = HIGH;
        } else {
            out[i] = LOW;
        }
    }
}

void WS2812_ConvertArray(uint32_t *colors, uint32_t colorsSize, uint16_t *out) {
    int j;
    for (j = 0; j < colorsSize; j++) {
        WS2812_Convert(colors[j], out + 24*j);
    }
}

void DMA_Init() {
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    DMA1_Channel5->CCR      = 0;
    DMA1_Channel5->CPAR     = (uint32_t)&(SPI2->DR);
    DMA1_Channel5->CMAR     = (uint32_t)SPI_ARRAY;
    DMA1_Channel5->CNDTR    = SPI_ARRAY_SIZE;
    DMA1_Channel5->CCR      = DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_PSIZE_0 |DMA_CCR_MSIZE_0 |
                                DMA_CCR_PL | DMA_CCR_EN;
}

void WS2812_Init() {
//    int i = 0;
//    //Timer_Init();
//    
//    memset(COLORS, 0x00000000, LED_NUMBER*sizeof(uint32_t));
//   
//    
//    //Delay(10000);
//     for (i  = 0; i < LED_NUMBER; i++) {
//        COLORS[i] = 0x000f00;
//    }
//    WS2812_ConvertArray(COLORS, LED_NUMBER, SPI_ARRAY);DMA_Init();   
    SPI2_Init();
 //   DMA_Init();   
   // SPI2_Init();
}

uint32_t WS2812_RGB(uint32_t red, uint32_t green, uint32_t blue) {
    return (green << 16) | (red << 8) | blue;
}


void WS2812_Label(uint8_t led, uint8_t labelLength, uint32_t labelColor) {
    uint8_t i = 0;
    uint8_t firstLabelLED   = SKIP_LEDS + led - (labelLength + 1) / 2 + 1;
    uint8_t lastLabelLED    = SKIP_LEDS + led + labelLength / 2;

    memset(COLORS, 0x00000000, LED_NUMBER*sizeof(uint32_t));
    for (i = firstLabelLED; i <= lastLabelLED; i++) {
        COLORS[i] = labelColor;
    }
    WS2812_ConvertArray(COLORS, LED_NUMBER, SPI_ARRAY);
    DMA_Init();
}

void WS2812_LabelWithTail(uint8_t led, uint8_t labelLength, uint32_t labelColor, uint8_t tailLeft, uint8_t tailLength, uint32_t tailColor) {
    uint8_t i = 0;
    uint8_t firstLabelLED   = SKIP_LEDS + led - (labelLength + 1) / 2 + 1;
    uint8_t lastLabelLED    = SKIP_LEDS + led + labelLength / 2;
    uint8_t firstTailLED    = tailLeft ? (firstLabelLED - tailLength) : (lastLabelLED + 1);
    uint8_t lastTailLED     = tailLeft ? (firstLabelLED - 1) : (lastLabelLED + tailLength);

    memset(COLORS, 0x00000000, LED_NUMBER*sizeof(uint32_t));
    for (i = firstLabelLED; i <= lastLabelLED; i++) {
        COLORS[i] = labelColor;
    }
    for (i = firstTailLED; i <= lastTailLED; i++) {
        COLORS[i] = tailColor;
    }
    WS2812_ConvertArray(COLORS, LED_NUMBER, SPI_ARRAY);
    DMA_Init();
}

void WS2812_LabelWithGradientTail(uint8_t led, uint8_t labelLength, uint32_t labelColor, uint8_t tailLeft, uint8_t tailLength, uint32_t tailColor) {
    uint8_t i = 0;
    uint8_t firstLabelLED   = SKIP_LEDS + led - (labelLength + 1) / 2 + 1;
    uint8_t lastLabelLED    = SKIP_LEDS + led + labelLength / 2;
    uint8_t firstTailLED    = tailLeft ? (firstLabelLED - tailLength) : (lastLabelLED + 1);
    uint8_t lastTailLED     = tailLeft ? (firstLabelLED - 1) : (lastLabelLED + tailLength);

    memset(COLORS, 0x00000000, LED_NUMBER*sizeof(uint32_t));
    for (i = firstLabelLED; i <= lastLabelLED; i++) {
        COLORS[i] = labelColor;
    }
    for (i = firstTailLED; i <= lastTailLED; i++) {
        COLORS[i] = tailColor * (lastTailLED - i + 1) / tailLength;
    }
    WS2812_ConvertArray(COLORS, LED_NUMBER, SPI_ARRAY);
    DMA_Init();
}

void WS2812_StableLabel(uint8_t led, uint32_t color1, uint32_t color2) {
    uint8_t firstLabelLED   = SKIP_LEDS + led;
    uint8_t secondLabelLED  = firstLabelLED + 1;
    
    memset(COLORS, 0x00000000, LED_NUMBER*sizeof(uint32_t));
    COLORS[firstLabelLED] = color1;
    COLORS[secondLabelLED] = color2;
    WS2812_ConvertArray(COLORS, LED_NUMBER, SPI_ARRAY);
    DMA_Init();
}
