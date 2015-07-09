#include "stm32f4xx.h" 
#include "math.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"

#define G (0xFFFF/4)
#define CTRL_REG4 0x20
#define FREQ100 0x67
#define FREQ800 0x87
#define CTRL_REG3 0x23
#define INT1_EN 0x88
#define PWM_STABLE 2000

// SPI communication pins for accelerometer
uint8_t SPI1_SCK    = 5;    // PA5
uint8_t SPI1_MISO   = 6;    // PA6
uint8_t SPI1_MOSI   = 7;    // PA7
uint8_t SPI1_NSS    = 3;    // PE3
uint8_t ACCEL_INT1  = 0;    // PE0

// Motors control and frequency measurement pins
uint8_t MOT_PWM1    = 12;   // PD12
uint8_t MOT_PWM2    = 14;   // PD14
uint8_t MOT_FREQ1   = 2;    // PE2
uint8_t MOT_FREQ2   = 4;    // PE4

uint8_t BUTTON      = 10;   // PE10

uint8_t UART2_TX    = 2;    // PA2
uint8_t UART2_RX    = 3;    // PA3

uint8_t SERV_PWM    = 14;   // PB14

int16_t ax = 0, ay = 0, az = 0;
int param = 100;
int xoff, yoff, zoff;
int ENGRDY = 0;
int COUNT1 = 0;
int COUNT2 = 0;
uint8_t NEW_PWM_RV = 0;
//int16_t accelRegisters[6] = {0xA800, 0xA900, 0xAA00, 0xAB00, 0xAC00, 0xAD00};
int16_t accelRegisters[6] = {0xA800, 0, 0, 0, 0, 0};
int16_t accel[6];

#define MEASUREMENT_TIME 0.02
uint8_t firstAngleMeasurement = 1;
double angle = 0;
double prevAngle = 0;
double angularVelocity = 0;
int pwm1, pwm2;

double F = 0;
double k1 = 2.15;
double k2 = 1.5e-4;

uint8_t update = 0;

uint8_t received = 0;
char str[100];
int PWM_RX = 0, pwm = 0, i=-1, st = 1;
uint8_t SEND_TELEMETRY_FLAG = 0;

// Identification variables
double y[3];                // 3 last angles
double u[3];                // 3 last thrusts

double w[3];                // unknown coeffs in diff eq
double Afull[10*3];         // matrix of our equation Afull * w = Bfull
double Bfull[10];           // right column

uint8_t anglesAccumulated;  // number of angles we have at the moment
uint8_t row;
//////////////////////

double det3(double *mat)
{
    return mat[0*3 + 0]*(mat[1*3 + 1]*mat[2*3 + 2] - mat[1*3 + 2]*mat[2*3 + 1]) - mat[0*3 + 1]*(mat[1*3 + 0]*mat[2*3 + 2] - mat[1*3 + 2]*mat[2*3 + 0]) +
    mat[0*3 + 2]*(mat[1*3 + 0]*mat[2*3 + 1] - mat[1*3 + 1]*mat[2*3 + 0]);
}

/* A - matrix n x m */
void transpose(double *A, double *At, int n, int m) {
    int i, j;
    for (i = 0; i < n; i++) {
        for (j = 0; j < m; j++) {
            At[j*n + i] = A[i*m + j];
        }
    }
}

/* A - matrix n x k 
   B - matrix k x m */
void mat_mul(double *A, double *B, double *C, int n, int m, int k) {
    int i, j, h;
    for (i = 0; i < n; i++) {
        for (j = 0; j < m; j++) {
            C[i*m + j] = 0;
            for (h = 0; h < k; h++) {
                C[i*m + j] += A[i*k + h] * B[h*m + j];  
            }
        }
    }
}

void system_solve(double *F, double *B, double *x, int n, int m) {
    double *Ft;
    double *A;
    double *b;
    double det, detI;
    double *tmp;
    int i, j, y, z;

    Ft = (double *)malloc(n*m*sizeof(double));
    transpose(F, Ft, n, m);

    A = (double *)malloc(m*m*sizeof(double));
    b = (double *)malloc(m*sizeof(double));

    mat_mul(Ft, F, A, m, m, n);
    mat_mul(Ft, B, b, m, 1, n);

    det = det3(A);

    tmp = (double *)malloc(m*m*sizeof(double));

    for (i = 0; i < m; i++) {
        memcpy(tmp, A, m*m*sizeof(double));
        for (j = 0; j < m; j++) {
            tmp[j*m + i] = b[j];
        }
         
        detI = det3(tmp);
        x[i] = detI / det;
    }

    free(Ft);
    free(A);
    free(b);
    free(tmp);
}


void Delay(void) {
  volatile uint32_t i;
  for (i=0; i != 0x70000; i++);
}
 
void send_to_uart(uint8_t data) {
    while(!(USART2->SR & USART_SR_TC)); 
    USART2->DR=data; 
}
 
void USART_init(void) {
    GPIOA->MODER |= (2 << UART2_TX*2) | (2 << UART2_RX*2);
    GPIOA->OTYPER &= ~((1 << UART2_TX) | (1 << UART2_RX));
    GPIOA->OSPEEDR |= (3 << UART2_TX*2) | (3 << UART2_RX*2);
    GPIOA->PUPDR |= (1 << UART2_TX*2) | (1 << UART2_RX*2);
    GPIOA->AFR[0]|= (7 << UART2_TX*4) | (7 << UART2_RX*4);

    USART2->BRR = 0x341; 
    USART2->CR1 |= USART_CR1_UE | USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE; 
    NVIC_EnableIRQ(USART2_IRQn);
}

void RCC_Init() {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN | RCC_APB2ENR_TIM9EN | RCC_APB2ENR_SPI1EN;;
    
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | 
                    RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_DMA2EN;
    
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN | 
                    RCC_APB1ENR_TIM5EN | RCC_APB1ENR_TIM7EN | RCC_APB1ENR_TIM12EN | RCC_APB1ENR_USART2EN;
}

void SPI_FIRST_INIT(){
    NVIC_EnableIRQ(DMA2_Stream5_IRQn);
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    DMA2_Stream5->M0AR    = (uint32_t)accelRegisters;//???
     
    DMA2_Stream5->NDTR    = 6;
    DMA2_Stream5->PAR     = (uint32_t)&(SPI1->DR);
    DMA2_Stream5->CR      = DMA_SxCR_CHSEL_0 | DMA_SxCR_CHSEL_1 | DMA_SxCR_DIR_0 | DMA_SxCR_MINC | DMA_SxCR_PSIZE_0 | DMA_SxCR_MSIZE_0 |
                                DMA_SxCR_PL | DMA_SxCR_TCIE;
  //  DMA2_Stream5->CR      |= DMA_SxCR_EN;
}


void ADXL345_DMA_Init() {
     GPIOE->BSRRH |= (1 << SPI1_NSS);
 DMA2_Stream0->CR      = 0;
    DMA2_Stream0->PAR     = (uint32_t)&(SPI1->DR);
    DMA2_Stream0->M0AR    = (uint32_t)accel;///???
    DMA2_Stream0->NDTR    = 4;
    DMA2_Stream0->CR      = DMA_SxCR_CHSEL_0 | DMA_SxCR_CHSEL_1 | DMA_SxCR_MINC | DMA_SxCR_PSIZE_0 | DMA_SxCR_MSIZE_0 |
                                DMA_SxCR_TCIE | DMA_SxCR_PL | DMA_SxCR_EN;
    //   DMA2_Stream5->CR      =  0;
    //DMA2->HIFCR=0xC40;
    SPI1->CR2|=3;
    DMA2_Stream5->M0AR    = (uint32_t)accelRegisters;//???
     
    DMA2_Stream5->NDTR    = 4;
  DMA2_Stream5->CR      |= DMA_SxCR_EN;
    
   
}

//double Aappr = -4.596e-7;
//double Bappr = 0.002354;
//double Cappr = -2.154;

double Aappr = 5.13e-7;
double Bappr = 0.001;
double Cappr = -1.81;
double D = 0;
double x1 = 0;
double x2 = 0;

double chooseRoot() {
    if (D < 0) return 2000;
    
    x1 = (-Bappr - sqrt(D)) / (2 * Aappr);
    x2 = (-Bappr + sqrt(D)) / (2 * Aappr);
    
    if (x1 > 1000 && x1 < 2000) return x1;
    if (x2 > 1000 && x2 < 2000) return x2;
    return 2000;
}

uint8_t angleCount = 0;
double angleSum = 0;

void ADXL345_ProcessData(){
//    ((uint8_t *)(&ax))[0] = (uint8_t)(accel[0] & 0xFF);
//    ((uint8_t *)(&ax))[1] = (uint8_t)(accel[1] & 0xFF);
//    ((uint8_t *)(&ay))[0] = (uint8_t)(accel[2] & 0xFF);
//    ((uint8_t *)(&ay))[1] = (uint8_t)(accel[3] & 0xFF);
//    ((uint8_t *)(&az))[0] = (uint8_t)(accel[4] & 0xFF);
//    ((uint8_t *)(&az))[1] = (uint8_t)(accel[5] & 0xFF);
    ax = accel[1];
    ay = accel[2];
    az = accel[3];
    ax -= xoff;
    ay -= yoff;
    az -= zoff;
    angle = atan((double)ax / (double)az);
    
    angleSum += angle;
    angleCount++;
    if (angleCount == 8) {
        angle = angleSum / angleCount; // true angle we work with
        angleSum = 0;
        angleCount = 0;
        
        if (firstAngleMeasurement) {
            firstAngleMeasurement = 0;
        } else {
            angularVelocity = (angle - prevAngle) / MEASUREMENT_TIME;
        }
        prevAngle = angle;
        
        F = k1*angle + k2*angularVelocity;
    
        if (update) {
            if (F > 0) {
                TIM4->CCR1 = 1000;
                pwm1 = 1000;
                D = Bappr*Bappr - 4*Aappr*(Cappr - fabs(F));
                pwm2 = (int)chooseRoot();
                TIM4->CCR3 = pwm2;
            } else if (F < 0) {
                TIM4->CCR3 = 1000;
                pwm2 = 1000;
                D = Bappr*Bappr - 4*Aappr*(Cappr - fabs(F));
                pwm1 = (int)chooseRoot();
                TIM4->CCR1 = pwm1;
            }
            SEND_TELEMETRY_FLAG = 1;
        }
        
        // Identification block
        if (anglesAccumulated < 2) {
            y[anglesAccumulated] = angle;
            u[anglesAccumulated] = F;
        } else {
            y[2] = angle;
            u[2] = F;
            
            row = anglesAccumulated - 2;
            Afull[row*3 + 0] = (y[1] - y[0]) / MEASUREMENT_TIME;
            Afull[row*3 + 1] = y[0];
            Afull[row*3 + 2] = -u[0];
            
            Bfull[row] = (2*y[1] - y[2] - y[0]) / MEASUREMENT_TIME / MEASUREMENT_TIME;
            
            y[0] = y[1];
            y[1] = y[2];
            u[0] = u[1];
            u[1] = u[2];
            
            if (anglesAccumulated == 11) {
                system_solve(Afull, Bfull, w, 10, 3);
                anglesAccumulated = 1;
            }      
        } 
        anglesAccumulated++;
    }
}

void EXTI0_IRQHandler() {
    if (EXTI->PR & EXTI_PR_PR0) {
        EXTI->PR = EXTI_PR_PR0;
       
        ADXL345_DMA_Init();
    }
}

void DMA2_Stream5_IRQHandler() {
    if (DMA2->HISR & DMA_HISR_TCIF5) {
        DMA2->HIFCR = DMA_HIFCR_CTCIF5;
    } 
    if (DMA2->HISR & DMA_HISR_HTIF5) {
        DMA2->HIFCR = DMA_HIFCR_CHTIF5;
    }
    
}

void DMA2_Stream0_IRQHandler() {
    GPIOE->BSRRL |= (1 << SPI1_NSS);
    if (DMA2->LISR & DMA_LISR_TCIF0) {
        DMA2->LIFCR = DMA_LIFCR_CTCIF0;
        if((GPIOA->IDR & (1 << 1)) == 1) {  
            ADXL345_DMA_Init();
        }
        
        ADXL345_ProcessData();
    }
}



void EXTI_Init() {
    // acclerometer external interrupt on ACCEL_INT1
    SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI0_PE;
    EXTI->FTSR 	|= EXTI_FTSR_TR0;  
    EXTI->IMR 	|= EXTI_IMR_MR0;
    //NVIC_SetPriority(EXTI0_IRQn, 0x0F);
    NVIC_EnableIRQ(EXTI0_IRQn); 
    
    // frequency calculation interrupt for motor1
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PE;   
    EXTI->FTSR 	|= EXTI_FTSR_TR2; 
    EXTI->IMR 	|= EXTI_IMR_MR2;
    //NVIC_SetPriority(EXTI2_IRQn, 0x0F);
    NVIC_EnableIRQ(EXTI2_IRQn); 
    
    // frequency calculation interrupt for motor2
    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PE; 
    EXTI->FTSR 	|= EXTI_FTSR_TR4; 
    EXTI->IMR 	|= EXTI_IMR_MR4;
    //NVIC_SetPriority(EXTI4_IRQn, 0x0F);
    NVIC_EnableIRQ(EXTI4_IRQn); 
    
    // button
    SYSCFG->EXTICR[2] = SYSCFG_EXTICR3_EXTI10_PE; 
	EXTI->RTSR 	|= EXTI_RTSR_TR10;  	
	EXTI->IMR 	|= EXTI_IMR_MR10; 		
	NVIC_EnableIRQ(EXTI15_10_IRQn);  		
}

void GPIO_init() {
    // PD12, PD14 - motor pwm
    GPIOD->MODER    |= (2 << MOT_PWM1*2) | (2 << MOT_PWM2*2);               // alternative function
    GPIOD->OTYPER   &= ~((1 << MOT_PWM1) | (1 << MOT_PWM2));                // push-pull
    GPIOD->OSPEEDR  |= (3 << MOT_PWM1*2) | (3 << MOT_PWM2*2);               // high speed
    GPIOD->PUPDR    |= (1 << MOT_PWM1*2) | (1 << MOT_PWM2*2);               // pull-up
    GPIOD->AFR[1]   |= (2 << (MOT_PWM1 - 8)*4) | (2 << (MOT_PWM2 - 8)*4);   // AF2
    
    // accel data_ready interrupt
    GPIOE->MODER 	&= ~(3 << ACCEL_INT1*2);                                // input
    GPIOE->OTYPER	&= ~(1 << ACCEL_INT1);                                  // push-pull
	GPIOE->OSPEEDR 	|= 3 << ACCEL_INT1*2;                                   // high speed
    GPIOE->PUPDR    |= 1 << ACCEL_INT1*2;                                   // pull-up

    // button
    GPIOE->MODER    &= ~(3 << BUTTON*2);                                    // input  
    GPIOE->OSPEEDR  |= 3 << BUTTON*2;                                       // high speed
    GPIOE->PUPDR    |= 2 << BUTTON*2;                                       // pull-down
    
    // motor frequency calculation
    GPIOE->MODER 	&= ~((3 << MOT_FREQ1*2) | (3 << MOT_FREQ2*2));          // input
    GPIOE->OTYPER	&= ~((1 << MOT_FREQ1) | (1 << MOT_FREQ2));              // push-pull
	GPIOE->OSPEEDR 	|= (3 << MOT_FREQ1*2) | (3 << MOT_FREQ2*2);             // high speed 
    GPIOE->PUPDR    |= (1 << MOT_FREQ1*2) | (1 << MOT_FREQ2*2);             // pull-up
    
    // servo pwm control
    GPIOB->MODER    |= 2 << SERV_PWM*2;                                     // alternative function
    GPIOB->OTYPER   &= ~(1 << SERV_PWM);                                    // push-pull
    GPIOB->OSPEEDR  |= 3 << SERV_PWM*2;                                     // high speed
    GPIOB->PUPDR    |= 1 << SERV_PWM*2;                                     // pull-up
    GPIOB->AFR[1]   |= 9 << (SERV_PWM - 8)*4;                               // AF9 - TIM12CH1
    
    GPIOB->MODER    |= 1 << 10*2;
    GPIOB->OTYPER   &= ~(1 << 10);
    GPIOB->OSPEEDR  |= 3 << 10*2;
    GPIOB->PUPDR    |= 1 << 10*2;
}

void TIMERS_init() {
	pwm = 2000;
	TIM4->PSC 		= 7;	//PWM timer		// Divide tact frequency Fnew = F / (PSC + 1); my F = 72MHz, so Fnew = 100kHz
	TIM4->ARR 		= 10000;			// Number of ticks to get overflow
	TIM4->CCR1 	= pwm;		//first motor		// CCy and ARR ratio set duty cycle of pwm
	TIM4->CCMR1 	|= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1; // Set pwm mode 1
	TIM4->CCER 	|= TIM_CCER_CC1E; // Enable TIM3 capture/compre register 3
    
    TIM4->CCR3 	= pwm;		//second one		// CCy and ARR ratio set duty cycle of pwm
	TIM4->CCMR2 	|= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1; // Set pwm mode 1
	TIM4->CCER 	|= TIM_CCER_CC3E; // Enable TIM3 capture/compre register 3
	
	TIM4->CR1 = TIM_CR1_CEN;		// Enable timer
    
    TIM3->PSC = 7;
    TIM3->ARR = 5000;
    TIM3->DIER |= 1;					// Enable update interrupt
	NVIC_EnableIRQ(TIM3_IRQn);
    
    TIM3->CR1 = TIM_CR1_CEN;
    
    TIM2->PSC = 799;                   //button press timer
    TIM2->ARR = 2000;					// 50 ms to avoid button rattle
	TIM2->DIER |= 1;					// Enable update interrupt
	NVIC_EnableIRQ(TIM2_IRQn);
    
    TIM5->DIER |= 1;					// timer for frequency calculation
	//NVIC_EnableIRQ(TIM5_IRQn);
    
    TIM5->CR1 = TIM_CR1_CEN;
    
    TIM9->DIER |= 1;					// timer for frequency calculation
    
    TIM9->CR1 = TIM_CR1_CEN;
    
    TIM7->PSC = 7;
    TIM7->ARR = 5000;
    TIM7->DIER |= 1;
    //NVIC_SetPriority(TIM7_IRQn, 0xFF);
    NVIC_EnableIRQ(TIM7_IRQn);
    
    TIM12->PSC      = 7;
    TIM12->ARR      = 20000;
    TIM12->CCR1     = 1472;
    TIM12->CCMR1 	|= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
    TIM12->CCER     |= TIM_CCER_CC1E;
    
    TIM12->CR1      |= TIM_CR1_CEN;
}

void SPI1_GPIO_Init() { //spi gpio   
	GPIOA->MODER 	|= (2 << SPI1_SCK*2) | (2 << SPI1_MISO*2) | (2 << SPI1_MOSI*2);
	GPIOA->OSPEEDR 	|= (3 << SPI1_SCK*2) | (3 << SPI1_MISO*2) | (3 << SPI1_MOSI*2);
	GPIOA->AFR[0] 	|= (5 << SPI1_SCK*4) | (5 << SPI1_MISO*4) | (5 << SPI1_MOSI*4);                         // AF5
	GPIOA->OTYPER	&= ~((1 << SPI1_SCK) | (1 << SPI1_MISO) | (1 << SPI1_MOSI));
    
    /*-------- Configuring ChipSelect-Pin PE3 --------*/ 
    GPIOE->MODER 	|= (1 << SPI1_NSS*2);
	GPIOE->OSPEEDR 	|= (3 << SPI1_NSS*2);
	GPIOE->OTYPER	&= ~(1 << SPI1_NSS);
    GPIOE->PUPDR |= 1 << SPI1_NSS*2;
    GPIOE->BSRRL |= 1 << SPI1_NSS;
}

void SPI1_Init() {
    SPI1_GPIO_Init();
    
	
	
	SPI1->CR1 = 0;//|= 1 << 11;
	
	SPI1->CR1 |= SPI_CR1_BR; 							                        // baudrate = Fpclk / 256
	SPI1->CR1 |= SPI_CR1_CPOL;													// polarity
	SPI1->CR1 |= SPI_CR1_CPHA;													// phase	
	SPI1->CR1 &= ~(SPI_CR1_LSBFIRST);										    // MSBFIRST		
	SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;										// Software slave management		
	//SPI1->CR2 |= SPI_CR2_SSOE;
    SPI1->CR2 |= SPI_CR2_TXDMAEN;
    SPI1->CR2 |= SPI_CR2_RXDMAEN;
	SPI1->CR1 |= SPI_CR1_MSTR;                                                  // Master configuration	
	SPI1->CR1 |= SPI_CR1_DFF;//?
    SPI1->CR1 |= SPI_CR1_SPE;         // Enable SPI    
    //SPI1->DR = 0x0F;
}

uint16_t SPI1_Transfer(uint16_t byte) {
    uint16_t data;
    GPIOE->BSRRH |= (1 << SPI1_NSS);
	while ((SPI1->SR & SPI_SR_TXE)==0);
	SPI1->DR = byte;
	
	while ((SPI1->SR & SPI_SR_RXNE)==0);
	data = SPI1->DR;
    GPIOE->BSRRL |= (1 << SPI1_NSS);
    return data;
}

uint16_t LSDH_read(uint8_t address) { //accelerometer
    uint16_t tmp = 0;
    address |= 0x80;
    tmp = SPI1_Transfer(address << 8);
    return tmp & 0xFF;
}

void LSDH_write(uint8_t address, uint8_t data) {
    uint16_t tmp;
    
    address |= 0x00;
    tmp = address << 8;
    tmp |= data;
    SPI1_Transfer(tmp);
}

//void EXTI0_IRQHandler() {  //what to do then accelerometer is ready
//    if (EXTI->PR & EXTI_PR_PR0) {
//        EXTI->PR |= EXTI_PR_PR0;
//        ((uint8_t *)&ax)[0] = LSDH_read(0x28);
//        ((uint8_t *)&ax)[1] = LSDH_read(0x29);
//        ((uint8_t *)&ay)[0] = LSDH_read(0x2A);
//        ((uint8_t *)&ay)[1] = LSDH_read(0x2B);
//        ((uint8_t *)&az)[0] = LSDH_read(0x2C);
//        ((uint8_t *)&az)[1] = LSDH_read(0x2D);
//        ax -= xoff;
//        ay -= yoff;
//        az -= zoff;
//        
//        ADXL345_ProcessData();
//    }
//}

void TIM2_IRQHandler(void) { //button press
	if (TIM2->SR & TIM_SR_UIF) {
		TIM2->SR &= ~TIM_SR_UIF;   	// Clear interrupt flag
		TIM2->CR1 &= ~TIM_CR1_CEN; 	// Turn timer off
		if (TIM4->CCR1 == 1000) {
            TIM4->CCR1 = 1200;
            TIM4->CCR3 = 1200;
        }
        else if (TIM4->CCR1 == 1200) {
            TIM4->CCR1 = 1300;
            TIM4->CCR3 = 1300;
        }
        else if (TIM4->CCR1 == 1300) {
            TIM4->CCR1 = 1400;
            TIM4->CCR3 = 1400;
        }
        else if (TIM4->CCR1 == 1400) {
            TIM4->CCR1 = 1000;
            TIM4->CCR3 = 1000;
        }
	}
}

//void EXTI15_10_IRQHandler(void) { //button press
//	if (EXTI->PR & EXTI_PR_PR10) {
//		EXTI->PR = EXTI_PR_PR10;		// Clear interrupt flag
//		TIM2->CNT = 0;					// Reset timer counter
//		TIM2->CR1 |= TIM_CR1_CEN;		// Turn timer on
//	}
//}

void EXTI2_IRQHandler() { //this used to calculate the frequency of motor
    if (EXTI->PR & EXTI_PR_PR2) {
        EXTI->PR |= EXTI_PR_PR2;
        if (TIM5->CR1 && 1) {
            COUNT1 = TIM5->CNT;
            TIM5->CR1 &= ~1;
            TIM5->CNT = 0;
        }
        else {
            TIM5->CNT = 0;
            TIM5->CR1 |= 1;
        }
    }
}

void EXTI4_IRQHandler() { //this used to calculate the frequency of motor
    if (EXTI->PR & EXTI_PR_PR4) {
        EXTI->PR |= EXTI_PR_PR4;
        if (TIM9->CR1 && 1) {
            COUNT2 = TIM9->CNT;
            TIM9->CR1 &= ~1;
            TIM9->CNT = 0;
        }
        else {
            TIM9->CNT = 0;
            TIM9->CR1 |= 1;
        }
    }
}

void LSDH_Calibr()
{
    int i = 0;
    double x = 0, y = 0, z = 0;
    for(i = 0; i < 100; i++) {
        while((GPIOE->IDR & 0x01)) {};
    ((uint8_t *)&ax)[0] = LSDH_read(0x28);
    ((uint8_t *)&ax)[1] = LSDH_read(0x29);
    ((uint8_t *)&ay)[0] = LSDH_read(0x2A);
    ((uint8_t *)&ay)[1] = LSDH_read(0x2B);
    ((uint8_t *)&az)[0] = LSDH_read(0x2C);
    ((uint8_t *)&az)[1] = LSDH_read(0x2D);
        x += ax;
        y += ay;
        z += az;
    }
    xoff = (int)(x/100);
    yoff = (int)(y/100);
    zoff = (int)(z/100 - G);
}

uint8_t status = 0;
int timer = 0;
int timer_1000 = 2000;
int timer_2000 = 2000;
int pwm_step = 5;

void TIM3_IRQHandler() { //moving pwm from 2 to 1ms in the start
    TIM3->SR &= ~TIM_SR_UIF;
    //if ( timer > 0) timer--;
    /*else if (TIM4->CCR1 == 1000 || TIM4->CCR3 == 1000) {
        TIM3->CR1 &= ~TIM_CR1_CEN;
        TIM4->CCR1 = 1000;
        TIM4->CCR3 = 1000;
        ENGRDY = 1;
    }
    else {
        TIM4->CCR1--;
        TIM4->CCR3--;
    } */
    if (status == 0) {
        timer++;
        if (pwm == 1000 && timer == timer_1000) {
            status = 1;
            timer = 0;
        } else if (pwm == 2000 && timer == timer_2000) {
            status = 2;
            timer = 0;
        }
    } else if (status == 1) {
        pwm += pwm_step;
        TIM4->CCR1 = pwm;
        TIM4->CCR3 = pwm;
        if (pwm == 2000) {
            status = 0;
            timer = 0;
        }
    } else if (status == 2) {
        pwm -= pwm_step;
        TIM4->CCR1 = pwm;
        TIM4->CCR3 = pwm;
        if (pwm == 1000) {
            ENGRDY = 1;
            status = 3;
            timer = 0;
        }
    }    
}
   
void Accel_Init() {
    SPI1_Init();
    LSDH_write(CTRL_REG4, 0x77); 
    LSDH_write(CTRL_REG3, INT1_EN);
    LSDH_Calibr();
}

uint8_t serv_count = 0;

void SendTelemetry() {
    char tele[100];
    uint8_t len = 0, i;
    
//    if (serv_count == 0) {
//        if (TIM12->CCR1 == 2400) {
//            TIM12->CCR1 = 544;
//        } else {
//            TIM12->CCR1 = 2400;
//        } 
//    }
//    serv_count++;
    
//    if (pwm1 == 0 && pwm2 == 0) {
//        return;
//    }
    //sprintf(tele, "%8.2f\t%8.4f\t%8.4f\t%8d\t%8d\t%8d\t%8d\n", F, angle, angularVelocity, pwm1, COUNT1, pwm2, COUNT2);
    //sprintf(tele, "%d\t%d\t%d\t%d\n", pwm1, COUNT1, pwm2, COUNT2);
    sprintf(tele, "%8.2f\t%8.2f\t%8.2f\t%d%d\n", angle, angularVelocity, F, pwm1, pwm2);
    //sprintf(tele, "%8.2f\t%8.2f\t%8.2f\n", w[0], w[1], w[2]);
    len = strlen(tele);
    
    for (i = 0; i < len; i++) {
        send_to_uart(tele[i]);
    }
    
}

void TIM7_IRQHandler(void) {
    TIM7->SR &= ~TIM_SR_UIF;
    //SendTelemetry();
}

uint8_t KOEFF_RX = 0;
uint8_t KOEFF2_RX = 0;
double koeff = 0;
double my_pow = 0;
double d_st = 0;
int k = 0;



void USART2_IRQHandler() {
    if (USART2->SR & USART_SR_RXNE) {
        USART2->SR &= ~USART_SR_RXNE;
        received = USART2->DR;

        if (received == 's') {
            TIM4->CCR1 = 1000;
            TIM4->CCR3 = 1000;
            pwm1 = 1000;
            pwm2 = 1000;
            update = 0;
        } else if (received == 'c') {
            LSDH_Calibr();
            update = 1;
        } else if (received == 'b') {
            update = 1;
        } else if (received == 'n') {
            GPIOB->BSRRL |= 1 << 10;
        } else if (received == 'f') {
      
            GPIOB->BSRRH |= 1 << 10;
        } /*else if (received == 'l') {
            TIM12->CCR1 = 544;
        } else if (received == 'r') {
            TIM12->CCR1 = 2400;
        }*/
        
        /*else if (received == 'p') {
            PWM_RX = 1;
            i = -1;
            st = 1;
            pwm = 0;
        }
        else if (received == 'e') {
            PWM_RX = 0;
            while( i >= 0) {
                pwm += (str[i--]-'0') * st;
                st*=10;
            }
            pwm1 = pwm2 = pwm;
            TIM4->CCR1 	= pwm1;
            TIM4->CCR3 	= pwm2;
            NEW_PWM_RV = 1;
        }
        else if (PWM_RX == 1) {
            str[++i] = received;
        }  */else if (received == 'g') {
            KOEFF_RX = 1;
            i = -1;
            d_st = 1;
            my_pow = 10;
            koeff = 0;
        }
        else if (KOEFF_RX == 1 && received=='.')
        {
            while( i >= 0) {
                koeff += (str[i--]-'0') * d_st;
                d_st*=my_pow;
            }
            d_st = 1.0/10;
            my_pow = 1.0/10;
        }
        else if (received == 'e') {
            KOEFF_RX = 0;
            k = -1;
            //printf("%d  ", i);
            while( k < i) {
                koeff += (str[++k]-'0') * d_st;
                d_st*=my_pow; 
            }
            k1 = koeff;
        }
        else if (KOEFF_RX == 1) {
            str[++i] = received;
        } else if (received == 'x') {
            KOEFF2_RX = 1;
            i = -1;
            d_st = 1;
            my_pow = 10;
            koeff = 0;
        }
        else if (KOEFF2_RX == 1 && received=='.')
        {
            while( i >= 0) {
                koeff += (str[i--]-'0') * d_st;
                d_st*=my_pow;
            }
            d_st = 1.0/10;
            my_pow = 1.0/10;
        }
        else if (received == 'y') {
            KOEFF2_RX = 0;
            k = -1;
            //printf("%d  ", i);
            while( k < i) {
                koeff += (str[++k]-'0') * d_st;
                d_st*=my_pow; 
            }
            k2 = koeff;
        }
        else if (KOEFF2_RX == 1) {
            str[++i] = received;
        }
    }
}

int main() {
    RCC_Init();
    USART_init();
    GPIO_init();
    TIMERS_init();
    SPI_FIRST_INIT();  
    while(ENGRDY != 1) {};
    
    anglesAccumulated = 0;
        
    Accel_Init();
    
    EXTI_Init();
    
    TIM7->CR1 |= TIM_CR1_CEN;
    //update = 1;
    while(1) {
        if (SEND_TELEMETRY_FLAG) {
            SendTelemetry();
            SEND_TELEMETRY_FLAG = 0;
        }
    }
}
