#include "stm32f4xx.h" 
#include "string.h"
#include "stdio.h"

#include "telemetry.h"
#include "adxl345.h"
#include "gyro.h"
#include "motors.h"
#include "processing.h"

uint8_t UART2_TX = 2;    // PA2
uint8_t UART2_RX = 3;    // PA3

typedef enum { WAITING_FOR_COMMAND, WAITING_FOR_INT, WAITING_FOR_FLOAT } waiting;

typedef enum { WAITING_FOR_PWM1, WAITING_FOR_PWM2, ANGLE_WINDOW_SIZE, ANGVEL_WINDOW_SIZE, INT_NONE } waitingForInt;

typedef enum { WAITING_FOR_KP, WAITING_FOR_KD, WAITING_FOR_KI, FLOAT_NONE } waitingForFloat;

typedef enum { FULL, MOVE_DESCRIPTION, AX, AY, AZ} telemetryMode;

uint8_t received;
char str[100];

// reading int variables
int i;
int st;
int intValue;

// reading float variables
int k;
float my_pow;
float d_st;
float floatValue;

waiting                 curWaiting          = WAITING_FOR_COMMAND;
waitingForInt           curWaitingForInt    = INT_NONE;
waitingForFloat         curWaitingForFloat  = FLOAT_NONE;

uint8_t telemetryOn = 0;
telemetryMode curTelemetryMode = FULL;


uint8_t displayAngle            = 1;
uint8_t displayAngularVelocity  = 1;
uint8_t displayF                = 1;
uint8_t displayPwm              = 1;

char tele[100] = "";
uint8_t len = 0, j;

void USART_Init(void) {
    GPIOA->MODER    |= (2 << UART2_TX*2) | (2 << UART2_RX*2);
    GPIOA->OTYPER   &= ~((1 << UART2_TX) | (1 << UART2_RX));
    GPIOA->OSPEEDR  |= (3 << UART2_TX*2) | (3 << UART2_RX*2);
    GPIOA->PUPDR    |= (1 << UART2_TX*2) | (1 << UART2_RX*2);
    GPIOA->AFR[0]   |= (7 << UART2_TX*4) | (7 << UART2_RX*4);

    USART2->BRR     = 0x45;//0x341; 
    USART2->CR3     |= USART_CR3_DMAT;
    USART2->CR1     |= USART_CR1_UE | USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE; 
    NVIC_SetPriority(USART2_IRQn, 0x02);
    NVIC_EnableIRQ(USART2_IRQn);
    
//    GPIOA->MODER &= ~(3UL << 15*2);
//    GPIOA->MODER |= 1 << (15*2);GPIOA->BSRRH |= 1 << 15;
////    GPIOA->BSRRL |= 1 << 15;
//    
//    while (1) {
//    sprintf(tele, "AT\r\n");
//        len = strlen(tele);
//        for (j = 0; j < len; j++) {
//            send_to_uart(tele[j]);
//        }
//    }
    
}

void Telemetry_TIM_Init() {
    TIM7->PSC = 7;
    TIM7->ARR = 2500;
    TIM7->DIER |= 1;
    NVIC_SetPriority(TIM7_IRQn, 0xFF);
    NVIC_EnableIRQ(TIM7_IRQn);
    
    //TIM7->CR1 |= TIM_CR1_CEN;
}

void TIM7_IRQHandler(void) {
    TIM7->SR &= ~TIM_SR_UIF;
    SendTelemetry();
}

void initWaitingForInt() {
    curWaiting = WAITING_FOR_INT;
    i = -1;
    st = 1;
    intValue = 0;
}

void initWaitingForFloat() {
    curWaiting = WAITING_FOR_FLOAT;
    i = -1;
    d_st = 1;
    my_pow = 10;
    floatValue = 0;
}


void USART2_IRQHandler() {
    if (USART2->SR & USART_SR_RXNE) {
        USART2->SR &= ~USART_SR_RXNE;
        received = USART2->DR;

        switch (curWaiting) {
            case WAITING_FOR_COMMAND:
                switch (received) {
                    case 'a':
                        stabilizationOn     = 0;
                        telemetryOn         = 0;
                        kalmanOn            = 0;
                        angleAveragingOn    = 0;                                  
                        break;
                    case 'e':   
                        if (stabilizationOn) {
                            stabilizationOn = 0;
                            Motors_Stop();
                        } else {
                            //Motors_InitForStab();
                            stabilizationOn = 1; 
                        }
                        break;
                    case 'd':
                        stabilizationOn = 0;
                        Motors_Stop();
                        ADXL345_Calibr();
                        break;
                    case 'h':
                        telemetryOn ^= 1;
                        break;
                    case 'i':
                        curTelemetryMode = FULL;
                        break;
                    case 'k':
                        curTelemetryMode = MOVE_DESCRIPTION;
                        break;
                    case 'l':
                        curTelemetryMode = AX;
                        break;
                    case 'q':
                        curTelemetryMode = AY;
                        break;
                    case 'r':
                        curTelemetryMode = AZ;
                        break;
                    case 'm':
                        initWaitingForInt();
                        curWaitingForInt = WAITING_FOR_PWM1;
                        break;
                    case 'n':
                        initWaitingForInt();
                        curWaitingForInt = WAITING_FOR_PWM2;
                        break;
                    case 'o':
                        initWaitingForFloat();
                        curWaitingForFloat = WAITING_FOR_KP;
                        break;
                    case 'p':
                        initWaitingForFloat();
                        curWaitingForFloat = WAITING_FOR_KD;
                        break;
                    case 'x':
                        initWaitingForFloat();
                        curWaitingForFloat = WAITING_FOR_KI;
                        break;
                    case 's':
                        kalmanOn ^= 1;
                        break;
                    case 't':
                        angleAveragingOn ^= 1;
                        if (angleAveragingOn) {
                            angleSum = 0;
                            angleIndex = 0;
                        }
                        break;
                    case 'u':
                        initWaitingForInt();
                        curWaitingForInt = ANGLE_WINDOW_SIZE;
                        break;
                    case 'v':
                        initWaitingForInt();
                        curWaitingForInt = ANGVEL_WINDOW_SIZE;
                        break;
                    case 'w':
                        angVelAveragingOn ^= 1;  
                        if (angVelAveragingOn) {
                            angVelSum = 0;
                            angVelIndex = 0;
                        }                    
                        break;
                    case 'A':
                        freshFreq = 1;
                        curFreq = HZ100;
                        curDT = 0.01;
                        break;
                    case 'B':
                        freshFreq = 1;
                        curFreq = HZ800;
                        curDT = 0.00125;
                        break;
                    case 'C':
                        impulseOn = 1;
                        break;
                    case 'D':
                        freshFreq = 1;
                        curFreq = HZ1600;
                        curDT = 0.000625;
                        break;
                    case 'E':
                        freshFreq = 1;
                        curFreq = HZ3200;
                        curDT = 0.0003125;
                        break;
                    
                    case 'F':
                        gyroFreshFreq = 1;
                        gyroCurFreq = GYRO_HZ100;
                        gyroCurDT = 0.01;
                        break;
                    case 'G':
                        gyroFreshFreq = 1;
                        gyroCurFreq = GYRO_HZ250;
                        gyroCurDT = 0.004;
                        break;
                    case 'H':
                        gyroFreshFreq = 1;
                        gyroCurFreq = GYRO_HZ500;
                        gyroCurDT = 0.002;
                        break;
                    case 'I':
                        gyroFreshFreq = 1;
                        gyroCurFreq = GYRO_HZ1000;
                        gyroCurDT = 0.001;
                        break;
                }
                break;
            case WAITING_FOR_INT:
                switch (received) {
                    case 'b':   
                        curWaiting = WAITING_FOR_COMMAND;
                        while (i >= 0) {
                            intValue += (str[i--] - '0') * st;
                            st *= 10;
                        }         
                        switch (curWaitingForInt) {
                            case WAITING_FOR_PWM1:
                                minPwm = intValue;
//                                pwm1 = intValue;
//                                TIM4->CCR1 = pwm1;
                                break;
                            case WAITING_FOR_PWM2:
                                pwm2 = intValue;
                                TIM4->CCR3 	= pwm2;
                                break;
                            case ANGLE_WINDOW_SIZE:
                                allocAngleAveraging(intValue);
                                break;
                            case ANGVEL_WINDOW_SIZE:
                                allocAngVelAveraging(intValue);
                                break;
                            default:
                                break;
                        }
                        break;
                    default:
                        str[++i] = received;
                        break;
                }
                break;
            case WAITING_FOR_FLOAT:
                switch (received) {
                    case 'b':
                        curWaiting = WAITING_FOR_COMMAND;
                        k = -1;
                        while( k < i) {
                            floatValue += (str[++k] - '0') * d_st;
                            d_st *= my_pow; 
                        }
                        switch (curWaitingForFloat) {
                            case WAITING_FOR_KP:
                                Kp = floatValue;
                                break;
                            case WAITING_FOR_KD:
                                Kd = floatValue;
                                break;
                            case WAITING_FOR_KI:
                                Ki = floatValue;
                                break;
                            default:
                                break;
                        }
                        break;
                    case '.':
                        while( i >= 0) {
                            floatValue += (str[i--] - '0') * d_st;
                            d_st *= my_pow;
                        }
                        d_st = 1.0/10;
                        my_pow = 1.0/10;
                        break;
                    default:
                        str[++i] = received;
                        break;
                }
                break;
        }
    }
}

void send_to_uart(uint8_t data) {
    while(!(USART2->SR & USART_SR_TC)); 
    USART2->DR = data; 
}

void SendTelemetry() {
    if (telemetryOn) {
        switch (curTelemetryMode) {
            case FULL:
                sprintf(tele, "%.2f %.2f %.2f %hd %hd %hd %d %d %.2f %.2f %.2f %d %d %.2f %.2f %.2f %d %d\n", 
            angle, angularVelocity, F, ax, ay, az, pwm1, pwm2, gyroX, gyroY, gyroZ, COUNT1, COUNT2, Kp, Ki, Kd, angleWindowSize, angVelWindowSize);
                break;
            case MOVE_DESCRIPTION:
                sprintf(tele, "%.2f %.2f %.2f %d %d\n", angle, angularVelocity, F, pwm1, pwm2);
                break;
            case AX:
                sprintf(tele, "%hd %.2f\n", ax, Ax);
                break;
            case AY:
                sprintf(tele, "%hd %.2f\n", ay, Ay);
                break;
            case AZ:
                sprintf(tele, "%hd %.2f\n", az, Az);
                break;
        }
        len = strlen(tele);
        
        Telemetry_DMA_Init();
    }
}

uint8_t USART_DMA_transferComleted = 1;
void Telemetry_DMA_Init() {
    if (USART_DMA_transferComleted) {
        USART_DMA_transferComleted = 0;
        NVIC_EnableIRQ(DMA1_Stream6_IRQn);
        
        DMA1_Stream6->CR    = 0;
        DMA1_Stream6->PAR   = (uint32_t)&(USART2->DR);
        DMA1_Stream6->M0AR  = (uint32_t)tele;     
        DMA1_Stream6->NDTR  = len;
        DMA1_Stream6->CR    |= DMA_SxCR_CHSEL_2 | DMA_SxCR_MINC /*| DMA_SxCR_PSIZE_0 | DMA_SxCR_MSIZE_0 */| 
                                    DMA_SxCR_TCIE | DMA_SxCR_DIR_0  | DMA_SxCR_PL | DMA_SxCR_EN;
    }
}

void DMA1_Stream6_IRQHandler() {
    if (DMA1->HISR & DMA_HISR_TCIF6) {
        DMA1->HIFCR = DMA_HIFCR_CTCIF6;
        USART_DMA_transferComleted = 1;
    }
}
