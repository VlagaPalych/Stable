#include "stm32f4xx.h" 
#include "string.h"
#include "stdio.h"

#include "telemetry.h"
#include "adxl345.h"
#include "gyro.h"
#include "motors.h"
#include "processing.h"
#include "commands.h"

uint8_t UART1_TX = 9;    // PA
uint8_t UART1_RX = 10;    // PA

typedef enum { WAITING_FOR_COMMAND, WAITING_FOR_INT, WAITING_FOR_FLOAT } waiting;

typedef enum { WAITING_FOR_PWM1, WAITING_FOR_PWM2, WAITING_FOR_ANGLE_WINDOW_SIZE, WAITING_FOR_ANGVEL_WINDOW_SIZE, INT_NONE } waitingForInt;

typedef enum { WAITING_FOR_KP, WAITING_FOR_KD, WAITING_FOR_KI, WAITING_FOR_RESEARCH_AMPL, WAITING_FOR_RESEARCH_FREQ, WAITING_FOR_MAX_ANGLE, FLOAT_NONE } waitingForFloat;

typedef enum { FULL } telemetryMode;

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

uint8_t recalibrate = 0;

char tele[100] = "";
uint8_t len = 0, j;

void USART_Init(void) {
    GPIOA->MODER    |= (2 << UART1_TX*2) | (2 << UART1_RX*2);
    GPIOA->OTYPER   &= ~((1 << UART1_TX) | (1 << UART1_RX));
    GPIOA->OSPEEDR  |= (3 << UART1_TX*2) | (3 << UART1_RX*2);
    GPIOA->PUPDR    |= (1 << UART1_TX*2) | (1 << UART1_RX*2);
    GPIOA->AFR[1]   |= (7 << (UART1_TX-8)*4) | (7 << (UART1_RX-8)*4);

    USART1->BRR     = 0x45;//0x341; 
    USART1->CR3     |= USART_CR3_DMAT;
    USART1->CR1     |= USART_CR1_UE | USART_CR1_M | USART_CR1_PCE | USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE; 
    NVIC_SetPriority(USART1_IRQn, 0x02);
    NVIC_EnableIRQ(USART1_IRQn);
    
//    GPIOA->MODER &= ~(3UL << 15*2);
//    GPIOA->MODER |= 1 << (15*2);GPIOA->BSRRH |= 1 << 15;
////    GPIOA->BSRRL |= 1 << 15;
//    
//    sprintf(tele, "123456789");while (1) {
//        
//        len = strlen(tele);
////        for (j = 0; j < len; j++) {
////            send_to_uart(tele[j]);
////        }
//        Telemetry_DMA_Init();
//    }
    
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

int rst = 0;
void USART1_IRQHandler() {
    if (USART1->SR & USART_SR_RXNE) {
        USART1->SR &= ~USART_SR_RXNE;
        received = USART1->DR;

        switch (curWaiting) {
            case WAITING_FOR_COMMAND:
                switch (received) {
                    case TURN_EVERYTHING_OFF:
                        stabilizationOn     = 0;
                        telemetryOn         = 0;
                        kalmanOn            = 0;
                        angleAveragingOn    = 0;                                  
                        break;
                    case STABILIZATION:   
                        if (stabilizationOn) {
                            stabilizationOn = 0;
                            Motors_Stop();
                        } else {
                            //Motors_InitForStab();
                            stabilizationOn = 1; 
                        }
                        break;
                    case CALIBRATION:
                        stabilizationOn = 0;
                        Motors_Stop();
                        
                        ADXL345_Calibr();
                        Gyro_Calibr();
                        
                        break;
                    case LOWPASS:
                        lowpassOn ^= 1;
                        break;
                    case TELEMETRY:
                        telemetryOn ^= 1;
                        break;
                    case TELEMETRY_MODE_FULL:
                        curTelemetryMode = FULL;
                        break;
                    case PWM1:
                        initWaitingForInt();
                        curWaitingForInt = WAITING_FOR_PWM1;
                        break;
                    case PWM2:
                        initWaitingForInt();
                        curWaitingForInt = WAITING_FOR_PWM2;
                        break;
                    
                    case KP:
                        initWaitingForFloat();
                        curWaitingForFloat = WAITING_FOR_KP;
                        break;
                    case KI:
                        initWaitingForFloat();
                        curWaitingForFloat = WAITING_FOR_KI;
                        break;
                    case KD:
                        initWaitingForFloat();
                        curWaitingForFloat = WAITING_FOR_KD;
                        break;
                    
                    case KALMAN:
                        kalmanOn ^= 1;
                        break;
                    case ANGLE_MOVING_AVERAGE:
                        angleAveragingOn ^= 1;
                        if (angleAveragingOn) {
                            angleSum = 0;
                            angleIndex = 0;
                        }
                        break;
                    case ANGLE_WINDOW_SIZE:
                        initWaitingForInt();
                        curWaitingForInt = WAITING_FOR_ANGLE_WINDOW_SIZE;
                        break;
                    case ANGVEL_WINDOW_SIZE:
                        initWaitingForInt();
                        curWaitingForInt = WAITING_FOR_ANGVEL_WINDOW_SIZE;
                        break;
                    case ANGVEL_MOVING_AVERAGE:
                        angVelAveragingOn ^= 1;  
                        if (angVelAveragingOn) {
                            angVelSum = 0;
                            angVelIndex = 0;
                        }                    
                        break;
                        
                    case ACCEL_FREQ_HZ25:
                        freshFreq = 1;
                        curFreq = HZ25;
                        curDT = 0.04;
                        break;
                    case ACCEL_FREQ_HZ50:
                        freshFreq = 1;
                        curFreq = HZ50;
                        curDT = 0.02;
                        break;    
                    case ACCEL_FREQ_HZ100:
                        freshFreq = 1;
                        curFreq = HZ100;
                        curDT = 0.01;
                        break;
                    case ACCEL_FREQ_HZ800:
                        freshFreq = 1;
                        curFreq = HZ800;
                        curDT = 0.00125;
                        break;
                    case ACCEL_FREQ_HZ1600:
                        freshFreq = 1;
                        curFreq = HZ1600;
                        curDT = 0.000625;
                        break;
                    case ACCEL_FREQ_HZ3200:
                        freshFreq = 1;
                        curFreq = HZ3200;
                        curDT = 0.0003125;
                        break;
                    
                    
                    case GYRO_FREQ_HZ100:
                        gyroFreshFreq = 1;
                        gyroCurFreq = GYRO_HZ100;
                        gyroCurDT = 0.01;
                        break;
                    case GYRO_FREQ_HZ250:
                        gyroFreshFreq = 1;
                        gyroCurFreq = GYRO_HZ250;
                        gyroCurDT = 0.004;
                        break;
                    case GYRO_FREQ_HZ500:
                        gyroFreshFreq = 1;
                        gyroCurFreq = GYRO_HZ500;
                        gyroCurDT = 0.002;
                        break;
                    case GYRO_FREQ_HZ1000:
                        gyroFreshFreq = 1;
                        gyroCurFreq = GYRO_HZ1000;
                        gyroCurDT = 0.001;
                        break;
                    
                    case MAX_ANGLE:
                        curWaiting = WAITING_FOR_FLOAT;
                        curWaitingForFloat = WAITING_FOR_MAX_ANGLE;
                        initWaitingForFloat();
                        break;
                    
                    case IMPULSE:
                        research = IMPULSE_RESPONSE;
                        researchIndex = 0;
                        break;
                    case STEP:
                        research = STEP_RESPONSE;
                        researchIndex = 0;
                        break;
                    case SINE:
                        curWaiting = WAITING_FOR_FLOAT;
                        curWaitingForFloat = WAITING_FOR_RESEARCH_AMPL;
                        initWaitingForFloat();
                        research = SINE_RESPONSE;
                        break;
                    case EXP:
                        curWaiting = WAITING_FOR_FLOAT;
                        curWaitingForFloat = WAITING_FOR_RESEARCH_AMPL;
                        initWaitingForFloat();
                        research = EXP_RESPONSE;
                        break;
                    case NO_RESEARCH_SYMBOL:
                        research = NO_RESEARCH;
                        Motors_Stop();
                        researchIndex = 0;
                        break;
                    case SIMPLE:
                        research = SIMPLE_CONTROL;
                        break;
                    
                    case PROGRAMMING_MODE:
                        GPIOB->BSRRL |= 1 << 8;
                        for (rst = 0; rst < 100000; rst++) {
                            __nop();
                        }
                        SCB->AIRCR = (0x5fa << 16) | (1 << 2);
                        break;
                }
                break;
            case WAITING_FOR_INT:
                switch (received) {
                    case NUMBER_END:   
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
                            case WAITING_FOR_ANGLE_WINDOW_SIZE:
                                allocAngleAveraging(intValue);
                                break;
                            case WAITING_FOR_ANGVEL_WINDOW_SIZE:
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
                            case WAITING_FOR_RESEARCH_AMPL:
                                researchAmplitude = floatValue;
                                curWaiting = WAITING_FOR_FLOAT;
                                curWaitingForFloat = WAITING_FOR_RESEARCH_FREQ;
                                initWaitingForFloat();
                                break;
                            case WAITING_FOR_RESEARCH_FREQ:
                                researchFrequency = floatValue;
                                researchIndex = 0;
                                break;
                            case WAITING_FOR_MAX_ANGLE:
                                maxAngle = floatValue;
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
    while((USART1->SR & USART_SR_TC)==0); 
    USART1->DR = data; 
}

void SendRaw() {
    sprintf(tele, "r%hd\n", azHistory[azHistoryIndex]);
    len = strlen(tele);
    Telemetry_DMA_Init();
}

void SendRawAndProcessed() {
    sprintf(tele, "%hd\n", az);
    len = strlen(tele);
    Telemetry_DMA_Init();
}

void SendTelemetry() {
    if (telemetryOn) {
        switch (curTelemetryMode) {
            case FULL:
                if (lowpassOn) {
                        sprintf(tele, "%.2f %.2f %.2f %hd %hd %hd %d %d %.2f %d %d %.2f %.2f %.2f %d\n", 
                    angle, angularVelocity, F, finalAX, ay, finalAZ, pwm1, pwm2, filteredGX, COUNT1, COUNT2, Kp, Ki, Kd, research);
                } else {
                    sprintf(tele, "%.2f %.2f %.2f %hd %hd %hd %d %d %hd %d %d %.2f %.2f %.2f\n", 
                    angle, angularVelocity, F, ax, ay, az, pwm1, pwm2, gx, COUNT1, COUNT2, Kp, Ki, Kd);
                }
//                if (lowpassOn) {
//                    sprintf(tele, "%hd\n", finalGX);
//                } else {
//                    sprintf(tele, "%hd\n", gx);
//                }
                break;
        }
        len = strlen(tele);

        Telemetry_DMA_Init();
//        send_to_uart('a');
    }
}

uint8_t USART_DMA_transferComleted = 1;
void Telemetry_DMA_Init() {
    if (USART_DMA_transferComleted) {
        USART_DMA_transferComleted = 0;
        NVIC_EnableIRQ(DMA2_Stream7_IRQn);
        
        DMA2_Stream7->CR    = 0;
        DMA2_Stream7->PAR   = (uint32_t)&(USART1->DR);
        DMA2_Stream7->M0AR  = (uint32_t)tele;     
        DMA2_Stream7->NDTR  = len;
        DMA2_Stream7->CR    |= DMA_SxCR_CHSEL_2 | DMA_SxCR_MINC /*| DMA_SxCR_PSIZE_0 | DMA_SxCR_MSIZE_0 */| 
                                    DMA_SxCR_TCIE | DMA_SxCR_DIR_0  | DMA_SxCR_PL | DMA_SxCR_EN;
    }
}

void DMA2_Stream7_IRQHandler() {
    if (DMA2->HISR & DMA_HISR_TCIF7) {
        DMA2->HIFCR = DMA_HIFCR_CTCIF7;
        USART_DMA_transferComleted = 1;
    }
}
