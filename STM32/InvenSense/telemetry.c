#include "telemetry.h"
#include "string.h"
#include "stm32f4xx.h" 
#include "motors.h"

uint8_t USART1_TX = 9;  // PA
uint8_t USART1_RX = 10; // PA

typedef enum { WAITING_FOR_COMMAND, WAITING_FOR_INT, WAITING_FOR_FLOAT } waiting;

typedef enum { WAITING_FOR_PWM1, WAITING_FOR_PWM2, WAITING_FOR_MIN_PWM, WAITING_FOR_MAX_PWM, 
                WAITING_FOR_PARAMS_BITMASK, INT_NONE } waitingForInt;

typedef enum { WAITING_FOR_KP, WAITING_FOR_KD, WAITING_FOR_KI, WAITING_FOR_RESEARCH_AMPL, WAITING_FOR_RESEARCH_FREQ, 
WAITING_FOR_MAX_ANGLE, WAITING_FOR_ACCEL_DEVIATION, WAITING_FOR_BOUNDARY_ANGLE, WAITING_FOR_MAX_ANGVEL, FLOAT_NONE } waitingForFloat;

uint32_t paramsBitMask;
Message message;

extern uint8_t Message_Size;
#define MESSAGE_HEADER  0x21

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

uint8_t received;
char str[100];

// reading int variables
int i;
int stepen;
int intValue;

// reading float variables
int k;
float my_pow;
float d_st;
float floatValue;

waiting                 curWaiting          = WAITING_FOR_COMMAND;
waitingForInt           curWaitingForInt    = INT_NONE;
waitingForFloat         curWaitingForFloat  = FLOAT_NONE;

void initWaitingForInt() {
    curWaiting = WAITING_FOR_INT;
    i = -1;
    stepen = 1;
    intValue = 0;
}

void initWaitingForFloat() {
    curWaiting = WAITING_FOR_FLOAT;
    i = -1;
    d_st = 1;
    my_pow = 10;
    floatValue = 0;
}

int rst = 0;  // reset purposes
void USART1_IRQHandler() {
    if (USART1->SR & USART_SR_RXNE) {
        USART1->SR &= ~USART_SR_RXNE;
        received = USART1->DR;

        switch (curWaiting) {
            case WAITING_FOR_COMMAND:
                switch (received) {
                    case TURN_EVERYTHING_OFF:
                        telemetry_on = 0;
                        //research = NO_RESEARCH;
                        break;
                    case STOP_MOTORS:   
                        Motors_Stop();
                        break;
//                    case CALIBRATION:
//                        //research = NO_RESEARCH;
//                        Motors_Stop();
//                        
//                        ADXL345_Calibr();
//                        ADXRS290_Calibr();
//                        //Gyro_Calibr();
                        
//                        break;
                    case TELEMETRY:
                        telemetry_on ^= 1;
                        break;
                    
                    case PARAMS_BITMASK_SYMBOL:
                        initWaitingForInt();
                        curWaitingForInt = WAITING_FOR_PARAMS_BITMASK;
                        break;
                    
                    case PWM1:
                        initWaitingForInt();
                        curWaitingForInt = WAITING_FOR_PWM1;
                        break;
                    case PWM2:
                        initWaitingForInt();
                        curWaitingForInt = WAITING_FOR_PWM2;
                        break;
                    
                    case MIN_PWM:
                        initWaitingForInt();
                        curWaitingForInt = WAITING_FOR_MIN_PWM;
                        break;
                    case MAX_PWM:
                        initWaitingForInt();
                        curWaitingForInt = WAITING_FOR_MAX_PWM;
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
                    
//                    case IMPULSE:
//                        research = IMPULSE_RESPONSE;
//                        researchIndex = 0;
//                        break;
//                    case STEP:
//                        research = STEP_RESPONSE;
//                        researchIndex = 0;
//                        break;
//                    case SINE:
//                        curWaiting = WAITING_FOR_FLOAT;
//                        curWaitingForFloat = WAITING_FOR_RESEARCH_AMPL;
//                        initWaitingForFloat();
//                        research = SINE_RESPONSE;
//                        break;
//                    case EXP:
//                        curWaiting = WAITING_FOR_FLOAT;
//                        curWaitingForFloat = WAITING_FOR_RESEARCH_AMPL;
//                        initWaitingForFloat();
//                        research = EXP_RESPONSE;
//                        break;
//                    case NO_RESEARCH_SYMBOL:
//                        research = NO_RESEARCH;
//                        Motors_Stop();
//                        researchIndex = 0;
//                        break;
//                    case SIMPLE:
//                        research = SIMPLE_CONTROL;
//                        Kp = (maxPwm - minPwm) / maxAngle;
//                        break;
//                    case PID:
//                        research = PID_CONTROL;
//                        break;
//                    case OPERATOR:
//                        research = OPERATOR_CONTROL;
//                        break;
//                    case ADJUST:
//                        research = ADJUST_CONTROL;
//                        pwm1 = minPwm;
//                        pwm2 = minPwm;
//                        break;
                    
                    case PROGRAMMING_MODE:
                        GPIOB->MODER |= 1 << 8*2;
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
                            intValue += (str[i--] - '0') * stepen;
                            stepen *= 10;
                        }         
                        switch (curWaitingForInt) {
                            case WAITING_FOR_PARAMS_BITMASK:
                                paramsBitMask = intValue;
                                break;
                            case WAITING_FOR_PWM1:
                                //if (research == OPERATOR_CONTROL) {
                                    motors.pwm1 = intValue;
                                    Motors_Run();
                                //}
                                break;
                            case WAITING_FOR_PWM2:
                                //if (research == OPERATOR_CONTROL) {
                                    motors.pwm2 = intValue;
                                    Motors_Run();
                                //}
                                break;
//                            case WAITING_FOR_MIN_PWM:
//                                minPwm = intValue;
//                                break;
//                            case WAITING_FOR_MAX_PWM:
//                                maxPwm = intValue;
//                                break;
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
//                            case WAITING_FOR_KP:
//                                Kp = floatValue;
//                                break;
//                            case WAITING_FOR_KD:
//                                Kd = floatValue;
//                                break;
//                            case WAITING_FOR_KI:
//                                Ki = floatValue;
//                                break;
//                            case WAITING_FOR_RESEARCH_AMPL:
//                                researchAmplitude = floatValue;
//                                curWaiting = WAITING_FOR_FLOAT;
//                                curWaitingForFloat = WAITING_FOR_RESEARCH_FREQ;
//                                initWaitingForFloat();
//                                break;
//                            case WAITING_FOR_RESEARCH_FREQ:
//                                researchFrequency = floatValue;
//                                researchIndex = 0;
//                                break;
//                            case WAITING_FOR_MAX_ANGLE:
//                                maxAngle = floatValue;
//                                break;
//                            case WAITING_FOR_ACCEL_DEVIATION:
//                                accelDeviation = floatValue;
//                                break;
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


void Telemetry_DMA_Init() {
    NVIC_EnableIRQ(DMA2_Stream7_IRQn);
    NVIC_SetPriority(DMA2_Stream7_IRQn, 0x02);      
        
    DMA2_Stream7->PAR = (uint32_t)&(USART1->DR);
    DMA2_Stream7->CR = DMA_SxCR_CHSEL_2 | DMA_SxCR_DIR_0 | DMA_SxCR_MINC | DMA_SxCR_PL | DMA_SxCR_TCIE;
}

uint8_t completed = 1;
void Telemetry_DMA_Run(uint8_t *data, uint8_t size) {
    DMA2_Stream7->M0AR = (uint32_t)(data);
    DMA2_Stream7->NDTR = size;
    DMA2_Stream7->CR |= DMA_SxCR_EN;
    completed = 0;
}

void DMA2_Stream7_IRQHandler() {
    if (DMA2->HISR & DMA_HISR_TCIF7) {
        DMA2->HIFCR = DMA_HIFCR_CTCIF7 | DMA_HIFCR_CHTIF7;
        DMA2_Stream7->CR &= ~DMA_SxCR_EN;
        completed = 1;
    }
}

#define MAX_PACKET_SIZE 86 // 84 + header + crc
extern float mine_accel[3];
extern float mine_gyro[3];
extern float mine_compass[3];
extern float mpl_euler[3];
extern float dmp_euler[3];
extern float mine_euler[3];
extern uint16_t pwm1;
extern uint16_t pwm2;
extern uint16_t freq1;
extern uint16_t freq2;
extern float F;

uint8_t tele[MAX_PACKET_SIZE];
uint8_t tele_len;
static uint8_t crc;
static uint8_t crc_i;

void float_to_bytes(float *pFloat, uint8_t *fourBytes) {
    uint8_t i;
    for (i = 0; i < 4; i++) {
        fourBytes[i] = (((uint8_t *)pFloat)[i]);
    }
}

void Telemetry_Send() {
    if (telemetry_on && completed) {
        tele[0] = MESSAGE_HEADER;
        crc = tele[0];
        tele_len = 1;
        if (paramsBitMask & BIT_ACCEL_X) {
            float_to_bytes(&mine_accel[0], &tele[tele_len]);
            tele_len += sizeof(mine_accel[0]);
        }
        if (paramsBitMask & BIT_ACCEL_Y) {
            float_to_bytes(&mine_accel[1], &tele[tele_len]);
            tele_len += sizeof(mine_accel[1]);
        }
        if (paramsBitMask & BIT_ACCEL_Z) {
            float_to_bytes(&mine_accel[2], &tele[tele_len]);
            tele_len += sizeof(mine_accel[2]);
        }
        if (paramsBitMask & BIT_GYRO_X) {
            float_to_bytes(&mine_gyro[0], &tele[tele_len]);
            tele_len += sizeof(mine_gyro[0]);
        }
        if (paramsBitMask & BIT_GYRO_Y) {
            float_to_bytes(&mine_gyro[1], &tele[tele_len]);
            tele_len += sizeof(mine_gyro[1]);
        }
        if (paramsBitMask & BIT_GYRO_Z) {
            float_to_bytes(&mine_gyro[2], &tele[tele_len]);
            tele_len += sizeof(mine_gyro[2]);
        }
        if (paramsBitMask & BIT_COMPASS_X) {
            float_to_bytes(&mine_compass[0], &tele[tele_len]);
            tele_len += sizeof(mine_compass[0]);
        }
        if (paramsBitMask & BIT_COMPASS_Y) {
            float_to_bytes(&mine_compass[1], &tele[tele_len]);
            tele_len += sizeof(mine_compass[1]);
        }
        if (paramsBitMask & BIT_COMPASS_Z) {
            float_to_bytes(&mine_compass[2], &tele[tele_len]);
            tele_len += sizeof(mine_compass[2]);
        }
        if (paramsBitMask & BIT_MPL_EULER_X) {
            float_to_bytes(&mpl_euler[0], &tele[tele_len]);
            tele_len += sizeof(mpl_euler[0]);
        }
        if (paramsBitMask & BIT_MPL_EULER_Y) {
            float_to_bytes(&mpl_euler[1], &tele[tele_len]);
            tele_len += sizeof(mpl_euler[1]);
        }
        if (paramsBitMask & BIT_MPL_EULER_Z) {
            float_to_bytes(&mpl_euler[2], &tele[tele_len]);
            tele_len += sizeof(mpl_euler[2]);
        }
        if (paramsBitMask & BIT_DMP_EULER_X) {
            float_to_bytes(&dmp_euler[0], &tele[tele_len]);
            tele_len += sizeof(dmp_euler[0]);
        }
        if (paramsBitMask & BIT_DMP_EULER_Y) {
            float_to_bytes(&dmp_euler[1], &tele[tele_len]);
            tele_len += sizeof(dmp_euler[1]);
        }
        if (paramsBitMask & BIT_DMP_EULER_Z) {
            float_to_bytes(&dmp_euler[2], &tele[tele_len]);
            tele_len += sizeof(dmp_euler[2]);
        }
        if (paramsBitMask & BIT_MINE_EULER_X) {
            float_to_bytes(&mine_euler[0], &tele[tele_len]);
            tele_len += sizeof(mine_euler[0]);
        }
        if (paramsBitMask & BIT_MINE_EULER_Y) {
            float_to_bytes(&mine_euler[1], &tele[tele_len]);
            tele_len += sizeof(mine_euler[1]);
        }
        if (paramsBitMask & BIT_MINE_EULER_Z) {
            float_to_bytes(&mine_euler[2], &tele[tele_len]);
            tele_len += sizeof(mine_euler[2]);
        }
        if (paramsBitMask & BIT_PWM1) {
            tele[tele_len]      = (uint8_t)((pwm1 >> 8) & 0xff);
            tele[tele_len+1]    = (uint8_t)(pwm1 & 0xff);
            tele_len += sizeof(pwm1);
        }
        if (paramsBitMask & BIT_PWM2) {
            tele[tele_len]      = (uint8_t)((pwm2 >> 8) & 0xff);
            tele[tele_len+1]    = (uint8_t)(pwm2 & 0xff);
            tele_len += sizeof(pwm2);
        }
        if (paramsBitMask & BIT_FREQ1) {
            tele[tele_len]      = (uint8_t)((freq1 >> 8) & 0xff);
            tele[tele_len+1]    = (uint8_t)(freq1 & 0xff);
            tele_len += sizeof(freq1);
        }
        if (paramsBitMask & BIT_FREQ2) {
            tele[tele_len]      = (uint8_t)((freq2 >> 8) & 0xff);
            tele[tele_len+1]    = (uint8_t)(freq2 & 0xff);
            tele_len += sizeof(freq2);
        }
        if (paramsBitMask & BIT_F) {
            tele[tele_len]      = (uint8_t)(((uint8_t *)&F)[0]);
            tele[tele_len+1]    = (uint8_t)(((uint8_t *)&F)[1]);
            tele[tele_len+2]    = (uint8_t)(((uint8_t *)&F)[2]);
            tele[tele_len+3]    = (uint8_t)(((uint8_t *)&F)[3]);
            tele_len += sizeof(F);
        }
        
        for (crc_i = 1; crc_i < tele_len; crc_i++) {
            crc ^= tele[crc_i];
        }
        tele[tele_len] = crc;
        tele_len++;
        Telemetry_DMA_Run(tele, tele_len);
    }
}



