#include "telemetry.h"
#include "adxl345.h"
#include "stm32f4xx.h" 
#include "string.h"
#include "stdio.h"

typedef enum { WAITING_FOR_COMMAND, WAITING_FOR_INT, WAITING_FOR_FLOAT } waiting;

typedef enum { WAITING_FOR_PWM1, WAITING_FOR_PWM2, INT_NONE } waitingForInt;

typedef enum { WAITING_FOR_K1, WAITING_FOR_K2, FLOAT_NONE } waitingForFloat;

typedef enum { TELEMETRY_FULL, AX, AY, AZ} telemetryMode;

extern int pwm1;
extern int pwm2;
extern uint8_t received;
extern char str[100];
extern int i;
extern int st;
int intValue;

extern int k;
extern double my_pow;
extern double d_st;
double floatValue;

extern double k1;
extern double k2;

waiting                 curWaiting          = WAITING_FOR_COMMAND;
waitingForInt           curWaitingForInt    = INT_NONE;
waitingForFloat         curWaitingForFloat  = FLOAT_NONE;

uint8_t telemetryOn = 0;
telemetryMode curTelemetryMode = AZ;

uint8_t curFreq = HZ100;
uint8_t freshFreq = 0;

extern short ax;
extern short ay;
extern short az;

extern double Ax;
extern double Ay;
extern double Az;

extern double angle;
extern double angularVelocity;
extern double F;

extern uint8_t stabilizationOn;
extern uint8_t kalmanOn;
extern uint8_t averagingOn;

uint8_t displayAngle            = 1;
uint8_t displayAngularVelocity  = 1;
uint8_t displayF                = 1;
uint8_t displayPwm              = 1;

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
                    case 'e':
                        stabilizationOn ^= 1;
                        break;
                    case 'h':
                        telemetryOn ^= 1;
                        break;
                    case 'k':
                        curTelemetryMode = TELEMETRY_FULL;
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
                        curWaitingForFloat = WAITING_FOR_K1;
                        break;
                    case 'p':
                        initWaitingForFloat();
                        curWaitingForFloat = WAITING_FOR_K2;
                        break;
                    case 's':
                        kalmanOn ^= 1;
                        break;
                    case 't':
                        averagingOn ^= 1;
                        break;
                    case 'A':
                        freshFreq = 1;
                        curFreq = HZ100;
                        break;
                    case 'B':
                        freshFreq = 1;
                        curFreq = HZ800;
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
                                pwm1 = intValue;
                                TIM4->CCR1 = pwm1;
                                break;
                            case WAITING_FOR_PWM2:
                                pwm2 = intValue;
                                TIM4->CCR3 	= pwm2;
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
                            case WAITING_FOR_K1:
                                k1 = floatValue;
                                break;
                            case WAITING_FOR_K2:
                                k2 = floatValue;
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
    char tele[100] = "";
    char angleStr[5];
    char angulVelStr[5];
    char fStr[5];
    char pwmStr[10];
    uint8_t len = 0, i;
    
    if (telemetryOn) {
        switch (curTelemetryMode) {
            case TELEMETRY_FULL:
                if (displayAngle) {
                    sprintf(angleStr, "%.2f ", angle);
                    strcat(tele, angleStr);
                }
                if (displayAngularVelocity) {
                    sprintf(angulVelStr, "%.2f ", angularVelocity);
                    strcat(tele, angulVelStr);
                }
                if (displayF) {
                    sprintf(fStr, "%.2f ", F);
                    strcat(tele, fStr);
                } 
                if (displayPwm) {
                    sprintf(pwmStr, "%d %d", pwm1, pwm2);
                    strcat(tele, pwmStr);
                }
                strcat(tele, "\n");
                //sprintf(tele, "%.2f %.2f %.2f %d %d\n", angle, angularVelocity, F, pwm1, pwm2);
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
        //sprintf(tele, "%8.2f\t%8.4f\t%8.4f\t%8d\t%8d\t%8d\t%8d\n", F, angle, angularVelocity, pwm1, COUNT1, pwm2, COUNT2);
        //sprintf(tele, "%d\t%d\t%d\t%d\n", pwm1, COUNT1, pwm2, COUNT2);
        
        //sprintf(tele, "%8.2f\t%8.2f\t%8.2f\n", w[0], w[1], w[2]);
        len = strlen(tele);
        
        for (i = 0; i < len; i++) {
            send_to_uart(tele[i]);
        }
    }
}
