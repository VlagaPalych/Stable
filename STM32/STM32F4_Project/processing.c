#include "processing.h"
#include "stdint.h"
#include "math.h"
#include "stm32f4xx.h"
#include "syssolve.h"
#include "motors.h"
#include "adxl345.h"
#include "telemetry.h"
#include "stdlib.h"
#include "string.h"
#include "adxrs290.h"
#include "adxrs453.h"
#include "filters.h"


#define DT MEASUREMENT_TIME
#define G2 65025

float maxAngle = 0.17;
float accelDeviation = 0.05;
uint8_t angleFromAccel = 0;
float maxAngVel = 0.5;

researchType research = NO_RESEARCH;
uint32_t researchIndex = 0;
float researchAmplitude = 0;
float researchFrequency = 0;

uint8_t processCounter = 0;

float adxrs_b[ADXRS_FILTER_SIZE];
int16_t adxrsHistory[HISTORY_SIZE];
uint8_t adxrsHistoryIndex = 0;
uint8_t adxrsCurHistoryIndex = 0;
float filteredVel = 0;
    
 
//int16_t accel_history[3][HISTORY_SIZE];
//uint8_t accel_historyIndex[3];
//uint8_t accel_curHistoryIndex[3];
//int16_t filtered_a[3];
float final_a[3];
    
uint8_t doAccelProcess = 0;
uint8_t doGyroProcess = 0;
uint8_t doAdxrsProcess = 0;
    
uint8_t lowpassOn = 1;
uint8_t accelLowpassReady = 0;
    
uint8_t stabilizationOn = 0;


//**************************************************************
//
// F depends on duty cycle of pwm like:
// F = k_dc * pwm + b_dc
//
// F depends on squared frequency of rotation of motor
// F = k_f2 * f + b_f2
//
//**************************************************************

float k_dc = 0.002496;
float b_dc = -2.898293;

float k_f2 = 4.915078e-8;
float b_f2 = -0.178341;

float pwm_from_force(float F) {
    return (F - b_dc) / k_dc;
}

float force_from_freq2(float f2) {
    return k_f2 * f2 + b_f2;
}    

void Motors_CalcPwm() {
    float absF = fabs(F);
    if (F > 0) {
        pwm1 = pwm_from_force(F);
        if (pwm1 > maxPwm) {
            pwm1 = maxPwm;
        }
        pwm2 = minPwm;
    } else if (F < 0) {
        pwm2 = pwm_from_force(F);
        if (pwm2 > maxPwm) {
            pwm2 = maxPwm;
        }
        pwm1 = minPwm;
    }

}

//**************************************************************
//
// Implementation of pid-controller in form:
// u(t) = Kp*e(t) + Ki*int_0^t{e(tau)dtau} + Kd*(de(t)/dt)
//
//**************************************************************

float angleIntegral = 0;
float angleIntegralMax = 1e5;

void pid_control() {
    angleIntegral += angle[0];
    
    F = Kp * angle[0]  + Ki * angleIntegral + Kd * angleRate[0];

    Motors_CalcPwm();
    Motors_Run();   
  
}

float T[9];
float eulerAngleRate[3];
float phi = 0;
float teta = 0;
float psi = 0;

void calcAngleRate() {
    uint8_t i = 0;
    for (i = 0; i < ADXRS290_NUMBER; i++) {
        ars_angleRate[i][0] = ars_filteredData[i][0] / 200.0;
        ars_angleRate[i][1] = ars_filteredData[i][1] / 200.0;
        angleRate[i] = (ars_angleRate[0][i] - ars_angleRate[1][i]) / 2.0;    
    }
    angleRate[2] = filteredVel / 80.0;
    
    phi = angle[0] * 3.14159 / 180.0;
    teta = angle[1] * 3.14159 / 180.0;
    //psi = angle[2] * 3.14159 / 180.0;
    
//    T[0] = 1;    T[1] = sin(phi) * tan(teta);    T[2] = cos(phi) * tan(teta);
//    T[3] = 0;    T[4] = cos(phi);                T[5] = - sin(phi);
//    T[6] = 0;    T[7] = sin(phi) / cos(teta);    T[8] = cos(phi) / cos(teta);
//    
//    mat_mul(T, angleRate, eulerAngleRate, 3, 1, 3);
    
    eulerAngleRate[0] = angleRate[0] + angleRate[1]*sin(phi)*tan(teta) + angleRate[2]*cos(phi)*tan(teta);
    eulerAngleRate[1] = angleRate[1]*cos(phi) + angleRate[2]*sin(phi);
    eulerAngleRate[2] = angleRate[1]*sin(phi)/cos(teta) + angleRate[2]*cos(phi)/cos(teta);
    
}

float lowpass(int16_t *history, uint8_t lowpassIndex, float *fir, int firSize) {
    uint8_t i = 0, tmpIndex = 0;
    float output = 0;
    float tmp = 0;
    for (i = 0; i < firSize; i++) {
        tmpIndex = lowpassIndex - i;
        tmp = fir[i] * history[tmpIndex];
        output += tmp;
    }
    return output;
}

void Processing_TIM_Init() {
    TIM7->PSC = 63;
    TIM7->ARR = 1000000;
    TIM7->DIER |= 1;
    NVIC_SetPriority(TIM7_IRQn, 0xFF);
    NVIC_EnableIRQ(TIM7_IRQn);
    
    TIM7->CR1 |= TIM_CR1_CEN;
}

void getFinalData() {
    uint8_t i = 0;
    if (lowpassOn) {
        for (i = 0; i < 3; i++) {
            final_a[i] = filtered_a[i];
        }
    } else {
        for (i = 0; i < 3; i++) {
            final_a[i] = a[i];
        }
    }
}


float roll = 0;
float pitch = 0;

//void calcAngle() {
//    float gx = finalAX;
//    float gy = finalAY;
//    float gz = finalAZ;
//    
//    roll    = atanf(gy / gz);
//    pitch   = atanf(- gx / sqrt(gy*gy + gz*gz));
//    
//    roll    *= 180.0 / 3.14159;
//    pitch   *= 180.0 / 3.14159;
//}

#define dt 0.01
#define Sa 100.0

float A[4] = {1, dt, 0, 1};
float I[4] = {1, 0, 0, 1};

float Xroll[2] = {0, 0};
float Zroll[2] = {0, 0};
float Proll[4] = {0, 0, 0, 0};
float Qroll[4] = {dt*dt*dt*dt*Sa*Sa/4, dt*dt*dt*dt*Sa*Sa/4 + dt*dt*Sa*Sa, dt*dt*dt*dt*Sa*Sa/4 + dt*dt*Sa*Sa, dt*dt*Sa*Sa};
float Rroll[4] = {4.76e-5, 0, 0, 1.444e-5};

float Xpitch[2] = {0, 0};
float Zpitch[2] = {0, 0};
float Ppitch[4] = {0, 0, 0, 0};
float Qpitch[4] = {dt*dt*dt*dt*Sa*Sa/4, dt*dt*dt*dt*Sa*Sa/4 + dt*dt*Sa*Sa, dt*dt*dt*dt*Sa*Sa/4 + dt*dt*Sa*Sa, dt*dt*Sa*Sa};
float Rpitch[4] = {4.76e-5, 0, 0, 3.72e-5};
 
                
void kalman2(const float *A, const float *Q, const float *R, float *x, const float *z, float *P) {
    float x_apriori[2], P_apriori[4], At[4];
    float K[4];
    float tmp1[4], tmp2[4];
    
    // x_apriori = A*x
    mat_mul(A, x, x_apriori, 2, 2, 1);
    
    // P_apriori = A*P_aposteriori*At + Q
    mat_mul(A, P, tmp1, 2, 2, 2);
    transpose(A, At, 2, 2);
    mat_mul(tmp1, At, tmp2, 2, 2, 2);
    mat_add(tmp2, Q, P_apriori, 2, 2);
    
    // K = P_apriori(P_apriori + R)^-1
    mat_add(P_apriori, R, tmp1, 2, 2);
    mat2_inv(tmp1, tmp2);
    mat_mul(P_apriori, tmp2, K, 2, 2, 2);
    
    // x_aposteriori = x_apriori + K(z - x_apriori)
    mat_sub(z, x_apriori, tmp1, 2, 1);
    mat_mul(K, tmp1, tmp2, 2, 2, 1);
    mat_add(x_apriori, tmp2, x, 2, 1);
    
    // P_aposteriori = (I - K)P_apriori
    mat_sub(I, K, tmp1, 2, 2);
    mat_mul(tmp1, P_apriori, P, 2, 2, 2);
}

float invS[9] = {1.0098, 0.02385, 0.00744, -0.00619, 1.00095, 0.03252, -0.00405, -0.02073, 1.00135};
float offset[3] = {25.7667, 7.58333, 83.66666};
float tmp[3];

void TIM7_IRQHandler(void) {
    TIM7->SR &= ~TIM_SR_UIF;
    
    getFinalData();
//    calcAngleRate(); 
//    calcAngle();
//       
//    Zroll[0] = roll;
//    Zroll[1] = eulerAngleRate[0];
//    Zpitch[0] = pitch;
//    Zpitch[1] = eulerAngleRate[1];
//    
//    kalman2(A, Qroll, Rroll, Xroll, Zroll, Proll);
//    kalman2(A, Qpitch, Rpitch, Xpitch, Zpitch, Ppitch);
//    
//    angle[0] = Xroll[0];
//    angle[1] = Xpitch[0];
//  
//    researchIndex++;
//    switch (research) {
//        case IMPULSE_RESPONSE:
//            if (researchIndex == 1) {
//                F = 1.0;
//                Motors_CalcPwm();
//                Motors_Run();
//            } else if (researchIndex == 50) {
//                F = 0;
//                Motors_CalcPwm();
//                Motors_Stop();
//                research = NO_RESEARCH;
//            }
//            break;
//        case STEP_RESPONSE:
//            if (researchIndex == 1) {
//                F = 1.0;
//                Motors_CalcPwm();
//                Motors_Run();
//            }
//            break;
//        case SINE_RESPONSE:
//            F = researchAmplitude * sin(researchFrequency * researchIndex);
//            Motors_CalcPwm();
//            Motors_Run();
//            break;
//        case EXP_RESPONSE:
//            F = researchAmplitude * exp(-researchFrequency * researchIndex);
//            Motors_CalcPwm();
//            Motors_Run();
//            break;
//        case PID_CONTROL:
//            pid_control();
//            break;
//        default:
//            researchIndex--;
//            break;
//    }
//    
//    message.angle = Xroll[0];
//    message.angleRate = Xroll[1];
//    message.pwm1 = pwm1;
//    message.pwm2 = pwm2;
//    message.freq1 = COUNT1;
//    message.freq2 = COUNT2;

//    mat_sub(final_a, offset, tmp, 3, 1);
//    mat_mul(invS, tmp, final_a, 3, 1, 3);
    
    message.ars1_x = ars_filteredData[0][0];
    message.ars1_y = ars_filteredData[0][1];
    message.ars2_x = ars_filteredData[1][0];
    message.ars2_y = ars_filteredData[1][1];
    message.ars3_z = filteredVel;

    SendTelemetry(&message); 
}


//void identify() {
//        // Identification block
//        if (anglesAccumulated < 2) {
//            y[anglesAccumulated] = angle;
//            u[anglesAccumulated] = F;
//        } else {
//            y[2] = angle;
//            u[2] = F;
//            
//            row = anglesAccumulated - 2;
//            Afull[row*3 + 0] = (y[2] - y[1]) / ((float)DT);
//            Afull[row*3 + 1] = y[2];
//            Afull[row*3 + 2] = -u[2];
//            
//            Bfull[row] = (2*y[1] - y[2] - y[0]) / ((float)DT) / ((float)DT);
//            
//            y[0] = y[1];
//            y[1] = y[2];
//            u[0] = u[1];
//            u[1] = u[2];
//            
//            if (anglesAccumulated == 11) {
//                identificationReady = 1;         
//                anglesAccumulated = 1;
//            }      
//            
//            if (identificationReady) {
//                system_solve(Afull, Bfull, w, 10, 3);
//            }
//        } 
//        anglesAccumulated++; 
//}
