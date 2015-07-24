#include "processing.h"
#include "stdint.h"
#include "math.h"
#include "stm32f4xx.h"
#include "syssolve.h"
#include "motors.h"
#include "adxl345.h"
#include "telemetry.h"

#define DT MEASUREMENT_TIME

//float Ak = 1;
//float Hk = 1;

//float Qk = 0.01;
//float Rkx = 0.75;
//float Rky = 0.75;
//float Rkz = 1.1;
//float Kx = 0;
//float Ky = 0;
//float Kz = 0;

//float predictedAx = 0;
//float predictedAy = 0;
//float predictedAz = 0;

//float predictedPx = 0;
//float predictedPy = 0;
//float predictedPz = 0;

//float Px = 0;
//float Py = 0;
//float Pz = 255;

// state
float Ax = 0;
float Ay = 0;
float Az = 255;


float prevAngle = 0;

uint8_t stabilizationOn = 0;
uint8_t kalmanOn        = 0;
uint8_t averagingOn     = 1;
uint8_t impulseOn       = 0;

// A = [1 DT
//      0 1 ]
float A[4] = {1, DT, 0, 1};
float At[4] = {1, 0, DT, 1};

// H = [1 0]
float H[2] = {1, 0};
float Ht[2] = {1, 0};

float Xaposteriori[2] = {0, 0};
float Paposteriori[4] = {0, 0, 0, 1000};
float R[1] = {0.03};
float Q[4] = {0.1, 0, 0, 0.1};
float I[4] = {1, 0, 0, 1};

float Xapriori[2];
float Papriori[4];
    

void kalman() {
    float tmp1[4]; // A * Paposteriori
    float y[1];
    float z[1];
    float tmp2[1]; // H * Xapriori
    float S[1], Sinv[1];
    float tmp3[2]; // H * Papriori
    float tmp4[2]; // tmp3 * Ht
    float K[2];
    float tmp5[4];
    
    z[0] = angle;
    
    // Xapriori calculation
    mat_mul(A, Xaposteriori, Xapriori, 2, 1, 2);
    
    // Papriori calculation
    mat_mul(A, Paposteriori, tmp1, 2, 2, 2);
    mat_mul(tmp1, At, tmp5, 2, 2, 2);
    mat_add(tmp5, Q, Papriori, 2, 2);
    
    // y calculation
    mat_mul(H, Xapriori, tmp2, 1, 1, 2);
    mat_sub(z, tmp2, y, 1, 1);
    
    // S calculation
    mat_mul(H, Papriori, tmp3, 1, 2, 2);
    mat_mul(tmp3, Ht, tmp4, 1, 1, 2);
    mat_add(tmp4, R, S, 1, 1);
    
    mat_mul(Papriori, Ht, tmp3, 2, 1, 2);
    Sinv[0] = 1 / S[0];
    mat_mul(tmp3, Sinv, K, 2, 1, 1);
    
    mat_mul(K, y, tmp4, 2, 1, 1);
    mat_add(Xapriori, tmp4, Xaposteriori, 2, 1);
    
    mat_mul(K, H, tmp1, 2, 2, 1);
    mat_sub(I, tmp1, tmp5, 2, 2);
    mat_mul(tmp5, Papriori, Paposteriori, 2, 2, 2);

    angle           = Xaposteriori[0];
    angularVelocity = Xaposteriori[1];
}


//void kalman() {
//    // prediction
//    predictedAx = Ak * Ax;
//    predictedAy = Ak * Ay;
//    predictedAz = Ak * Az;
//    
//    predictedPx = Ak * Px * Ak + Qk;
//    predictedPy = Ak * Py * Ak + Qk;
//    predictedPz = Ak * Pz * Ak + Qk;
//    
//    // correction
//    Kx = predictedPx * Hk / (Hk * predictedPx * Hk + Rkx);
//    Ky = predictedPy * Hk / (Hk * predictedPy * Hk + Rky);
//    Kz = predictedPz * Hk / (Hk * predictedPz * Hk + Rkz);
//    
//    Ax = predictedAx + Kx * ((float)ax - Hk * predictedAx);
//    Ay = predictedAy + Ky * ((float)ay - Hk * predictedAy);
//    Az = predictedAz + Kz * ((float)az - Hk * predictedAz);
//    
//    Px = (1 - Kx * Hk) * predictedPx;
//    Py = (1 - Ky * Hk) * predictedPy;
//    Pz = (1 - Kz * Hk) * predictedPz;
//}

float Aappr = 5.13e-7;
float Bappr = 0.001;
float Cappr = -1.81;
float D = 0;
float x1 = 0;
float x2 = 0;

float chooseRoot() {
    if (D < 0) return 2000;
    
    x1 = (-Bappr - sqrt(D)) / (2 * Aappr);
    x2 = (-Bappr + sqrt(D)) / (2 * Aappr);
    
    if (x1 > 1000 && x1 < 2000) return x1;
    if (x2 > 1000 && x2 < 2000) return x2;
    return 2000;
}

uint8_t angleCount = 0;
float angleSum = 0;
uint8_t averaged = 0;
float angles[8] = {0, 0, 0, 0, 0, 0, 0, 0};

//void averaging() {
//    angle = atan((float)Ay / (float)Az);
//    angleSum += angle;
//    angleCount++;
//    if (angleCount == 8) {
//        angle = angleSum / angleCount; // true angle we work with
//        angleSum = 0;
//        angleCount = 0;
//        averaged = 1;
//    }
//}
void averaging() {
    angleSum -= angles[angleCount];
    angles[angleCount] = angle;
    angleSum += angle;
    if (angleCount == 7) {
        angleCount = 0;
    } else {
        angleCount++;
    }
    angle = angleSum / 8;
}

void control() {
    
    prevAngle = angle;
    
    F = k1*angle + k2*angularVelocity;

    if (stabilizationOn/* && STABRDY*/) {
        if (impulseOn) {
            impulseOn = 0;
            angle -= 1;
            angularVelocity = (angle - prevAngle) / ((float)DT);
            F = k1*angle + k2*angularVelocity;
        }
        
        if (F > 0) {
            pwm1 = minPwm;
            TIM4->CCR1 = minPwm;
            D = Bappr*Bappr - 4*Aappr*(Cappr - fabs(F));
            pwm2 = (int)chooseRoot();
            if (pwm2 < minPwm) {
                pwm2 = minPwm;
            }
            TIM4->CCR3 = pwm2;
        } else if (F < 0) {
            pwm2 = minPwm;
            TIM4->CCR3 = minPwm;
            D = Bappr*Bappr - 4*Aappr*(Cappr - fabs(F));
            pwm1 = (int)chooseRoot();
            if (pwm1 < minPwm) {
                pwm1 = minPwm;
            }
            TIM4->CCR1 = pwm1;
        }
    }     
        // Identification block
        if (anglesAccumulated < 2) {
            y[anglesAccumulated] = angle;
            u[anglesAccumulated] = F;
        } else {
            y[2] = angle;
            u[2] = F;
            
            row = anglesAccumulated - 2;
            Afull[row*3 + 0] = (y[2] - y[1]) / ((float)DT);
            Afull[row*3 + 1] = y[2];
            Afull[row*3 + 2] = -u[2];
            
            Bfull[row] = (2*y[1] - y[2] - y[0]) / ((float)DT) / ((float)DT);
            
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

    SendTelemetry();
}

void process() {
    angle = atan((float)ay / (float)az);
       
    if (averagingOn) {
        averaging();
    } 
    if (kalmanOn) {
        kalman();
    } else {
        angularVelocity = (angle - prevAngle) / ((float)DT);
    }
    control();
}
