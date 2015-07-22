#include "processing.h"
#include "stdint.h"
#include "math.h"
#include "stm32f4xx.h"
#include "syssolve.h"
#include "motors.h"
#include "adxl345.h"

#define DT MEASUREMENT_TIME

double Ak = 1;
double Hk = 1;

double Qk = 0.01;
double Rkx = 0.75;
double Rky = 0.75;
double Rkz = 1.1;
double Kx = 0;
double Ky = 0;
double Kz = 0;

double predictedAx = 0;
double predictedAy = 0;
double predictedAz = 0;

double predictedPx = 0;
double predictedPy = 0;
double predictedPz = 0;

double Px = 0;
double Py = 0;
double Pz = 255;

// state
double Ax = 0;
double Ay = 0;
double Az = 255;


double prevAngle = 0;

uint8_t stabilizationOn = 0;
uint8_t kalmanOn        = 0;
uint8_t averagingOn     = 0;

// A = [1 DT
//      0 1 ]
double A[4] = {1, DT, 0, 1};
double At[4] = {1, 0, DT, 1};

// H = [1 0]
double H[2] = {1, 0};
double Ht[2] = {1, 0};

double Xaposteriori[2] = {0, 0};
double Paposteriori[4] = {0, 0, 0, 1000};
double R[1] = {0.03};
double Q[4] = {0.1, 0, 0, 0.1};
double I[4] = {1, 0, 0, 1};

double Xapriori[2];
double Papriori[4];
    

void kalman() {
    double tmp1[4]; // A * Paposteriori
    double y[1];
    double z[1];
    double tmp2[1]; // H * Xapriori
    double S[1], Sinv[1];
    double tmp3[2]; // H * Papriori
    double tmp4[2]; // tmp3 * Ht
    double K[2];
    double tmp5[4];
    
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
//    Ax = predictedAx + Kx * ((double)ax - Hk * predictedAx);
//    Ay = predictedAy + Ky * ((double)ay - Hk * predictedAy);
//    Az = predictedAz + Kz * ((double)az - Hk * predictedAz);
//    
//    Px = (1 - Kx * Hk) * predictedPx;
//    Py = (1 - Ky * Hk) * predictedPy;
//    Pz = (1 - Kz * Hk) * predictedPz;
//}

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
uint8_t averaged = 0;

void averaging() {
    angle = atan((double)Ay / (double)Az);
    angleSum += angle;
    angleCount++;
    if (angleCount == 8) {
        angle = angleSum / angleCount; // true angle we work with
        angleSum = 0;
        angleCount = 0;
        averaged = 1;
    }
}

void control() {
    
    prevAngle = angle;
    
    F = k1*angle + k2*angularVelocity;

    if (stabilizationOn) {
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
//        if (anglesAccumulated < 2) {
//            y[anglesAccumulated] = angle;
//            u[anglesAccumulated] = F;
//        } else {
//            y[2] = angle;
//            u[2] = F;
//            
//            row = anglesAccumulated - 2;
//            Afull[row*3 + 0] = (y[1] - y[0]) / MEASUREMENT_TIME;
//            Afull[row*3 + 1] = y[0];
//            Afull[row*3 + 2] = -u[0];
//            
//            Bfull[row] = (2*y[1] - y[2] - y[0]) / MEASUREMENT_TIME / MEASUREMENT_TIME;
//            
//            y[0] = y[1];
//            y[1] = y[2];
//            u[0] = u[1];
//            u[1] = u[2];
//            
//            if (anglesAccumulated == 11) {
//                system_solve(Afull, Bfull, w, 10, 3);
//                anglesAccumulated = 1;
//            }      
//        } 
//        anglesAccumulated++;
}

void process() {
    angle = atan((double)ay / (double)az);
    
    if (kalmanOn) {
        kalman();
    } else {
        /*Ax = ax;
        Ay = ay;
        Az = az;*/
        angularVelocity = (angle - prevAngle) / DT;
    }
    
    if (averagingOn) {
        averaging();
        angularVelocity = (angle - prevAngle) / DT;
    }
    
    if ((averagingOn && averaged) || !averagingOn) {
        averaged = 0;
        control();
    }
}
