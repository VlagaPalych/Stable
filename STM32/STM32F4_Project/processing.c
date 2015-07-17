#include "processing.h"
#include "stdint.h"
#include "math.h"
#include "stm32f4xx.h"

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

extern short ax;
extern short ay;
extern short az;


double F = 0;
double k1 = 2.15;
double k2 = 1.5e-4;
uint8_t firstAngleMeasurement = 1;
double angle = 0;
double prevAngle = 0;
double angularVelocity = 0;
int pwm1, pwm2;

uint8_t stabilizationOn = 0;

void kalman() {
    // prediction
    predictedAx = Ak * Ax;
    predictedAy = Ak * Ay;
    predictedAz = Ak * Az;
    
    predictedPx = Ak * Px * Ak + Qk;
    predictedPy = Ak * Py * Ak + Qk;
    predictedPz = Ak * Pz * Ak + Qk;
    
    // correction
    Kx = predictedPx * Hk / (Hk * predictedPx * Hk + Rkx);
    Ky = predictedPy * Hk / (Hk * predictedPy * Hk + Rky);
    Kz = predictedPz * Hk / (Hk * predictedPz * Hk + Rkz);
    
    Ax = predictedAx + Kx * ((double)ax - Hk * predictedAx);
    Ay = predictedAy + Ky * ((double)ay - Hk * predictedAy);
    Az = predictedAz + Kz * ((double)az - Hk * predictedAz);
    
    Px = (1 - Kx * Hk) * predictedPx;
    Py = (1 - Ky * Hk) * predictedPy;
    Pz = (1 - Kz * Hk) * predictedPz;
}

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

void control() {
    angle = atan((double)Ay / (double)Az);
    
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
    
        if (stabilizationOn) {
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
}
