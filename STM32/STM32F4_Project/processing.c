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
    TIM7->ARR = 10000;
    TIM7->DIER |= 1;
    NVIC_SetPriority(TIM7_IRQn, 0xFF);
    NVIC_EnableIRQ(TIM7_IRQn);
    
    TIM7->CR1 |= TIM_CR1_CEN;
}

float roll = 0;
float pitch = 0;

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


//*********************************************************
//
// accData, gyrData - arrays with 3 values
// roll, pitch - pointers to keep angle values
//
//*********************************************************

#define REASONABLE_ACCEL_MAGNITUDE 560 // 2g

void complementary_filter(float *accData, float *gyrData, float *roll, float *pitch) {
    float pitchAcc, rollAcc; 
    float accMagnitude;
    
    *pitch += gyrData[1] * dt;
    *roll += gyrData[0] * dt;
    
    accMagnitude = sqrt(accData[0]*accData[0] + accData[1]*accData[1] + accData[2]*accData[2]);
    if (accMagnitude < REASONABLE_ACCEL_MAGNITUDE) {
        pitchAcc = atan2f(accData[0], accData[2]) * 180 / 3.14159;
        *pitch = *pitch * 0.98 + pitchAcc * 0.02;
        
        rollAcc = atan2f(accData[1], accData[2]) * 180 / 3.14159;
        *roll = *roll * 0.98 + rollAcc * 0.02;
    }
}

float pitchGyr = 0, rollGyr = 0;

void TIM7_IRQHandler(void) {
    float pitchAcc, rollAcc; 
    float accMagnitude;
    //float mean_detector = 0;
    TIM7->SR &= ~TIM_SR_UIF;
    
    complementary_filter(calibrated_a, calibrated_ar, &roll, &pitch);

    message.accelRoll = 0;
    message.accelPitch = 0;
    message.complementaryPitch = 0;
    message.complementaryRoll = 0;
    
    pitch += calibrated_ar[1] * dt;
    pitchGyr += calibrated_ar[1] * dt;
    roll += calibrated_ar[0] * dt;
    rollGyr += calibrated_ar[0] * dt;
    
    message.gyroPitch = pitchGyr;
    message.gyroRoll = rollGyr;
    
    accMagnitude = sqrt(calibrated_a[0]*calibrated_a[0] + calibrated_a[1]*calibrated_a[1] + calibrated_a[2]*calibrated_a[2]);
    if (accMagnitude < REASONABLE_ACCEL_MAGNITUDE) {
        pitchAcc = atan2f(calibrated_a[0], calibrated_a[2]) * 180 / 3.14159;
        message.accelPitch = pitchAcc;
        pitch = pitch * 0.98 + pitchAcc * 0.02;
        message.complementaryPitch = pitch;
        
        rollAcc = atan2f(calibrated_a[1], calibrated_a[2]) * 180 / 3.14159;
        message.accelRoll = rollAcc;
        roll = roll * 0.98 + rollAcc * 0.02;
        message.complementaryRoll = roll;
    }
    
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


//    arm_mean_f32(detector, ACCEL_DECIMATION, &mean_detector);
//    message.detector = mean_detector;
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

//#define QUASISTATIC_THRESHOLD 14.0f
//float detector[ACCEL_DECIMATION];
//uint8_t detector_index = 0;
//float lpf_rect_hpf_a[3];
//uint8_t quasistatic_new_a = 0;
//uint8_t quasistatic_moment = 0;


//void lpf_rect_hpf() {
//    uint8_t i = 0;
//    // calculate ax^2 + ay^2 + az^2
//    accel_modulo_history[accel_modulo_history_index] = sqrt(
//        lpf_rect_hpf_a[0]*lpf_rect_hpf_a[0] +
//        lpf_rect_hpf_a[1]*lpf_rect_hpf_a[1] +
//        lpf_rect_hpf_a[2]*lpf_rect_hpf_a[2]
//    );
//    accel_modulo_history_index++;
//    
//    if (accel_modulo_history_index >= 2*QUASISTATIC_HPF_SIZE) {
//        // transition to beginning of array required
//        memcpy(accel_modulo_history, &accel_modulo_history[QUASISTATIC_HPF_SIZE], QUASISTATIC_HPF_SIZE*sizeof(float)); 
//        accel_modulo_history_index = QUASISTATIC_HPF_SIZE;
//    }
//    if (accel_modulo_history_index >= QUASISTATIC_HPF_SIZE) {
//        // high-pass filter
//        arm_fir_f32(&quasistatic_hpf, 
//                    accel_modulo_history + accel_modulo_history_index - QUASISTATIC_HPF_SIZE,
//                    &accel_modulo_highpassed_history[accel_modulo_highpassed_history_index],
//                    1
//        ); 
//        // rectifier (abs)
//        accel_modulo_highpassed_history[accel_modulo_highpassed_history_index] = fabs(accel_modulo_highpassed_history[accel_modulo_highpassed_history_index]);
//        accel_modulo_highpassed_history_index++;
//        
//        if (accel_modulo_highpassed_history_index >= 2*QUASISTATIC_LPF_SIZE) {
//            // transition to beginning of array required
//            memcpy(accel_modulo_highpassed_history, &accel_modulo_highpassed_history[QUASISTATIC_LPF_SIZE], QUASISTATIC_LPF_SIZE*sizeof(float)); 
//            accel_modulo_highpassed_history_index = QUASISTATIC_LPF_SIZE;
//        }
//        if (accel_modulo_highpassed_history_index >= QUASISTATIC_LPF_SIZE) {
//            // low-pass filter
//            arm_fir_f32(&quasistatic_lpf,
//                        accel_modulo_highpassed_history + accel_modulo_highpassed_history_index - QUASISTATIC_LPF_SIZE,
//                        &detector[detector_index], 
//                        1
//            );
//            detector_index++;
//            if (detector_index == ACCEL_DECIMATION) {
//                detector_index = 0;
//            }
//            quasistatic_moment = 1;
//            for (i = 0; i < ACCEL_DECIMATION; i++) {
//                if (detector[i] > QUASISTATIC_THRESHOLD) {
//                    quasistatic_moment = 0;
//                }
//            }
//            if (quasistatic_moment) {
//                // quasistatic moment
//                // angle can be resetted
//            }
//        }
//    }
//}


//***************************************************
//
// Calibration of accelerometer using error model:
// A_meas = S * A_meas + o
//
//***************************************************

//void linear_model_accel_calibr() {
//    static float invS[9] = {1.0098, 0.02385, 0.00744, -0.00619, 1.00095, 0.03252, -0.00405, -0.02073, 1.00135};
//    static float offset[3] = {25.7667, 7.58333, 83.66666};
//    float tmp[3];
//    mat_sub(filtered_a, offset, tmp, 3, 1);
//    mat_mul(invS, tmp, calibrated_a, 3, 1, 3);
//}
