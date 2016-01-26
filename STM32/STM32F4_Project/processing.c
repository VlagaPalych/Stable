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

float adxrs_b[ADXRS_FILTER_SIZE] = {0.000765996615305244, 0.000660820151020175, 0.000904398442903659, 0.00116993648900164, 0.00144225289999618, 0.00170146983193675, 0.00192506416920931, 0.00208637960406920, 0.00215549334724139, 0.00210628189455340, 0.00190929130872978, 0.00154175764366672, 0.000985379572064331, 0.000231037660150656, -0.000720975242728358, -0.00185800385190914, -0.00315400290658119, -0.00456986885279512, -0.00605129658439567, -0.00753137446367871, -0.00893102356144855, -0.0101624543586776, -0.0111314200131555, -0.0117417714988076, -0.0118994911274908, -0.0115170351071913, -0.0105188708579065, -0.00884554022866118, -0.00645838490726134, -0.00334261386423469, 0.000489473606544453, 0.00499731904300335, 0.0101115226151986, 0.0157346375919648, 0.0217436975271735, 0.0279935671243145, 0.0343218642376052, 0.0405544117337099, 0.0465120006171619, 0.0520171194154150, 0.0569010809190037, 0.0610109684009580, 0.0642158515277905, 0.0664123296154894, 0.0675288324909505, 0.0675288324909505, 0.0664123296154894, 0.0642158515277905, 0.0610109684009580, 0.0569010809190037, 0.0520171194154150, 0.0465120006171619, 0.0405544117337099, 0.0343218642376052, 0.0279935671243145, 0.0217436975271735, 0.0157346375919648, 0.0101115226151986, 0.00499731904300335, 0.000489473606544453, -0.00334261386423469, -0.00645838490726134, -0.00884554022866118, -0.0105188708579065, -0.0115170351071913, -0.0118994911274908, -0.0117417714988076, -0.0111314200131555, -0.0101624543586776, -0.00893102356144855, -0.00753137446367871, -0.00605129658439567, -0.00456986885279512, -0.00315400290658119, -0.00185800385190914, -0.000720975242728358, 0.000231037660150656, 0.000985379572064331, 0.00154175764366672, 0.00190929130872978, 0.00210628189455340, 0.00215549334724139, 0.00208637960406920, 0.00192506416920931, 0.00170146983193675, 0.00144225289999618, 0.00116993648900164, 0.000904398442903659, 0.000660820151020175, 0.000765996615305244};
int16_t adxrsHistory[HISTORY_SIZE];
uint8_t adxrsHistoryIndex = 0;
uint8_t adxrsCurHistoryIndex = 0;
float filteredVel = 0;
    
    
// accel FIR-filter coefficients
float accel_b[ACCEL_FILTER_SIZE] = {-0.000971251596459908, -7.69406717604047e-05, -6.19799209654469e-05, -3.30906543649738e-05, 1.10814561797306e-05, 7.24805531343648e-05, 0.000152598071257586, 0.000253698805736882, 0.000377317468335945, 0.000525701202486072, 0.000700518195310734, 0.000903997339458505, 0.00113739692160716, 0.00140312152077752, 0.00170195103075679, 0.00203566452362004, 0.00240546060485815, 0.00281231733030840, 0.00325638090562598, 0.00373853563596467, 0.00425842477132322, 0.00481595944431221, 0.00541027647136111, 0.00604061130952030, 0.00670524975565512, 0.00740308239924367, 0.00813154544121787, 0.00888873230285860, 0.00967096533068780, 0.0104756034830591, 0.0112971993347504, 0.0121352733530491, 0.0129849453923593, 0.0138385519287171, 0.0146955841292333, 0.0155486053454725, 0.0163942550438339, 0.0172263771763229, 0.0180408483356263, 0.0188316697226914, 0.0195946307655523, 0.0203241247156848, 0.0210158009852968, 0.0216645459211678, 0.0222667388005664, 0.0228165591799628, 0.0233122609941272, 0.0237489436996918, 0.0241240737240030, 0.0244343475043236, 0.0246784705527375, 0.0248538046118870, 0.0249598963537244, 0.0249951385161779, 0.0249598963537244, 0.0248538046118870, 0.0246784705527375, 0.0244343475043236, 0.0241240737240030, 0.0237489436996918, 0.0233122609941272, 0.0228165591799628, 0.0222667388005664, 0.0216645459211678, 0.0210158009852968, 0.0203241247156848, 0.0195946307655523, 0.0188316697226914, 0.0180408483356263, 0.0172263771763229, 0.0163942550438339, 0.0155486053454725, 0.0146955841292333, 0.0138385519287171, 0.0129849453923593, 0.0121352733530491, 0.0112971993347504, 0.0104756034830591, 0.00967096533068780, 0.00888873230285860, 0.00813154544121787, 0.00740308239924367, 0.00670524975565512, 0.00604061130952030, 0.00541027647136111, 0.00481595944431221, 0.00425842477132322, 0.00373853563596467, 0.00325638090562598, 0.00281231733030840, 0.00240546060485815, 0.00203566452362004, 0.00170195103075679, 0.00140312152077752, 0.00113739692160716, 0.000903997339458505, 0.000700518195310734, 0.000525701202486072, 0.000377317468335945, 0.000253698805736882, 0.000152598071257586, 7.24805531343648e-05, 1.10814561797306e-05, -3.30906543649738e-05, -6.19799209654469e-05, -7.69406717604047e-05, -0.000971251596459908};

int16_t azHistory[HISTORY_SIZE];
uint8_t azHistoryIndex = 0;
uint8_t azCurHistoryIndex = 0;
int16_t filteredAZ = 0;
int16_t finalAZ = 0;
    
int16_t axHistory[HISTORY_SIZE];
uint8_t axHistoryIndex = 0;
uint8_t axCurHistoryIndex = 0;
int16_t filteredAX = 0;
int16_t finalAX = 0;
int16_t finalAY = 0;
    
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

void getFinalData() {
    if (lowpassOn) {
        finalAX = filteredAX;
        finalAZ = filteredAZ;
    } else {
        finalAX = ax;
        finalAZ = az;
    }
    finalAY = ay;
}


float roll = 0;
float pitch = 0;

void calcAngle() {
    float gx = finalAX;
    float gy = finalAY;
    float gz = finalAZ;
    
    roll    = atanf(gy / gz);
    pitch   = atanf(- gx / sqrt(gy*gy + gz*gz));
    
    roll    *= 180.0 / 3.14159;
    pitch   *= 180.0 / 3.14159;
}

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


void TIM7_IRQHandler(void) {
    TIM7->SR &= ~TIM_SR_UIF;
    
    getFinalData();
    calcAngleRate(); 
    calcAngle();
       
    Zroll[0] = roll;
    Zroll[1] = eulerAngleRate[0];
    Zpitch[0] = pitch;
    Zpitch[1] = eulerAngleRate[1];
    
    kalman2(A, Qroll, Rroll, Xroll, Zroll, Proll);
    kalman2(A, Qpitch, Rpitch, Xpitch, Zpitch, Ppitch);
    
    angle[0] = Xroll[0];
    angle[1] = Xpitch[0];
  
    researchIndex++;
    switch (research) {
        case IMPULSE_RESPONSE:
            if (researchIndex == 1) {
                F = 1.0;
                Motors_CalcPwm();
                Motors_Run();
            } else if (researchIndex == 50) {
                F = 0;
                Motors_CalcPwm();
                Motors_Stop();
                research = NO_RESEARCH;
            }
            break;
        case STEP_RESPONSE:
            if (researchIndex == 1) {
                F = 1.0;
                Motors_CalcPwm();
                Motors_Run();
            }
            break;
        case SINE_RESPONSE:
            F = researchAmplitude * sin(researchFrequency * researchIndex);
            Motors_CalcPwm();
            Motors_Run();
            break;
        case EXP_RESPONSE:
            F = researchAmplitude * exp(-researchFrequency * researchIndex);
            Motors_CalcPwm();
            Motors_Run();
            break;
        case PID_CONTROL:
            pid_control();
            break;
        default:
            researchIndex--;
            break;
    }
    
    message.angle = Xroll[0];
    message.angleRate = Xroll[1];
    message.pwm1 = pwm1;
    message.pwm2 = pwm2;
    message.freq1 = COUNT1;
    message.freq2 = COUNT2;

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
