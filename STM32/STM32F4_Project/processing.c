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
#include "gyro.h"

#define DT MEASUREMENT_TIME
#define G2 75746

float maxAngle = 0.26;

researchType research = NO_RESEARCH;
uint32_t researchIndex = 0;
float researchAmplitude = 0;
float researchFrequency = 0;

uint8_t processCounter = 0;

float gyro_b[GYRO_FILTER_SIZE] = {-0.000687685100567948, -0.000503796863668868, -0.000669665666328788, -0.000855276751476558, -0.00105554993475612, -0.00126508273334009, -0.00147593999534209, -0.00167849138703039, -0.00185916913593240, -0.00200684865461460, -0.00210530759211669, -0.00213822286546893, -0.00208958424584653, -0.00194067647281183, -0.00167455480891157, -0.00127360844400499, -0.000722641774982278, -6.87927808112551e-06, 0.000885291093669463, 0.00196280139735759, 0.00323181070591218, 0.00469392489725397, 0.00634689783560797, 0.00818380308704172, 0.0101930785397887, 0.0123584580913614, 0.0146590118621400, 0.0170698639054851, 0.0195615456872260, 0.0221015359753715, 0.0246541129281018, 0.0271813574961538, 0.0296440798312156, 0.0320025708463445, 0.0342176988964603, 0.0362515564208577, 0.0380687255412841, 0.0396368301799685, 0.0409274060866639, 0.0419169338508330, 0.0425870017582913, 0.0429251965156089, 0.0429251965156089, 0.0425870017582913, 0.0419169338508330, 0.0409274060866639, 0.0396368301799685, 0.0380687255412841, 0.0362515564208577, 0.0342176988964603, 0.0320025708463445, 0.0296440798312156, 0.0271813574961538, 0.0246541129281018, 0.0221015359753715, 0.0195615456872260, 0.0170698639054851, 0.0146590118621400, 0.0123584580913614, 0.0101930785397887, 0.00818380308704172, 0.00634689783560797, 0.00469392489725397, 0.00323181070591218, 0.00196280139735759, 0.000885291093669463, -6.87927808112551e-06, -0.000722641774982278, -0.00127360844400499, -0.00167455480891157, -0.00194067647281183, -0.00208958424584653, -0.00213822286546893, -0.00210530759211669, -0.00200684865461460, -0.00185916913593240, -0.00167849138703039, -0.00147593999534209, -0.00126508273334009, -0.00105554993475612, -0.000855276751476558, -0.000669665666328788, -0.000503796863668868, -0.000687685100567948};

int16_t gxHistory[HISTORY_SIZE];
uint8_t gxHistoryIndex = 0;
uint8_t gxCurHistoryIndex = 0;
float filteredGX = 0;    
    
// accel FIR-filter coefficients
float accel_b[ACCEL_FILTER_SIZE] = {-0.000971251596459908, -7.69406717604047e-05, -6.19799209654469e-05, -3.30906543649738e-05, 1.10814561797306e-05, 7.24805531343648e-05, 0.000152598071257586, 0.000253698805736882, 0.000377317468335945, 0.000525701202486072, 0.000700518195310734, 0.000903997339458505, 0.00113739692160716, 0.00140312152077752, 0.00170195103075679, 0.00203566452362004, 0.00240546060485815, 0.00281231733030840, 0.00325638090562598, 0.00373853563596467, 0.00425842477132322, 0.00481595944431221, 0.00541027647136111, 0.00604061130952030, 0.00670524975565512, 0.00740308239924367, 0.00813154544121787, 0.00888873230285860, 0.00967096533068780, 0.0104756034830591, 0.0112971993347504, 0.0121352733530491, 0.0129849453923593, 0.0138385519287171, 0.0146955841292333, 0.0155486053454725, 0.0163942550438339, 0.0172263771763229, 0.0180408483356263, 0.0188316697226914, 0.0195946307655523, 0.0203241247156848, 0.0210158009852968, 0.0216645459211678, 0.0222667388005664, 0.0228165591799628, 0.0233122609941272, 0.0237489436996918, 0.0241240737240030, 0.0244343475043236, 0.0246784705527375, 0.0248538046118870, 0.0249598963537244, 0.0249951385161779, 0.0249598963537244, 0.0248538046118870, 0.0246784705527375, 0.0244343475043236, 0.0241240737240030, 0.0237489436996918, 0.0233122609941272, 0.0228165591799628, 0.0222667388005664, 0.0216645459211678, 0.0210158009852968, 0.0203241247156848, 0.0195946307655523, 0.0188316697226914, 0.0180408483356263, 0.0172263771763229, 0.0163942550438339, 0.0155486053454725, 0.0146955841292333, 0.0138385519287171, 0.0129849453923593, 0.0121352733530491, 0.0112971993347504, 0.0104756034830591, 0.00967096533068780, 0.00888873230285860, 0.00813154544121787, 0.00740308239924367, 0.00670524975565512, 0.00604061130952030, 0.00541027647136111, 0.00481595944431221, 0.00425842477132322, 0.00373853563596467, 0.00325638090562598, 0.00281231733030840, 0.00240546060485815, 0.00203566452362004, 0.00170195103075679, 0.00140312152077752, 0.00113739692160716, 0.000903997339458505, 0.000700518195310734, 0.000525701202486072, 0.000377317468335945, 0.000253698805736882, 0.000152598071257586, 7.24805531343648e-05, 1.10814561797306e-05, -3.30906543649738e-05, -6.19799209654469e-05, -7.69406717604047e-05, -0.000971251596459908};

int16_t azHistory[HISTORY_SIZE];
uint8_t azHistoryIndex = 0;
uint8_t azCurHistoryIndex = 0;
int16_t finalAZ = 0;
    
int16_t axHistory[HISTORY_SIZE];
uint8_t axHistoryIndex = 0;
uint8_t axCurHistoryIndex = 0;
int16_t finalAX = 0;
    
uint8_t doAccelProcess = 0;
uint8_t doGyroProcess = 0;
    
uint8_t lowpassOn = 1;
uint8_t accelLowpassReady = 0;
uint8_t gyroLowpassReady = 0;

float prevAngle = 0;
float angleIntegral = 0;
float angleIntegralMax = 1e5;

uint8_t stabilizationOn     = 0;
uint8_t kalmanOn            = 0;
uint8_t angleAveragingOn    = 0;
uint8_t angVelAveragingOn   = 0;
uint8_t impulseOn           = 0;

uint8_t angleWindowSize     = 8;
uint8_t angVelWindowSize    = 8;

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

uint8_t angleIndex = 0;
float angleSum = 0;
float *angles = NULL;
uint8_t angVelIndex = 0;
float angVelSum = 0;
float *angVels = NULL;

uint8_t min(uint8_t val1, uint8_t val2) {
    return val1 > val2 ? val2 : val1;
}

void allocAngleAveraging(uint8_t newSize) {
    uint8_t shift = 0, j = 0;
    float *newAngles = (float *)malloc(newSize * sizeof(float));
    
    if (newSize > angleWindowSize) {
        memcpy(newAngles, angles, angleWindowSize * sizeof(float));
    } else {
        shift = angleWindowSize - newSize;
        memcpy(newAngles, angles + shift, newSize * sizeof(float));
        for (j = 0; j < shift; j++) {
            angleSum -= angles[j];
        }
        angleIndex %= newSize;
    }
    free(angles);
    angles = newAngles;
    angleWindowSize = newSize;
}

void allocAngVelAveraging(uint8_t newSize) {
    uint8_t shift = 0, j = 0;
    float *newAngVels = (float *)malloc(newSize * sizeof(float));
    
    if (newSize > angVelWindowSize) {
        memcpy(newAngVels, angVels, angVelWindowSize * sizeof(float));
    } else {
        shift = angVelWindowSize - newSize;
        memcpy(newAngVels, angVels + shift, newSize * sizeof(float));
        for (j = 0; j < shift; j++) {
            angVelSum -= angVels[j];
        }
        angVelIndex %= newSize;
    }
    free(angVels);
    angVels = newAngVels;
    angVelWindowSize = newSize;
}

void allocAveraging() {
    angles = (float *)malloc(angleWindowSize * sizeof(float));
    memset(angles, 0, angleWindowSize * sizeof(float));
    angVels = (float *)malloc(angVelWindowSize * sizeof(float));
    memset(angVels, 0, angVelWindowSize * sizeof(float));
}

void angleAveraging() {
    angleSum -= angles[angleIndex];
    angles[angleIndex] = angle;
    angleSum += angle;
    angle = angleSum / angleWindowSize;
    angles[angleIndex] = angle;
    angleIndex = (angleIndex + 1) % angleWindowSize;
}

void angVelAveraging() {
    angVelSum -= angVels[angVelIndex];
    angVels[angVelIndex] = angularVelocity;
    angVelSum += angularVelocity;
    angularVelocity = angVelSum / angVelWindowSize;
    angVels[angVelIndex] = angularVelocity;
    angVelIndex = (angVelIndex + 1) % angVelWindowSize;
}

uint8_t identificationReady = 0;

void calcPwms() {
    D = Bappr*Bappr - 4*Aappr*(Cappr - fabs(F));
    if (F > 0) {
        pwm1 = minPwm;
        pwm2 = (int)chooseRoot();
    } else {
        pwm2 = minPwm;
        pwm1 = (int)chooseRoot();
    }
}

void control() {
    F = Kp*angle + Kd*angularVelocity + Ki*angleIntegral;

    if (stabilizationOn) {
        calcPwms();
        Motors_Run();
    }     
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

    
}

float gyroX = 0;
float gyroY = 0;
float gyroZ = 0;

void transformGyroData() {
    if (lowpassOn) {
        gyroX = filteredGX / 32767.0 * 2000 * 3.14159 / 180.0;
    } else {
        gyroX = gx / 32767.0 * 2000 * 3.14159 / 180.0;
    }
    gyroY = gy / 32767.0 * 2000 * 3.14159 / 180.0;
    gyroZ = gz / 32767.0 * 2000 * 3.14159 / 180.0;
}

float lowpass(int16_t *history, uint8_t lowpassIndex, float *fir, uint8_t firSize) {
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
    TIM7->PSC = 7;
    TIM7->ARR = 10000;
    TIM7->DIER |= 1;
    NVIC_SetPriority(TIM7_IRQn, 0xFF);
    NVIC_EnableIRQ(TIM7_IRQn);
    
    TIM7->CR1 |= TIM_CR1_CEN;
}

void simplestControl(){
    if (angle > 0) {
        pwm2 = maxPwm * fabs(angle) / maxAngle + minPwm;
        pwm1 = minPwm;
        if (pwm2 > maxPwm) {
            pwm2 = maxPwm;
        }
    } else {
        pwm1 = maxPwm * fabs(angle) / maxAngle + minPwm;
        pwm2 = minPwm;
        if (pwm1 > maxPwm) {
            pwm1 = maxPwm;
        }
    }
    Motors_Run();
}

float a2 = 0;

void TIM7_IRQHandler(void) {
    TIM7->SR &= ~TIM_SR_UIF;
    
    GPIOD->BSRRL |= 1 << 15;
    transformGyroData();
      
    if (lowpassOn) {
        a2 = finalAX * finalAX + ay * ay + finalAZ * finalAZ;
    } else {
        a2 = ax*ax + ay*ay + az*az;
    }
    //if (abs(a2 - G2) < 0.25 * G2) {
//        if (lowpassOn) {
//            gyroAngle = atan((float)finalAX / (float)finalAZ);
//        } else {
//            gyroAngle = atan((float)ax / (float)az);
//        }
    //} else {
        gyroAngle += gyroX * DT;
    //}
    
    angle = gyroAngle;
    angularVelocity = gyroX;   
 
    angleIntegral += angle;
    if (angleIntegral > angleIntegralMax) {
        angleIntegral = angleIntegralMax;
    }
    
    researchIndex++;
    switch (research) {
        case IMPULSE_RESPONSE:
            if (researchIndex == 1) {
                F = 1.0;
                calcPwms();
                Motors_Run();
            } else if (researchIndex == 50) {
                Motors_Stop();
                research = NO_RESEARCH;
            }
            break;
        case STEP_RESPONSE:
            if (researchIndex == 1) {
                F = 1.0;
                calcPwms();
                Motors_Run();
            }
            break;
        case SINE_RESPONSE:
            F = researchAmplitude * sin(researchFrequency * researchIndex);
            calcPwms();
            Motors_Run();
            break;
        case EXP_RESPONSE:
            F = researchAmplitude * exp(-researchFrequency * researchIndex);
            calcPwms();
            Motors_Run();
            break;
        case NO_RESEARCH:
            researchIndex--;
            control();
            break;
        case SIMPLE_CONTROL:
            simplestControl();
            break;
    }
    
    
    SendTelemetry();
    GPIOD->BSRRH |= 1 << 15;
}
