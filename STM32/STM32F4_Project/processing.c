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

float angleIntegral = 0;
float angleIntegralMax = 1e5;

float angleAcceleration = 0;
float prevAngularVelocity = 0;

float Aappr = 5.13e-7;
float Bappr = 0.001;
float Cappr = -1.81;
float D = 0;
float x1 = 0;
float x2 = 0;
float minF = 0.075;

float chooseRoot() {
    if (D < 0) return 2000;
    
    x1 = (-Bappr - sqrt(D)) / (2 * Aappr);
    x2 = (-Bappr + sqrt(D)) / (2 * Aappr);
    
    if (x1 > 1000 && x1 < 2000) return x1;
    if (x2 > 1000 && x2 < 2000) return x2;
    return 2000;
}

int8_t sign(int val) {
    if (val >= 0) return 1;
    if (val < 0) return -1;
}

uint8_t min(uint8_t val1, uint8_t val2) {
    return val1 > val2 ? val2 : val1;
}

uint8_t identificationReady = 0;


void calcPwms() {
    float absF = fabs(F);
    float absPwm = 0;
    // tilt of the line is changed
    if (absF < 0.8) {
        pwm = (absF + 3.299) / 0.0028;       
    } else {
        pwm = (absF + 1.1542) / 0.0013;
    }
    if (pwm > maxPwm) {
        pwm = maxPwm;
    }
    if (pwm < minPwm) {
        pwm = minPwm;
    }

    if (F > 0) {
        pwm1 = minPwm;
        pwm2 = pwm;
    } else {
        pwm2 = minPwm;
        pwm1 = pwm;
    }
//    D = Bappr*Bappr - 4*Aappr*(Cappr - fabs(F));
//    if (F > 0) {
//        pwm1 = minPwm;
//        pwm2 = (int)chooseRoot();
//    } else {
//        pwm2 = minPwm;
//        pwm1 = (int)chooseRoot();
//    }
}

#define ANGVEL_HISTORY_SIZE 5
float boundaryAngle = 0.05;
float angularVelocityHistory[ANGVEL_HISTORY_SIZE];
uint8_t angularVelocityHistoryIndex = 0;

void turnOffUselessMotor() {
//    uint8_t i = 0;
//    uint8_t angVelSign = 1;
//    float basic = angularVelocityHistory[0];
//    
//    if (turnUselessOn) { 
//        for (i = 1; i < ANGVEL_HISTORY_SIZE; i++) {
//            if (basic * angularVelocityHistory[i] < 0) {
//                angVelSign = 0;
//                break;
//            }
//        }
//        
//        if (angle > 0) {
//            if ((angle > boundaryAngle) && angVelSign && basic > 0) {
//                pwm1 = 1000;
//            }
//        } else {
//            if ((angle < -boundaryAngle) && angVelSign && basic < 0) {
//                pwm2 = 1000;
//            }
//        }
//    }
}

void control() {
//    if (angle > 0) {
//        pwm2 = Kp * fabs(angle) + Kd * angularVelocity + minPwm;
//        if (pwm2 > maxPwm) {
//            pwm2 = maxPwm;
//        }
//        pwm1 = minPwm;
//    } else {
//        pwm1 = Kp * fabs(angle) - Kd * angularVelocity + minPwm;
//        if (pwm1 > maxPwm) {
//            pwm1 = maxPwm;
//        }
//        pwm2 = minPwm;
//    }
//    turnOffUselessMotor();
//    Motors_Run();
    
//    F = Kp*angle + Kd*angularVelocity + minF;

    //if (stabilizationOn) {
        calcPwms();
        turnOffUselessMotor();
        Motors_Run();
    //}     
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
        //angleRate[i] = (ars_angleRate[0][i] - ars_angleRate[1][i]) / 2.0;    
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

void simplestControl(){
//    if (angle > 0) {
//        pwm2 = Kp * fabs(angle) + minPwm;
//        pwm1 = minPwm;
//        if (pwm2 > maxPwm) {
//            pwm2 = maxPwm;
//        } 
//    } else {
//        pwm1 = Kp * fabs(angle) + minPwm;
//        pwm2 = minPwm;
//        if (pwm1 > maxPwm) {
//            pwm1 = maxPwm;
//        }  
//    }
//    turnOffUselessMotor();
//    Motors_Run();
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


uint8_t tranquilityTime = 30;
uint8_t tranquilityCount = 0;
float angVelSmall = 0.1;
#define ANGLE_STEP 0.02

uint8_t gCheck() {
    float a2 = 0;
    float diff = 0;
    float threshold = 0;
    int i;
     threshold   = accelDeviation * G2;
    for(i=tranquilityTime;i<tranquilityCount;i++){
        threshold*=0.7;
    }
    a2          = finalAX * finalAX + finalAY * finalAY + finalAZ * finalAZ;
    diff        = abs(a2 - G2);
   
    return diff < threshold;
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
    
 //   angle[2] += eulerAngleRate[2] * 0.01;
//    uint8_t i = 0;
//    for(i = 0; i < 3; i++) {
//        angle[i] += eulerAngleRate[i] * 0.01;
//    }

    
//    float accelAngle = 0;
//    float angleDiff = 0;
//    // where to take angle from
//    // three steps of control:
//    // 1) sum Ai^2 < deviation * G^2
//    // 2) absolute value of angular velocity must be small
//    // 3) for 300 ms 
//    
//    if (gCheck() && (angularVelocity < angVelSmall)) {   
//        tranquilityCount++; 
//    } else {
//        tranquilityCount = 0;
//    }
//    if (tranquilityCount >= tranquilityTime) {
//        angle = atan((float)finalAX / (float)finalAZ);
//        accelAngle = atan((float)finalAX / (float)finalAZ);
////        angleDiff = angle - accelAngle;
////        if (angleDiff > 0) {
////            angle -= ANGLE_STEP;
////        } else {
////            angle += ANGLE_STEP;
////        }
//        angleFromAccel = 1;
//        tranquilityCount--;
//    } else {
//        angle += angularVelocity * DT;    
//        angleFromAccel = 0;
//    }
}

//void calcAngAccel() {
//    angleAcceleration = (angularVelocity - prevAngularVelocity) / DT;
//    prevAngularVelocity=angularVelocity;
//}

float Edes = 0;
int pwmStep = 10;

#define CONTROL_TIME_STEP 0.5
uint8_t everyN = 2;
#define TAU 1.0

void adjustControl() {
    //int8_t pwms = sign(pwm);
  //  int8_t edesS;
//    float absEdes = 0;
//    float t1 = 0;
//    if (- angularVelocity * TAU / angle / 2.0 < 1.0) {
//        Edes = - angularVelocity / TAU - angle / TAU / TAU;
//    } else {
//        t1 = - 2 * angle / angularVelocity;
//        Edes = - angularVelocity / t1;
//    }
////    edesS=sign(Edes);
////    absEdes=fabs(Edes);
////    angleAcceleration*=edesS;
////    if(absEdes>angleAcceleration)
////        pwm+=pwms*pwmStep;
////    else pwm-=pwms*pwmStep;
////    pwm = fabs(pwm);
//    if (Edes > 0) {
//        if (Edes > angleAcceleration) {
//            pwm += pwmStep;
//        } else {
//            pwm -= pwmStep;
//        }
//    } else {
//        if (Edes > angleAcceleration) {
//            pwm += pwmStep;
//        } else {
//            pwm -= pwmStep;
//        }
//    }
////    pwm *= pwms;
//    // pwm = fabs(pwm);
//    // pwm *= pwms;
//    
//    Motors_SetPwm();
//    Motors_Run();
}

Quat curRotation;
#include "adxrs453.h"

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
    Zroll[1] = eulerAngleRate[0];;
    Zpitch[0] = pitch;
    Zpitch[1] = eulerAngleRate[1];
    
    kalman2(A, Qroll, Rroll, Xroll, Zroll, Proll);
    kalman2(A, Qpitch, Rpitch, Xpitch, Zpitch, Ppitch);
    
    angle[0] = Xroll[0];
    angle[1] = Xpitch[0];
  
//    researchIndex++;
//    switch (research) {
//        case IMPULSE_RESPONSE:
//            if (researchIndex == 1) {
//                F = 1.0;
//                calcPwms();
//                Motors_Run();
//            } else if (researchIndex == 50) {
//                F = 0;
//                calcPwms();
//                Motors_Stop();
//                research = NO_RESEARCH;
//            }
//            break;
//        case STEP_RESPONSE:
//            if (researchIndex == 1) {
//                F = 1.0;
//                calcPwms();
//                Motors_Run();
//            }
//            break;
//        case SINE_RESPONSE:
//            F = researchAmplitude * sin(researchFrequency * researchIndex);
//            calcPwms();
//            Motors_Run();
//            break;
//        case EXP_RESPONSE:
//            F = researchAmplitude * exp(-researchFrequency * researchIndex);
//            calcPwms();
//            Motors_Run();
//            break;
//        case SIMPLE_CONTROL:
//            simplestControl();
//            break;
//        case PID_CONTROL:
//            control();
//            break;
//        case ADJUST_CONTROL:
//            if (researchIndex % everyN == 0) {
//                adjustControl();
//            }
//            break;
//        default:
//            researchIndex--;
//            break;
//    }
//    
    message.ars1_x = ars_angleRate[0][0];
    message.ars1_y = ars_angleRate[0][1];
    message.ars1_t = ars_data[0][2];
    
    message.ars2_x = ars_angleRate[1][0];
    message.ars2_y = ars_angleRate[1][1];
    message.ars2_t = ars_data[1][2];
    
    message.ars5_x = ars_angleRate[3][0];
    message.ars5_y = ars_angleRate[3][1];
    message.ars5_t = ars_data[3][2];
    
    message.accel_x = ax;
    message.accel_y = ay;
    message.accel_z = az;

    SendTelemetry(&message); 
}
