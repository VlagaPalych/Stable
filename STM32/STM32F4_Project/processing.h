#ifndef PROCESSING_H
#define PROCESSING_H

#include "stdint.h"
#include "quaternion.h"

extern float roll;
extern float pitch;

extern Quat orient;

extern float angleRate[3];
extern float angle[3];

extern int pwm;
extern uint8_t everyN;
extern float angleAcceleration;
extern float Edes;

extern uint8_t tranquilityTime;

extern float maxAngle;
extern float accelDeviation;
extern uint8_t angleFromAccel;
extern float boundaryAngle;
extern float maxAngVel;

extern float F;
extern float Kp;
extern float Kd;
extern float Ki;

extern uint8_t stabilizationOn;

extern float pitchGyr, rollGyr;
extern float phi_x;
extern float phi_y;

extern float lpf_rect_hpf_a[3];
extern uint8_t quasistatic_new_a;

#define HISTORY_SIZE 256
#define ADXRS_FILTER_SIZE 90
#define ACCEL_FILTER_SIZE 107
#define GYRO_FILTER_SIZE 84

extern float adxrs_b[ADXRS_FILTER_SIZE];
extern int16_t adxrsHistory[HISTORY_SIZE];
extern uint8_t adxrsHistoryIndex;
extern uint8_t adxrsCurHistoryIndex;
extern float filteredVel;

extern float accel_b[ACCEL_FILTER_SIZE];

//extern int16_t accel_history[3][HISTORY_SIZE];
//extern uint8_t accel_historyIndex[3];
//extern uint8_t accel_curHistoryIndex[3];
//extern int16_t filtered_a[3];
extern float final_a[3];

extern uint8_t doAccelProcess;
extern uint8_t doGyroProcess;
extern uint8_t doAdxrsProcess;

extern uint8_t lowpassOn;
extern uint8_t accelLowpassReady;
extern uint8_t gyroLowpassReady;

extern uint8_t processCounter;

extern int pwmStep;

typedef enum {IMPULSE_RESPONSE, STEP_RESPONSE, SINE_RESPONSE, EXP_RESPONSE, NO_RESEARCH, 
                SIMPLE_CONTROL, PID_CONTROL, OPERATOR_CONTROL, ADJUST_CONTROL} researchType;
extern researchType research;
extern float researchAmplitude;
extern float researchFrequency;
extern uint32_t researchIndex;

void control(void);
void process(void);

void Processing_TIM_Init(void);
float lowpass(int16_t *history, uint8_t lowpassIndex, float *fir, int firSize);

void transformGyroData(void);
void checkCalibrationFinish(void);      

void lpf_rect_hpf(void);                


#define MEASUREMENT_TIME 0.01

#endif
