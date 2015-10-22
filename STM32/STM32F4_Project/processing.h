#ifndef PROCESSING_H
#define PROCESSING_H

#include "stdint.h"
#include "quaternion.h"

extern Quat orient;

extern float angleRate[3];
extern float angle[3];

extern int pwm;
extern uint8_t everyN;
extern float angleAcceleration;
extern float Edes;

#define GYRO_RECALIBRATION_BUFFER_SIZE 1500
extern float gyroRecalibrationAccumulator;

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

extern int16_t axHistory[HISTORY_SIZE];
extern uint8_t axHistoryIndex;
extern uint8_t axCurHistoryIndex;
extern int16_t finalAX;
extern int16_t finalAY;
extern int16_t filteredAX;
extern int16_t filteredAY;

extern int16_t azHistory[HISTORY_SIZE];
extern uint8_t azHistoryIndex;
extern uint8_t azCurHistoryIndex;
extern int16_t finalAZ;
extern int16_t filteredAZ;

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


#define MEASUREMENT_TIME 0.01

#endif
