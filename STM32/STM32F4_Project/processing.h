#ifndef PROCESSING_H
#define PROCESSING_H

#include "stdint.h"

extern float maxAngle;

extern float F;
extern float Kp;
extern float Kd;
extern float Ki;
extern float angle;
extern float angularVelocity;

extern float gyroX;
extern float gyroY;
extern float gyroZ;

extern uint8_t stabilizationOn;
extern uint8_t kalmanOn;
extern uint8_t angleAveragingOn;
extern uint8_t angVelAveragingOn;
extern uint8_t impulseOn;

extern uint8_t angleWindowSize;
extern uint8_t angVelWindowSize;
extern uint8_t angleIndex;
extern float angleSum;
extern uint8_t angVelIndex;
extern float angVelSum;

#define HISTORY_SIZE 256
#define ACCEL_FILTER_SIZE 107
#define GYRO_FILTER_SIZE 84

extern float accel_b[ACCEL_FILTER_SIZE];
extern float gyro_b[GYRO_FILTER_SIZE];

extern int16_t axHistory[HISTORY_SIZE];
extern uint8_t axHistoryIndex;
extern uint8_t axCurHistoryIndex;
extern int16_t finalAX;

extern int16_t azHistory[HISTORY_SIZE];
extern uint8_t azHistoryIndex;
extern uint8_t azCurHistoryIndex;
extern int16_t finalAZ;

extern int16_t gxHistory[HISTORY_SIZE];
extern uint8_t gxHistoryIndex;
extern uint8_t gxCurHistoryIndex;
extern float filteredGX;  

extern uint8_t doAccelProcess;
extern uint8_t doGyroProcess;
extern uint8_t lowpassOn;
extern uint8_t accelLowpassReady;
extern uint8_t gyroLowpassReady;

extern uint8_t processCounter;

typedef enum {IMPULSE_RESPONSE, STEP_RESPONSE, SINE_RESPONSE, EXP_RESPONSE, NO_RESEARCH, SIMPLE_CONTROL} researchType;
extern researchType research;
extern float researchAmplitude;
extern float researchFrequency;
extern uint32_t researchIndex;

void kalman(void);
void angleAveraging(void);
void angVelAveraging(void);
void control(void);
void process(void);

void allocAngleAveraging(uint8_t newSize);
void allocAngVelAveraging(uint8_t newSize);
void allocAveraging(void);

void Processing_TIM_Init(void);
float lowpass(int16_t *history, uint8_t lowpassIndex, float *fir, uint8_t firSize);

void transformGyroData(void);

#define MEASUREMENT_TIME 0.01

#endif
