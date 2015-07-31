#ifndef PROCESSING_H
#define PROCESSING_H

#include "stdint.h"

extern float F;
extern float Kp;
extern float Kd;
extern float Ki;
extern float angle;
extern float angularVelocity;

extern float Ax;
extern float Ay;
extern float Az;

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

extern uint8_t STABRDY;

void kalman(void);
void angleAveraging(void);
void angVelAveraging(void);
void control(void);
void process(void);

void allocAngleAveraging(uint8_t newSize);
void allocAngVelAveraging(uint8_t newSize);
void allocAveraging(void);


#define MEASUREMENT_TIME 0.01

#endif
