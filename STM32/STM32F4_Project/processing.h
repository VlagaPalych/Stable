#ifndef PROCESSING_H
#define PROCESSING_H

#include "stdint.h"

extern float F;
extern float k1;
extern float k2;
extern float angle;
extern float angularVelocity;

extern float Ax;
extern float Ay;
extern float Az;

extern uint8_t stabilizationOn;
extern uint8_t kalmanOn;
extern uint8_t averagingOn;
extern uint8_t impulseOn;

extern uint8_t STABRDY;

void kalman(void);
void averaging(void);
void control(void);
void process(void);


#define MEASUREMENT_TIME 0.01

#endif
