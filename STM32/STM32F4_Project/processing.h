#ifndef PROCESSING_H
#define PROCESSING_H

#include "stdint.h"

extern double F;
extern double k1;
extern double k2;
extern double angle;
extern double angularVelocity;

extern double Ax;
extern double Ay;
extern double Az;

extern uint8_t stabilizationOn;
extern uint8_t kalmanOn;
extern uint8_t averagingOn;

void kalman(void);
void averaging(void);
void control(void);
void process(void);


#define MEASUREMENT_TIME 0.01

#endif
