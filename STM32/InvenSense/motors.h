#ifndef MOTORS_H
#define MOTORS_H

#include "stdint.h"

typedef struct {
    int pwm1;
    int pwm2;
    int COUNT1;
    int COUNT2;
    uint8_t ENGRDY;
} Motors;

extern Motors motors;

void Motors_Run(void);
void Motors_Init(void);
void Motors_Stop(void);

#endif // MOTORS_H
