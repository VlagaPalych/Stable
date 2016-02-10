#ifndef SYSSOLVE_H
#define SYSSOLVE_H

#include "stdint.h"

// Identification variables
extern float y[3];                // 3 last angles
extern float u[3];                // 3 last thrusts

extern float w[3];                // unknown coeffs in diff eq
extern float Afull[10*3];         // matrix of our equation Afull * w = Bfull
extern float Bfull[10];           // right column

extern uint8_t anglesAccumulated;  // number of angles we have at the moment
extern uint8_t row;
//////////////////////

float det3(float *mat);
void transpose(const float *A, float *At, int m, int n);
void mat_add(const float *A, const float *B, float *C, int m, int n);
void mat_sub(const float *A, const float *B, float *C, int m, int n);
void mat_mul(const float *A, const float *B, float *C, int m, int n, int k);
void system_solve(float *F, float *B, float *x, int n, int m);

void mat2_inv(const float *A, float *Ainv);

#endif
