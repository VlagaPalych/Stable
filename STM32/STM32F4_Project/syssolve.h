#ifndef SYSSOLVE_H
#define SYSSOLVE_H

#include "stdint.h"

// Identification variables
extern double y[3];                // 3 last angles
extern double u[3];                // 3 last thrusts

extern double w[3];                // unknown coeffs in diff eq
extern double Afull[10*3];         // matrix of our equation Afull * w = Bfull
extern double Bfull[10];           // right column

extern uint8_t anglesAccumulated;  // number of angles we have at the moment
extern uint8_t row;
//////////////////////

double det3(double *mat);
void transpose(double *A, double *At, int n, int m);
void mat_add(const double *A, const double *B, double *C, int n, int m);
void mat_sub(const double *A, const double *B, double *C, int n, int m);
void mat_mul(const double *A, const double *B, double *C, int n, int m, int k);
void system_solve(double *F, double *B, double *x, int n, int m);

#endif
