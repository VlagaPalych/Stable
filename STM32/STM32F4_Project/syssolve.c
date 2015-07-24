#include "syssolve.h"
#include "string.h"
#include "stdlib.h"
#include "stm32f4xx.h"

float y[3];                // 3 last angles
float u[3];                // 3 last thrusts

float w[3];                // unknown coeffs in diff eq
float Afull[10*3];         // matrix of our equation Afull * w = Bfull
float Bfull[10];           // right column

uint8_t anglesAccumulated;  // number of angles we have at the moment
uint8_t row;

float det3(float *mat)
{
    float t1 = 0, t2 = 0, t3 = 0, part1 = 0, part2 = 0, part3 = 0, retval = 0;
    t1 = mat[1*3 + 1]*mat[2*3 + 2];
    t2 = mat[1*3 + 2]*mat[2*3 + 1];
    t3 = t1 - t2;
    part1 = mat[0*3 + 0]*t3;
    t1 = mat[1*3 + 0]*mat[2*3 + 2];
    t2 = mat[1*3 + 2]*mat[2*3 + 0];
    t3 = t1 - t2;
    part2 = mat[0*3 + 1]*t3;
    t1 = mat[1*3 + 0]*mat[2*3 + 1];
    t2 = mat[1*3 + 1]*mat[2*3 + 0];
    t3 = t1 - t2;
    part3 = mat[0*3 + 2]*t3;
    retval = part1 - part2 + part3;
    return retval;
}

/* A - matrix n x m */
void transpose(float *A, float *At, int n, int m) {
    int i, j;
    for (i = 0; i < n; i++) {
        for (j = 0; j < m; j++) {
            At[j*n + i] = A[i*m + j];
        }
    }
}

void mat_add(const float *A, const float *B, float *C, int n, int m) {
    int i, j;
    for (i = 0; i < n; i++) {
        for (j = 0; j < m; j++) {
            C[i*m + j] = A[i*m + j] + B[i*m + j];
        }
    }
}

void mat_sub(const float *A, const float *B, float *C, int n, int m) {
    int i, j;
    for (i = 0; i < n; i++) {
        for (j = 0; j < m; j++) {
            C[i*m + j] = A[i*m + j] - B[i*m + j];
        }
    }
}

/* A - matrix n x k 
   B - matrix k x m */
void mat_mul(const float *A, const float *B, float *C, int n, int m, int k) {
    int i, j, h;
    for (i = 0; i < n; i++) {
        for (j = 0; j < m; j++) {
            C[i*m + j] = 0;
            for (h = 0; h < k; h++) {
                C[i*m + j] += A[i*k + h] * B[h*m + j];  
            }
        }
    }
}

void system_solve(float *F, float *B, float *x, int n, int m) {
    float *Ft;
    float *A_3x3;
    float *b;
    float det, detI;
    float *tmp;
    int i, j;

    Ft = (float *)malloc(n*m*sizeof(float));
    transpose(F, Ft, n, m);
    for (i = 0; i < 10; i++) {
        for (j = 0; j < 3; j++) {
            if (F[i*3 + j] != Ft[j*10 + i]) {
                GPIOD->BSRRL |= 1 << 15;
            }
        }
    }

    A_3x3 = (float *)malloc(m*m*sizeof(float));
    b = (float *)malloc(m*sizeof(float));

    mat_mul(Ft, F, A_3x3, m, m, n);
    mat_mul(Ft, B, b, m, 1, n);

    det = det3(A_3x3);
    if (det == 0) return;

    tmp = (float *)malloc(m*m*sizeof(float));

    for (i = 0; i < m; i++) {
        memcpy(tmp, A_3x3, m*m*sizeof(float));
        for (j = 0; j < m; j++) {
            tmp[j*m + i] = b[j];
        }
         
        detI = det3(tmp);
        x[i] = detI / det;
    }

    free(Ft);
    free(A_3x3);
    free(b);
    free(tmp);
}
