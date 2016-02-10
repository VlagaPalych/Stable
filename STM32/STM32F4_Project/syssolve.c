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

float det3(float *mat) {
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

float det4(float *a) {
    float t1 = 0, t2 = 0, t3 = 0,part1 = 0, part2 = 0, part3 = 0, part4 = 0;
    
    t1 = a[1*4 + 1]*(a[2*4 + 2]*a[3*4 + 3] - a[3*4 + 2]*a[2*4 + 3]);
    t2 = a[1*4 + 2]*(a[2*4 + 1]*a[3*4 + 3] - a[3*4 + 1]*a[2*4 + 3]);
    t3 = a[1*4 + 3]*(a[2*4 + 1]*a[3*4 + 2] - a[3*4 + 1]*a[2*4 + 2]);
    part1 = a[0*4 + 0]*(t1 - t2 + t3);
    
    t1 = a[1*4 + 0]*(a[2*4 + 2]*a[3*4 + 3] - a[3*4 + 2]*a[2*4 + 3]);
    t2 = a[1*4 + 2]*(a[2*4 + 0]*a[3*4 + 3] - a[3*4 + 0]*a[2*4 + 3]);
    t3 = a[1*4 + 3]*(a[2*4 + 0]*a[3*4 + 2] - a[3*4 + 0]*a[2*4 + 2]);
    part2 = a[0*4 + 1]*(t1 - t2 + t3);
    
    t1 = a[1*4 + 0]*(a[2*4 + 1]*a[3*4 + 3] - a[3*4 + 1]*a[2*4 + 3]);
    t2 = a[1*4 + 1]*(a[2*4 + 0]*a[3*4 + 3] - a[3*4 + 0]*a[2*4 + 3]);
    t3 = a[1*4 + 3]*(a[2*4 + 0]*a[3*4 + 1] - a[3*4 + 0]*a[2*4 + 1]);
    part3 = a[0*4 + 2]*(t1 - t2 + t3);
    
    t1 = a[1*4 + 0]*(a[2*4 + 1]*a[3*4 + 2] - a[3*4 + 1]*a[2*4 + 2]);
    t2 = a[1*4 + 1]*(a[2*4 + 0]*a[3*4 + 2] - a[3*4 + 0]*a[2*4 + 2]);
    t3 = a[1*4 + 2]*(a[2*4 + 0]*a[3*4 + 1] - a[3*4 + 0]*a[2*4 + 1]);
    part4 = a[0*4 + 3]*(t1 - t2 + t3);
    
    return part1 - part2 + part3 - part4;
}

/* A - matrix n x m */
void transpose(const float *A, float *At, int m, int n) {
    int i, j;
    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++) {
            At[j*m + i] = A[i*n + j];
        }
    }
}

void mat_add(const float *A, const float *B, float *C, int m, int n) {
    int i, j;
    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++) {
            C[i*n + j] = A[i*n + j] + B[i*n + j];
        }
    }
}

void mat_sub(const float *A, const float *B, float *C, int m, int n) {
    int i, j;
    for (i = 0; i < m; i++) {
        for (j = 0; j < n; j++) {
            C[i*n + j] = A[i*n + j] - B[i*n + j];
        }
    }
}

/* A - matrix n x k 
   B - matrix k x m */
void mat_mul(const float *A, const float *B, float *C, int m, int n, int k) {
    int i, j, h;
    for (i = 0; i < m; i++) {
        for (j = 0; j < k; j++) {
            C[i*k + j] = 0;
            for (h = 0; h < n; h++) {
                C[i*k + j] += A[i*n + h] * B[h*k + j];;  
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
            }
        }
    }

    A_3x3 = (float *)malloc(m*m*sizeof(float));
    mat_mul(Ft, F, A_3x3, m, m, n);
    
    det = det3(A_3x3);
    if (det == 0) {
        free(Ft);
        free(A_3x3);
        return;
    }
    
    b = (float *)malloc(m*sizeof(float));
    mat_mul(Ft, B, b, m, 1, n);
    
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

float det2(const float *A) {
    return A[0*2 + 0]*A[1*2 + 1] - A[0*2 + 1]*A[1*2 + 0];
}

void mat2_inv(const float *A, float *Ainv) {
    float detA = det2(A);
    Ainv[0*2 + 0] = A[1*2 + 1] / detA;
    Ainv[0*2 + 1] = - A[0*2 + 1] / detA;
    Ainv[1*2 + 0] = - A[1*2 + 0] / detA;
    Ainv[1*2 + 1] = A[0*2 + 0] / detA;
}
