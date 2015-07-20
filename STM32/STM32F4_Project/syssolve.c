#include "syssolve.h"
#include "string.h"
#include "stdlib.h"

double y[3];                // 3 last angles
double u[3];                // 3 last thrusts

double w[3];                // unknown coeffs in diff eq
double Afull[10*3];         // matrix of our equation Afull * w = Bfull
double Bfull[10];           // right column

uint8_t anglesAccumulated;  // number of angles we have at the moment
uint8_t row;

double det3(double *mat)
{
    return mat[0*3 + 0]*(mat[1*3 + 1]*mat[2*3 + 2] - mat[1*3 + 2]*mat[2*3 + 1]) - mat[0*3 + 1]*(mat[1*3 + 0]*mat[2*3 + 2] - mat[1*3 + 2]*mat[2*3 + 0]) +
    mat[0*3 + 2]*(mat[1*3 + 0]*mat[2*3 + 1] - mat[1*3 + 1]*mat[2*3 + 0]);
}

/* A - matrix n x m */
void transpose(double *A, double *At, int n, int m) {
    int i, j;
    for (i = 0; i < n; i++) {
        for (j = 0; j < m; j++) {
            At[j*n + i] = A[i*m + j];
        }
    }
}

void mat_add(const double *A, const double *B, double *C, int n, int m) {
    int i, j;
    for (i = 0; i < n; i++) {
        for (j = 0; j < m; j++) {
            C[i*m + j] = A[i*m + j] + B[i*m + j];
        }
    }
}

void mat_sub(const double *A, const double *B, double *C, int n, int m) {
    int i, j;
    for (i = 0; i < n; i++) {
        for (j = 0; j < m; j++) {
            C[i*m + j] = A[i*m + j] - B[i*m + j];
        }
    }
}

/* A - matrix n x k 
   B - matrix k x m */
void mat_mul(const double *A, const double *B, double *C, int n, int m, int k) {
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

void system_solve(double *F, double *B, double *x, int n, int m) {
    double *Ft;
    double *A;
    double *b;
    double det, detI;
    double *tmp;
    int i, j;

    Ft = (double *)malloc(n*m*sizeof(double));
    transpose(F, Ft, n, m);

    A = (double *)malloc(m*m*sizeof(double));
    b = (double *)malloc(m*sizeof(double));

    mat_mul(Ft, F, A, m, m, n);
    mat_mul(Ft, B, b, m, 1, n);

    det = det3(A);

    tmp = (double *)malloc(m*m*sizeof(double));

    for (i = 0; i < m; i++) {
        memcpy(tmp, A, m*m*sizeof(double));
        for (j = 0; j < m; j++) {
            tmp[j*m + i] = b[j];
        }
         
        detI = det3(tmp);
        x[i] = detI / det;
    }

    free(Ft);
    free(A);
    free(b);
    free(tmp);
}
