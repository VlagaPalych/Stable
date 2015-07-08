#include <stdio.h>
#include <stdlib.h>
#include <string.h>

double res[3];

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

/* A - matrix n x k 
   B - matrix k x m */
void mat_mul(double *A, double *B, double *C, int n, int m, int k) {
    int i, j, h;
    for (i = 0; i < n; i++) {
        for (j = 0; j < m; j++) {
            C[i*m + j] = 0;
            for (h = 0; h < k; h++) {
                //printf("C[%d][%d] = %.2lf, A[%d][%d] = %.2lf, B[%d][%d] = %.2lf\n", i, j, C[i*m + j], i, h, A[i*k + h], h, j, B[h*m + j]);
                C[i*m + j] += A[i*k + h] * B[h*m + j];  
            }
        }
    }
}

void system_solve(double *F, double *B, double *x, int n, int m)
{
    double *Ft;
    double *A;
    double *b;
    double det, detI;
    double *tmp;
    int i, j, y, z;

    Ft = (double *)malloc(n*m*sizeof(double));
    transpose(F, Ft, n, m);

    A = (double *)malloc(m*m*sizeof(double));
    b = (double *)malloc(m*sizeof(double));

    mat_mul(Ft, F, A, m, m, n);
    mat_mul(Ft, B, b, m, 1, n);

    printf("A\n");
    for (i = 0; i < m; i++) {
        for (j = 0; j < m; j++) {
            printf("%.2lf ", A[i*m + j]);
        }
        printf("\n");
    }

    printf("b\n");
    for (j = 0; j < m; j++) {
        printf("%.2lf\n", b[j]);
    }

    det = det3(A);
    printf("DET = %.lf\n", det);

    tmp = (double *)malloc(m*m*sizeof(double));

    for (i = 0; i < m; i++) {
        memcpy(tmp, A, m*m*sizeof(double));
        for (j = 0; j < m; j++) {
            tmp[j*m + i] = b[j];
        }

        printf("tmp\n");
        for (z = 0; z < m; z++) {
            for (y = 0; y < m; y++) {
                printf("%.2lf ", tmp[z*m + y]);
            }
            printf("\n");
        }      
        detI = det3(tmp);
        x[i] = detI / det;
        printf("TMP_DET = %.lf\n", detI);   
    }

    free(Ft);
    free(A);
    free(b);
    free(tmp);
}

int main(int argc, char *argv[])
{
    double F[] = {1, 0, 3, 
                0, 5, 6,
                7, 8, 9,
                10, 11, 0};
    double B[] = {5, 10, 15, 20};
    double x[3];

    int n = 4;
    int m = 3;
    int i;

    system_solve(F, B, x, n, m);
    for (i = 0; i < m; i++) {
        printf("%.2lf ", x[i]);
    }

    return 0;
}
