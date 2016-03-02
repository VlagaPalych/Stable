#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"

#define ALPHA 1.0 // reflection coefficient
#define GAMMA 2.0 // expansion coefficient
#define RHO 0.5   // contraction coefficient

void nelder_mead(float (*f)(float *, uint8_t), uint8_t n, float *x_init, float *step, float e, float *x_min) {
    uint8_t i = 0, j = 0;
    float **x, *y, *x_bar, *x_r, y_r, *x_e, *x_c;
    float aver = 0; // average function value
    float crit = 0; // stop criteria variable
    uint8_t l, h, sh;
    
    // memory allocation
    // simplex
    x = (float **)malloc((n+1)*sizeof(float *));
    for (i = 0; i < n + 1; i++) {
        x[i] = (float *)malloc(n*sizeof(float));
    }    
    // function values in vertices
    y = (float *)malloc((n+1)*sizeof(float));
    // centroid
    x_bar = (float *)malloc(n*sizeof(float));
    // reflection point
    x_r = (float *)malloc(n*sizeof(float));
    // expansion point
    x_e = (float *)malloc(n*sizeof(float));
    // contraction point
    x_c = (float *)malloc(n*sizeof(float));
    
    
    // axis-by-axis initialization
    memcpy(x[0], x_init, n*sizeof(float));
    y[0] = f(x[0], n);
    for (i = 1; i < n + 1; i++) {
        for (j = 0; j < n; j++) {
            if (j == i - 1) {
                x[i][j] = x_init[j] + step[j];
            } else {
                x[i][j] = x_init[j];
            }
        }
        y[i] = f(x[i], n);
    }
    
    // main cycle
    while (1) {
        y[h] = f(x[h], n); // update value at changed vertex
        
        // stop criteria
        aver = 0;
        for (i = 0; i < n + 1; i++) {
            aver += y[i];
        }
        aver /= n + 1;
        crit = 0;
        for (i = 0; i < n + 1; i++) {
            crit += (y[i] - aver)*(y[i] - aver);
        }
        crit = sqrt(crit) / (n + 1);
        if (crit < e) {
            break;
        }
        
        // lowest, highest and second highest vertices
        // TODO: track these vertices automatically
        l = 0;
        h = 0;
        sh = 0;
        for (i = 1; i < n + 1; i++) {
            if (y[i] < y[l]) {
                l = i;
            } else if (y[i] > y[h]) {
                h = i;
            }
        }
        for (i = 1; i < n + 1; i++) {
            if ((i != h) && (y[i] > y[sh])) {
                sh = i;
            }
        }
        
        // centroid
        memset(x_bar, 0, n*sizeof(float));
        for (i = 0; i < n + 1; i++) {
            for (j = 0; j < n; j++) {
                x_bar[j] += x[i][j]; 
            }
        }
        for (j = 0; j < n; j++) {
            x_bar[j] -= x[h][j];    // except the highest vertex
            x_bar[j] /= n;          // averaging
        }
        
        // reflection point
        for (j = 0; j < n; j++) {
            x_r[j] = (1 + ALPHA)*x_bar[j] - ALPHA*x[h][j];
        }
        y_r = f(x_r, n);
        
        if (y_r < y[l]) {
            // expansion point
            for (j = 0; j < n; j++) {
                x_e[j] = (1 + GAMMA)*x_r[j] - GAMMA*x_bar[j];
            }
            if (f(x_e, n) < y[l]) {
                memcpy(x[h], x_e, n*sizeof(float)); // expansion
            } else {
                memcpy(x[h], x_r, n*sizeof(float)); // reflection
            }
        } else if (y_r > y[sh]) {
            if (y_r <= y[h]) {
                memcpy(x[h], x_r, n*sizeof(float)); // reflection
                y[h] = y_r;
            }
            
            // contraction point
            for (j = 0; j < n; j++) {
                x_c[j] = RHO*x[h][j] + (1 - RHO)*x_bar[j];
            }
            if (f(x_c, n) > y[h]) {
                for (i = 0; i < n + 1; i++) {   // mutiple contraction
                    if (i != l) {               // except the highest vertex
                        for (j = 0; j < n; j++) {
                            x[i][j] = (x[i][j] + x[l][j]) / 2;
                        }
                    }
                }
            } else {
                memcpy(x[h], x_c, n*sizeof(float)); // contraction
            }
        } else {
            memcpy(x[h], x_r, n*sizeof(float)); // reflection
        }
    }
    
    // result copying
    memcpy(x_min, x[l], n*sizeof(float));
    
    // memory deallocation
    // simplex
    for (i = 0; i < n + 1; i++) {
        free(x[i]);
    }   
    free(x);  
    // function values in vertices
    free(y);
    // centroid
    free(x_bar);
    // reflection point
    free(x_r);
    // expansion point
    free(x_e);
    // contraction point
    free(x_c);
}