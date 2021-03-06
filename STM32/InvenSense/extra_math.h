#ifndef EXTRA_MATH_H
#define EXTRA_MATH_H

#define VECT_SIZE 3

typedef struct {
    float v[3];
    float w;
} Quat;

void Quat_ToEuler(Quat q, float angle[3]);

float Vect_Mod(float *x) ;
void Vect_Norm(float *x);
    
void QUEST_Init(void);
void QUEST(void);

#define KALMAN_STATE_SIZE 7
extern float zk_data[KALMAN_STATE_SIZE];
extern float x_aposteriori_data[KALMAN_STATE_SIZE];

void Kalman(void);

void angleRate_to_eulerRate(float *angleRate, float *euler, float *eulerRate);
void radians_to_degrees(float *radians);
void degrees_to_radians(float *degrees);

#endif
