#ifndef EXTRA_MATH_H
#define EXTRA_MATH_H

#define VECT_SIZE 3

typedef struct {
    float v[3];
    float w;
} Quat;

void Quat_ToEuler(Quat q, float angle[3]);

void Vect_Norm(float *x);
    
void QUEST_Init(void);
void QUEST(void);

#endif
