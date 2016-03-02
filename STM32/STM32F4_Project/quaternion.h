#ifndef QUATERNION_H
#define QUATERNION_H

typedef struct {
    float v[3];
    float w;
} Quat;

void Quat_Mul(Quat q1, Quat q2, Quat *res);
void Quat_FromAngleRate(float angleRate[3], Quat *res);
void Quat_ToEuler(Quat q, float angle[3]);

void Vect3_Add(float *v1, float *v2, float *res);
void Vect3_Mul(float *v, float a, float *res);
float Vect3_ScalarMul(float *v1, float *v2);
void Vect3_VectorMul(float *v1, float *v2, float *res);
float Vect3_Mod(float *v);

#endif
