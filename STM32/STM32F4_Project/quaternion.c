#include "quaternion.h"
#include "stdint.h"
#include "math.h"

void Vect3_Add(float *x, float *y, float *res) {
    uint8_t i = 0;
    for (i = 0; i < 3; i++) {
        res[i] = x[i] + y[i];
    }
}

void Vect3_Mul(float *v, float a, float *res) {
    uint8_t i = 0;
    for (i = 0; i < 3; i++) {
        res[i] = v[i] * a;
    }
}

float Vect3_ScalarMul(float *x, float *y) {
    uint8_t i = 0;
    float tmp = 0;
    for (i = 0; i < 3; i++) {
        tmp += x[i] * y[i];
    }
    return tmp;
}

void Vect3_VectorMul(float *x, float *y, float *res) {
    res[0] = x[1]*y[2] - y[1]*x[2];
    res[1] = - x[0]*y[2] + y[0]*x[2];
    res[2] = x[0]*y[1] - y[0]*x[1];
}

void Quat_Mul(Quat p, Quat q, Quat *res) {
    float tmp1[3], tmp2[3];
    res->w = p.w * q.w - Vect3_ScalarMul(p.v, q.v);     // ww' - (v,v')
    Vect3_VectorMul(p.v, q.v, tmp1);                    // tmp1 = [v, v']
    Vect3_Mul(q.v, p.w, tmp2);                          // tmp2 = wv'
    Vect3_Mul(p.v, q.w, res->v);                        // res->v = w'v
    Vect3_Add(res->v, tmp1, res->v);                    // res->v += tmp1
    Vect3_Add(res->v, tmp2, res->v);                    // res->v += tmp2
}

float Vect3_Mod(float *v) {
    uint8_t i = 0;
    float tmp = 0;
    for (i = 0; i < 3; i++) {
        tmp += v[i] * v[i];
    }
    return sqrt(tmp);
}

void Quat_FromAngleRate(float angleRate[3], Quat *res) {
    float axis[3], phi = 0, sin_phi, cos_phi;
    
    Vect3_Mul(angleRate, 0.01, axis);       // get angles of 0.01 sec rotation
    Vect3_Mul(axis, 3.14159 / 180.0, axis); // to radians
    phi = Vect3_Mod(axis);                  // get angle of rotation around one exact axis
    Vect3_Mul(axis, 1.0 / phi, axis);       // normalize axis
    sin_phi = sin(phi / 2.0);
    cos_phi = cos(phi / 2.0);
    Vect3_Mul(axis, sin_phi, res->v);       // fill quaternion fields
    res->w = cos_phi;
}

void Quat_ToEuler(Quat q, float angle[3]) {
    uint8_t i = 0;
    angle[0] = atan(2*(q.w*q.v[0] + q.v[1]*q.v[2]) / (1.0 - 2*(q.v[0]*q.v[0] + q.v[1]*q.v[1])));
    angle[1] = asin(2*(q.w*q.v[1] + q.v[0]*q.v[2]));
    angle[2] = atan(2*(q.w*q.v[2] + q.v[0]*q.v[1]) / (1.0 - 2*(q.v[1]*q.v[1] + q.v[2]*q.v[2])));
    
    for (i = 0; i < 3; i++) {
        angle[i] *= 180.0 / 3.14159;
    }
}