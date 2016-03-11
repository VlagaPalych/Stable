#include "extra_math.h"
#include "arm_math.h"

void Vect_CrossProd(float *x, float *y, float *res) {
    res[0] = x[1]*y[2] - y[1]*x[2];
    res[1] = - x[0]*y[2] + y[0]*x[2];
    res[2] = x[0]*y[1] - y[0]*x[1];
}

float Vect_Mod(float *x) {
    float tmp = 0;
    arm_sqrt_f32(x[0]*x[0] + x[1]*x[1] + x[2]*x[2], &tmp);
    return tmp;
}

void Vect_Norm(float *x) {
    float mod = Vect_Mod(x);
    arm_scale_f32(x, mod, x, VECT_SIZE);
}

float Mat_Tr(arm_matrix_instance_f32 *M) {
    uint32_t i = 0;
    float tr = 0;
    for (i = 0; i < M->numRows; i++) {
        tr += M->pData[M->numCols*i + i];
    }
    return tr;
}

void Mat_Adj3(arm_matrix_instance_f32 *M, arm_matrix_instance_f32 *A) {
    A->pData[0] = +(M->pData[1*3+1] * M->pData[2*3+2] - M->pData[1*3+2] * M->pData[2*3+1]);
    A->pData[1] = -(M->pData[0*3+1] * M->pData[2*3+2] - M->pData[0*3+2] * M->pData[2*3+1]);
    A->pData[2] = +(M->pData[0*3+1] * M->pData[1*3+2] - M->pData[0*3+2] * M->pData[1*3+1]);
    A->pData[3] = -(M->pData[1*3+0] * M->pData[2*3+2] - M->pData[1*3+2] * M->pData[2*3+0]);
    A->pData[4] = +(M->pData[0*3+0] * M->pData[2*3+2] - M->pData[0*3+2] * M->pData[2*3+0]);
    A->pData[5] = -(M->pData[0*3+0] * M->pData[1*3+2] - M->pData[0*3+2] * M->pData[1*3+0]);
    A->pData[6] = +(M->pData[1*3+0] * M->pData[2*3+1] - M->pData[1*3+1] * M->pData[2*3+0]);
    A->pData[7] = -(M->pData[0*3+0] * M->pData[2*3+1] - M->pData[0*3+1] * M->pData[2*3+0]);
    A->pData[8] = +(M->pData[0*3+0] * M->pData[1*3+1] - M->pData[0*3+1] * M->pData[1*3+0]);
}

float Mat_Det3(arm_matrix_instance_f32 *M) {
    float t1 = 0, t2 = 0, t3 = 0, part1 = 0, part2 = 0, part3 = 0, retval = 0;
    t1 = M->pData[1*3 + 1]*M->pData[2*3 + 2];
    t2 = M->pData[1*3 + 2]*M->pData[2*3 + 1];
    t3 = t1 - t2;
    part1 = M->pData[0*3 + 0]*t3;
    t1 = M->pData[1*3 + 0]*M->pData[2*3 + 2];
    t2 = M->pData[1*3 + 2]*M->pData[2*3 + 0];
    t3 = t1 - t2;
    part2 = M->pData[0*3 + 1]*t3;
    t1 = M->pData[1*3 + 0]*M->pData[2*3 + 1];
    t2 = M->pData[1*3 + 1]*M->pData[2*3 + 0];
    t3 = t1 - t2;
    part3 = M->pData[0*3 + 2]*t3;
    retval = part1 - part2 + part3;
    return retval;
}

void Quat_Scale(Quat *q, float scale_factor) {
    uint8_t i = 0;
    q->w *= scale_factor;
    for (i = 0; i < VECT_SIZE; i++) {
        q->v[i] *= scale_factor;
    }
}
void Quat_ToEuler(Quat q, float angle[3]) {
    uint8_t i = 0;
    angle[0] = atan(2*(q.w*q.v[0] + q.v[1]*q.v[2]) / (1.0f - 2*(q.v[0]*q.v[0] + q.v[1]*q.v[1])));
    angle[1] = asin(2*(q.w*q.v[1] + q.v[0]*q.v[2]));
    angle[2] = atan(2*(q.w*q.v[2] + q.v[0]*q.v[1]) / (1.0f - 2*(q.v[1]*q.v[1] + q.v[2]*q.v[2])));
    
    for (i = 0; i < 3; i++) {
        angle[i] *= 180.0 / 3.14159;
    }
}

float w1[VECT_SIZE], w2[VECT_SIZE];
float v1[VECT_SIZE], v2[VECT_SIZE];
float a1 = 0.5, a2 = 0.5;

float v1_t[VECT_SIZE];
float v2_t[VECT_SIZE];
float w1_t[VECT_SIZE];
float w2_t[VECT_SIZE];
float s_data[VECT_SIZE*VECT_SIZE];
float s_adj_data[VECT_SIZE*VECT_SIZE];
float i_data[VECT_SIZE*VECT_SIZE] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
float z_data[VECT_SIZE];
float x_data[VECT_SIZE];

float tmp1_data[VECT_SIZE*VECT_SIZE];
float tmp2_data[VECT_SIZE*VECT_SIZE];
float tmp3_data[VECT_SIZE*VECT_SIZE];
float tmp4_data[VECT_SIZE*VECT_SIZE];

arm_matrix_instance_f32 v1_mat;
arm_matrix_instance_f32 v2_mat;
arm_matrix_instance_f32 w1_mat;
arm_matrix_instance_f32 w2_mat;

arm_matrix_instance_f32 v1_t_mat;
arm_matrix_instance_f32 v2_t_mat;
arm_matrix_instance_f32 w1_t_mat;
arm_matrix_instance_f32 w2_t_mat;
arm_matrix_instance_f32 S;
arm_matrix_instance_f32 S_adj;

arm_matrix_instance_f32 tmp1;
arm_matrix_instance_f32 tmp2;
arm_matrix_instance_f32 tmp3;
arm_matrix_instance_f32 tmp4;

arm_matrix_instance_f32 I;
arm_matrix_instance_f32 Z;
arm_matrix_instance_f32 X;

extern Quat orientation;

void QUEST_Init() {
    // vector matrices
    arm_mat_init_f32(&v1_mat, VECT_SIZE, 1, v1);
    arm_mat_init_f32(&v2_mat, VECT_SIZE, 1, v2);
    arm_mat_init_f32(&w1_mat, VECT_SIZE, 1, w1);
    arm_mat_init_f32(&w2_mat, VECT_SIZE, 1, w2);
    
    // transposed matrices
    arm_mat_init_f32(&v1_t_mat, 1, VECT_SIZE, v1_t);
    arm_mat_init_f32(&v2_t_mat, 1, VECT_SIZE, v2_t);
    arm_mat_init_f32(&w1_t_mat, 1, VECT_SIZE, w1_t);
    arm_mat_init_f32(&w2_t_mat, 1, VECT_SIZE, w2_t);
    
    // temporal matrices
    arm_mat_init_f32(&tmp1, VECT_SIZE, VECT_SIZE, tmp1_data);
    arm_mat_init_f32(&tmp2, VECT_SIZE, VECT_SIZE, tmp2_data);
    arm_mat_init_f32(&tmp3, VECT_SIZE, VECT_SIZE, tmp3_data);
    arm_mat_init_f32(&tmp4, VECT_SIZE, VECT_SIZE, tmp4_data);
    
    arm_mat_init_f32(&S, VECT_SIZE, VECT_SIZE, s_data);
    arm_mat_init_f32(&S_adj, VECT_SIZE, VECT_SIZE, s_adj_data);
    arm_mat_init_f32(&I, VECT_SIZE, VECT_SIZE, i_data);
    arm_mat_init_f32(&Z, VECT_SIZE, 1, z_data);
    arm_mat_init_f32(&X, VECT_SIZE, 1, x_data);
}

void QUEST() {
    float v_dot_prod = 0, w_dot_prod = 0;
    float v_cross_prod[VECT_SIZE], w_cross_prod[VECT_SIZE];
    float cos_teta_cf = 0;
    float lambda_max = 0;
    float alpha = 0, beta = 0, gamma = 0;  
    float sigma = 0, kappa = 0, delta = 0;
    float norm_cf = 0;
    
    // calculation of lambda_max
    
    arm_dot_prod_f32(v1, v2, VECT_SIZE, &v_dot_prod);
    arm_dot_prod_f32(w1, w2, VECT_SIZE, &w_dot_prod);
    
    Vect_CrossProd(v1, v2, v_cross_prod);
    Vect_CrossProd(w1, w2, w_cross_prod);
    
    cos_teta_cf = v_dot_prod*w_dot_prod + Vect_Mod(v_cross_prod)*Vect_Mod(w_cross_prod);
    
    arm_sqrt_f32(a1*a1 + 2*a1*a2*cos_teta_cf + a2*a2, &lambda_max);
    
    // transposition
    arm_mat_trans_f32(&v1_mat, &v1_t_mat);
    arm_mat_trans_f32(&v2_mat, &v2_t_mat);
    arm_mat_trans_f32(&w1_mat, &w1_t_mat);
    arm_mat_trans_f32(&w2_mat, &w2_t_mat);
    
    // S calculation
    
    arm_mat_mult_f32(&w1_mat, &v1_t_mat, &tmp1);    // tmp1 = w1*v1^T
    arm_mat_mult_f32(&v1_mat, &w1_t_mat, &tmp2);    // tmp2 = v1*w1^T
    arm_mat_add_f32(&tmp1, &tmp2, &tmp3);           // tmp3 = tmp1 + tmp2
    arm_mat_scale_f32(&tmp3, a1, &tmp4);            // tmp4 = a1*tmp3
    
    arm_mat_mult_f32(&w2_mat, &v2_t_mat, &tmp1);    // tmp1 = w2*v2^T
    arm_mat_mult_f32(&v2_mat, &w2_t_mat, &tmp2);    // tmp2 = v2*w2^T
    arm_mat_add_f32(&tmp1, &tmp2, &tmp3);           // tmp3 = tmp1 + tmp2
    arm_mat_scale_f32(&tmp3, a1, &tmp1);            // tmp1 = a2*tmp3
    
    arm_mat_add_f32(&tmp4, &tmp1, &S);              // S = tmp4 + tmp1 = a1(w1*v1^T + v1*w1^T) + a2(w2*v2^2 + v2*w2^t)
    
    // Z calculation
    
    Vect_CrossProd(w1, v1, v_cross_prod);                       // v_cross_prod = w1*v1
    arm_scale_f32(v_cross_prod, a1, v_cross_prod, VECT_SIZE);   // v_cross_prod *= a1
    Vect_CrossProd(w2, v2, w_cross_prod);                       // w_cross_prod = w2*v2
    arm_scale_f32(w_cross_prod, a2, w_cross_prod, VECT_SIZE);   // w_cross_prod *= a2
    arm_add_f32(v_cross_prod, w_cross_prod, z_data, VECT_SIZE); // Z = v_cross_prod + w_cross_prod
    
    sigma = 0.5f * Mat_Tr(&S);
    
    Mat_Adj3(&S, &S_adj);
    kappa = Mat_Tr(&S_adj);
    
    delta = Mat_Det3(&S);
    
    alpha = lambda_max*lambda_max - sigma*sigma + kappa;
    beta = lambda_max - sigma;
    gamma = (lambda_max + sigma) * alpha - delta;
    
    // X calculation
    arm_mat_scale_f32(&I, alpha, &tmp1);            // tmp1 = alpha*I
    arm_mat_scale_f32(&S, beta, &tmp2);             // tmp2 = beta*S
    arm_mat_mult_f32(&S, &S, &tmp3);                // tmp3 = S^2
    arm_mat_add_f32(&tmp1, &tmp2, &tmp4);           // tmp4 = tmp1 + tmp2
    arm_mat_add_f32(&tmp4, &tmp3, &tmp1);           // tmp1 = tmp4 + tmp3 = alpha*I + beta*S + S^2
    arm_mat_mult_f32(&tmp1, &Z, &X);                // X = tmp1 * Z
    
    arm_copy_f32(X.pData, orientation.v, VECT_SIZE);
    orientation.w = gamma;
    
    arm_sqrt_f32(gamma*gamma + X.pData[0]*X.pData[0] + X.pData[1]*X.pData[1] + X.pData[2]*X.pData[2], &norm_cf);
    Quat_Scale(&orientation, 1/norm_cf);
}


#define DELTA 0.01

float E_data[KALMAN_STATE_SIZE*KALMAN_STATE_SIZE] = {
    1, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 1
};
arm_matrix_instance_f32 E = {KALMAN_STATE_SIZE, KALMAN_STATE_SIZE, E_data};

float Fk_data[KALMAN_STATE_SIZE*KALMAN_STATE_SIZE] = {
    1, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0,
    0.5*DELTA, 0, 0, 0, 1, 0, 0,
    0, 0.5*DELTA, 0, 0, 0, 1, 0,
    0, 0, 0.5*DELTA, 0, 0, 0, 1
};
arm_matrix_instance_f32 Fk = {KALMAN_STATE_SIZE, KALMAN_STATE_SIZE, Fk_data};

float Fkt_data[KALMAN_STATE_SIZE*KALMAN_STATE_SIZE] = {
    1, 0, 0, 0, 0.5*DELTA, 0, 0,
    0, 1, 0, 0, 0, 0.5*DELTA, 0,
    0, 0, 1, 0, 0, 0, 0.5*DELTA,
    0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 1
};
arm_matrix_instance_f32 Fkt = {KALMAN_STATE_SIZE, KALMAN_STATE_SIZE, Fkt_data};

float Hk_data[KALMAN_STATE_SIZE*KALMAN_STATE_SIZE] = {
    1, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 1
};
arm_matrix_instance_f32 Hk = {KALMAN_STATE_SIZE, KALMAN_STATE_SIZE, Hk_data};

// 3 sigma of max change of angle rate around axis
#define q11 0.1
#define q22 0.1 
#define q33 0.1
float Qk_data[KALMAN_STATE_SIZE*KALMAN_STATE_SIZE] = {
    q11, 0, 0, 0, 0, 0, 0,
    0, q22, 0, 0, 0, 0, 0,
    0, 0, q33, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0
};
arm_matrix_instance_f32 Qk = {KALMAN_STATE_SIZE, KALMAN_STATE_SIZE, Qk_data};

float Rk_data[KALMAN_STATE_SIZE*KALMAN_STATE_SIZE] = {
    1e-2, 0, 0, 0, 0, 0, 0,
    0, 1e-2, 0, 0, 0, 0, 0,
    0, 0, 1e-2, 0, 0, 0, 0,
    0, 0, 0, 1e-4, 0, 0, 0,
    0, 0, 0, 0, 1e-4, 0, 0,
    0, 0, 0, 0, 0, 1e-4, 0,
    0, 0, 0, 0, 0, 0, 1e-4
};
arm_matrix_instance_f32 Rk = {KALMAN_STATE_SIZE, KALMAN_STATE_SIZE, Rk_data};

float x_apriori_data[KALMAN_STATE_SIZE] = {0, 0, 0, 1, 0, 0, 0};
arm_matrix_instance_f32 x_apriori = {KALMAN_STATE_SIZE, 1, x_apriori_data};
float x_aposteriori_data[KALMAN_STATE_SIZE];
arm_matrix_instance_f32 x_aposteriori = {KALMAN_STATE_SIZE, 1, x_aposteriori_data};

float P_apriori_data[KALMAN_STATE_SIZE*KALMAN_STATE_SIZE] = {
    0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0
};
arm_matrix_instance_f32 P_apriori = {KALMAN_STATE_SIZE, KALMAN_STATE_SIZE, P_apriori_data};
float P_aposteriori_data[KALMAN_STATE_SIZE*KALMAN_STATE_SIZE];
arm_matrix_instance_f32 P_aposteriori = {KALMAN_STATE_SIZE, KALMAN_STATE_SIZE, P_aposteriori_data};

float zk_data[KALMAN_STATE_SIZE];
arm_matrix_instance_f32 zk = {KALMAN_STATE_SIZE, 1, zk_data};

float Kk_data[KALMAN_STATE_SIZE*KALMAN_STATE_SIZE];
arm_matrix_instance_f32 Kk = {KALMAN_STATE_SIZE, KALMAN_STATE_SIZE, Kk_data};

float tmp5_data[KALMAN_STATE_SIZE*KALMAN_STATE_SIZE];
arm_matrix_instance_f32 tmp5 = {KALMAN_STATE_SIZE, KALMAN_STATE_SIZE, tmp5_data};

float tmp6_data[KALMAN_STATE_SIZE];
arm_matrix_instance_f32 tmp6 = {KALMAN_STATE_SIZE, 1, tmp6_data};

float tmp7_data[KALMAN_STATE_SIZE];
arm_matrix_instance_f32 tmp7 = {KALMAN_STATE_SIZE, 1, tmp7_data};

float tmp8_data[KALMAN_STATE_SIZE*KALMAN_STATE_SIZE];
arm_matrix_instance_f32 tmp8 = {KALMAN_STATE_SIZE, KALMAN_STATE_SIZE, tmp8_data};

void f(float *in, float *out) {
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;
    out[3] = -0.5*(in[0]*in[4] + in[1]*in[5] + in[2]*in[6]);
    out[4] = 0.5*(in[0]*in[3] + in[2]*in[5] - in[1]*in[6]);
    out[5] = 0.5*(in[1]*in[3] + in[0]*in[6] - in[2]*in[4]);
    out[6] = 0.5*(in[2]*in[3] + in[1]*in[4] - in[0]*in[5]);
}


void Kalman() {
    // Kalman gain
    arm_mat_add_f32(&P_apriori, &Rk, &tmp5);
    arm_mat_inverse_f32(&tmp5, &tmp8);
    arm_mat_mult_f32(&P_apriori, &tmp8, &Kk);
    
    // x_aposteriori
    arm_sub_f32(zk_data, x_apriori_data, tmp6_data, KALMAN_STATE_SIZE);
    arm_mat_mult_f32(&Kk, &tmp6, &tmp7);
    arm_add_f32(x_apriori_data, tmp7_data, x_aposteriori_data, KALMAN_STATE_SIZE); 
    
    // P_aposteriori
    arm_mat_sub_f32(&E, &Kk, &tmp5);
    arm_mat_mult_f32(&tmp5, &P_apriori, &P_aposteriori);
    
    // x_apriori
    f(x_aposteriori_data, tmp6_data);
    arm_scale_f32(tmp6_data, DELTA, tmp6_data, KALMAN_STATE_SIZE);
    arm_add_f32(x_aposteriori_data, tmp6_data, x_apriori_data, KALMAN_STATE_SIZE);
    
    // P_apriori
    arm_mat_mult_f32(&Fk, &P_aposteriori, &tmp5);
    arm_mat_mult_f32(&tmp5, &Fkt, &P_apriori);
    arm_mat_add_f32(&P_apriori, &Qk, &P_apriori);
}
