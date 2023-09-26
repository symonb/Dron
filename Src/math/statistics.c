#include <math.h>
#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include "arm_math.h"

#include "statistics.h"



void dev_clear(stdev_t* dev)
{
    dev->m_n = 0;
}

void dev_push(stdev_t* dev, float x)
{
    dev->m_n++;
    if (dev->m_n == 1) {
        dev->m_oldM = dev->m_newM = x;
        dev->m_oldS = 0.0f;
    }
    else {
        dev->m_newM = dev->m_oldM + (x - dev->m_oldM) / dev->m_n;
        dev->m_newS = dev->m_oldS + (x - dev->m_oldM) * (x - dev->m_newM);
        dev->m_oldM = dev->m_newM;
        dev->m_oldS = dev->m_newS;
    }
}

float dev_variance(stdev_t* dev)
{
    return ((dev->m_n > 1) ? dev->m_newS / (dev->m_n - 1) : 0.0f);
}

float dev_standard_deviation(stdev_t* dev)
{
    return sqrtf(dev_variance(dev));
}

void newton_gauss_method(float sample[][3], uint16_t N) {

    // simplest version with N == N_B
    const uint8_t N_B = 6;
    float beta[N_B];
    float beta_change[N_B];
    float residuals[N];
    float32_t J_values[N_B * N];
    float32_t J_inv_values[N_B * N];
    arm_matrix_instance_f32 J;
    arm_matrix_instance_f32 J_inv;
    float eps = 0.000001f;
    uint8_t counter = 0;
    uint8_t max_iterations = 21;
    bool stop = false;

    //  1) take the best guess of betas:
    float max_value[3] = { 0,0,0 };
    float min_value[3] = { 0,0,0 };
    for (uint16_t i = 0;i < N;i++) {
        for (uint8_t j = 0;j < 3;j++) {
            if (sample[i][j] > max_value[j]) {
                max_value[j] = sample[i][j];
            }
            else if (sample[i][j] < min_value[j]) {
                min_value[j] = sample[i][j];
            }
        }
    }
    for (uint8_t i = 0; i < 3; i++)
    {
        beta[i] = 2.f / (max_value[i] - min_value[i]);
        beta[i + 3] = (float)(max_value[i] + min_value[i]) / 2.f;
    }
    while (!stop && (counter < max_iterations)) {
        //  2) compute residual values:
        for (uint8_t j = 0; j < N; j++) {
            residuals[j] = -1 +
                powf(beta[0], 2) * powf(sample[j][0] - beta[3], 2) +
                powf(beta[1], 2) * powf(sample[j][1] - beta[4], 2) +
                powf(beta[2], 2) * powf(sample[j][2] - beta[5], 2);
        }

        //  3) Compute Jacobian (Jacobian inverse actually) values:
        for (uint8_t i = 0; i < N; i++) {
            for (uint8_t j = 0; j < 3; j++) {
                J_values[i * N_B + j] = 2 * beta[j] * powf(sample[i][j] - beta[j + 3], 2);
                J_values[i * N_B + j + 3] = -2 * powf(beta[j], 2) * (sample[i][j] - beta[j + 3]);
            }
        }

        arm_mat_init_f32(&J, N_B, N, J_values);
        arm_mat_init_f32(&J_inv, N_B, N, J_inv_values);
        arm_mat_inverse_f32(&J, &J_inv);

        //  4) Compute change of betas:
        for (uint8_t i = 0;i < N_B;i++) {
            beta_change[i] = 0;
            for (uint8_t j = 0; j < N; j++) {
                beta_change[i] += -J_inv.pData[i * N + j] * residuals[j];
            }
        }

        //  5) Update betas and stop or back to 2)
        stop = true;
        for (int i = 0;i < N_B;i++) {
            beta[i] += beta_change[i];
            if (fabs(beta_change[i]) > eps) {
                stop = false;
            }
        }
        counter++;
    }
    if (stop) {
        for (uint8_t i = 0;i < 3;++i) {
            acc_1.scale[i] = beta[i];
            acc_1.offset[i] = beta[i + 3];
        }
    }
}