#include <math.h>
#include <stdint.h>
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include "math/quaternions.h"
#include "filters.h"
#include "sensors/MS5XXX.h"
#define CONST_GRAVITY 9.80665f          //  [m/s^2]
#define CONST_MOLAR_GAS 8.31446f        //  [J/(mol*K)]
#define CONST_MOLAR_MASS_AIR 0.02896f   //  [kg/mol]
#define CONST_TEMPERATURE 293           //  [K]   assumed to be constant during a flight

static float acc_z = 1;

void baro_calculate_altitude(baro_t* baro, timeUs_t current_time) {
    // filter new data

    static timeUs_t last_time;
    float dt = US_TO_SEC(current_time - last_time);
    last_time = current_time;
    baro_preasure_filtering();
    float current_alt = 0.001f * (baro->h0_preasure - baro->filtered_preasure) * CONST_MOLAR_GAS / (CONST_GRAVITY * CONST_MOLAR_MASS_AIR) * CONST_TEMPERATURE;

    // not used for now (see kalman filter):
    baro->vel_raw = (current_alt - baro->altitude) / dt;

    // update altitiude:
    baro->altitude = current_alt;

}

void baro_kalman_fusion(baro_t* baro, timeUs_t current_time) {

    // simplest version of KF (without updating velocity by measuremnts): 
    static timeUs_t last_time;
    const float dt = US_TO_SEC(current_time - last_time);
    last_time = current_time;
    const float A[2][2] = { {1, dt },{ 0, 1} };
    const float B[2] = { dt * dt / 2, dt };
    const float C[2] = { 1, 0 };
    const float acc_dev = 0.01f;
    const float R[2][2] = { {acc_dev * B[0] * B[0], acc_dev * B[0] * B[1]},{acc_dev * B[1] * B[0], acc_dev * B[1] * B[1]} };
    const float Q = 0.001f;
    // covariance
    static float P[2][2];
    threef_t acc_vec = { Gyro_Acc[3], Gyro_Acc[4], Gyro_Acc[5] };

    // transform acc values into global frame and add filtering:
    float u = CONST_GRAVITY * (quaternion_rotate_vector(acc_vec, quaternion_conjugate(q_global_attitude)).yaw - 1);
    // state:
    static float x[2];
    // tate estimation:
    float x_e[2] = { A[0][0] * x[0] + A[0][1] * x[1] + B[0] * u , A[1][0] * x[0] + A[1][1] * x[1] + B[1] * u };

    float P_temp[2][2];
    P_temp[0][0] = A[0][0] * (P[0][0] * A[0][0] + P[0][1] * A[0][1]) + A[0][1] * (P[1][0] * A[0][0] + P[1][1] * A[0][1]) + R[0][0];
    P_temp[0][1] = A[0][0] * (P[0][0] * A[1][0] + P[0][1] * A[1][1]) + A[0][1] * (P[1][0] * A[1][0] + P[1][1] * A[1][1]) + R[0][1];
    P_temp[1][0] = A[1][0] * (P[0][0] * A[0][0] + P[0][1] * A[0][1]) + A[1][1] * (P[1][0] * A[0][0] + P[1][1] * A[0][1]) + R[1][0];
    P_temp[1][1] = A[1][0] * (P[0][0] * A[1][0] + P[0][1] * A[1][1]) + A[1][1] * (P[1][0] * A[1][0] + P[1][1] * A[1][1]) + R[1][1];

    float K[2];
    // calculate kalman gains:
    float L = C[0] * (C[0] * P_temp[0][0] + C[1] * P_temp[0][1]) + C[1] * (C[0] * P_temp[1][0] + C[1] * P_temp[1][1]) + Q;
    K[0] = (P_temp[0][0] * C[0] + P_temp[0][1] * C[1]) / L;
    K[1] = (P_temp[1][0] * C[0] + P_temp[1][1] * C[1]) / L;

    // update state:
    float g = (baro->altitude - (C[0] * x_e[0] + C[1] * x_e[1]));
    x[0] = x_e[0] + K[0] * g;
    x[1] = x_e[1] + K[1] * g;
    baro->altitude = x[0];
    baro->ver_vel = x[1];

    // update covariance matrix:
    float temp[2][2];
    temp[0][0] = 1 - K[0] * C[0];
    temp[0][1] = -K[0] * C[1];
    temp[1][0] = -K[1] * C[0];
    temp[1][1] = 1 - K[1] * C[1];

    P[0][0] = temp[0][0] * P_temp[0][0] + temp[0][1] * P_temp[1][0];
    P[0][1] = temp[0][0] * P_temp[0][1] + temp[0][1] * P_temp[1][1];
    P[1][0] = temp[1][0] * P_temp[0][0] + temp[1][1] * P_temp[1][0];
    P[1][1] = temp[1][0] * P_temp[0][1] + temp[1][1] * P_temp[1][1];
}

void baro_kalman_fusion_v2(baro_t* baro, timeUs_t current_time) {
    // version of KF with velocity updated from barometer: 
    static timeUs_t last_time;
    const float dt = US_TO_SEC(current_time - last_time);
    last_time = current_time;
    const float A[2][2] = { {1, dt },{ 0, 1} };
    const float B[2] = { dt * dt / 2, dt };
    const float C[2][2] = { {1, 0},{0,1} };
    const float acc_dev = 0.01f;
    const float R[2][2] = { {acc_dev * B[0] * B[0], acc_dev * B[0] * B[1]}, {acc_dev * B[1] * B[0], acc_dev * B[1] * B[1]} };
    const float baro_dev = 0.01f;
    const float Q[2][2] = { {baro_dev, 0} ,{0, baro_dev / dt} };
    // covariance
    static float P[2][2];
    threef_t acc_vec = { Gyro_Acc[3], Gyro_Acc[4], Gyro_Acc[5] };

    // transform acc values into global frame and add filtering:
    float u = CONST_GRAVITY * (quaternion_rotate_vector(acc_vec, quaternion_conjugate(q_global_attitude)).yaw - 1);
    // state:
    static float x[2];
    // tate estimation:
    float x_e[2] = { A[0][0] * x[0] + A[0][1] * x[1] + B[0] * u , A[1][0] * x[0] + A[1][1] * x[1] + B[1] * u };

    float P_temp[2][2];
    P_temp[0][0] = A[0][0] * (P[0][0] * A[0][0] + P[0][1] * A[0][1]) + A[0][1] * (P[1][0] * A[0][0] + P[1][1] * A[0][1]) + R[0][0];
    P_temp[0][1] = A[0][0] * (P[0][0] * A[1][0] + P[0][1] * A[1][1]) + A[0][1] * (P[1][0] * A[1][0] + P[1][1] * A[1][1]) + R[0][1];
    P_temp[1][0] = A[1][0] * (P[0][0] * A[0][0] + P[0][1] * A[0][1]) + A[1][1] * (P[1][0] * A[0][0] + P[1][1] * A[0][1]) + R[1][0];
    P_temp[1][1] = A[1][0] * (P[0][0] * A[1][0] + P[0][1] * A[1][1]) + A[1][1] * (P[1][0] * A[1][0] + P[1][1] * A[1][1]) + R[1][1];

    float K[2][2];
    // calculate kalman gains:
    float L[2][2];
    float L_inv[2][2];
    L[0][0] = C[0][0] * (C[0][0] * P_temp[0][0] + C[0][1] * P_temp[0][1]) + C[0][1] * (C[0][0] * P_temp[1][0] + C[0][1] * P_temp[1][1]) + Q[0][0];
    L[0][1] = C[0][0] * (C[1][0] * P_temp[0][0] + C[1][1] * P_temp[0][1]) + C[0][1] * (C[1][0] * P_temp[1][0] + C[1][1] * P_temp[1][1]) + Q[0][1];
    L[1][0] = C[1][0] * (C[0][0] * P_temp[0][0] + C[0][1] * P_temp[0][1]) + C[1][1] * (C[0][0] * P_temp[1][0] + C[0][1] * P_temp[1][1]) + Q[1][0];
    L[1][1] = C[1][0] * (C[1][0] * P_temp[0][0] + C[1][1] * P_temp[0][1]) + C[1][1] * (C[1][0] * P_temp[1][0] + C[1][1] * P_temp[1][1]) + Q[1][1];
    //inverse of L:
    float det = L[0][0] * L[1][1] - L[0][1] * L[1][0];
    if (det != 0.0f) {
        L_inv[0][0] = L[1][1] / det;
        L_inv[0][1] = -L[1][0] / det;
        L_inv[1][0] = -L[0][1] / det;
        L_inv[1][1] = L[0][0] / det;
    }
    else {
        L_inv[0][0] = 0;
        L_inv[0][1] = 0;
        L_inv[1][0] = 0;
        L_inv[1][1] = 0;
    }

    K[0][0] = L_inv[0][0] * (C[0][0] * P_temp[0][0] + C[0][1] * P_temp[0][1]) + L_inv[1][0] * (C[1][0] * P_temp[0][0] + C[1][1] * P_temp[0][1]);
    K[0][1] = L_inv[0][1] * (C[0][0] * P_temp[0][0] + C[0][1] * P_temp[0][1]) + L_inv[1][1] * (C[1][0] * P_temp[0][0] + C[1][1] * P_temp[0][1]);
    K[1][0] = L_inv[0][0] * (C[0][0] * P_temp[1][0] + C[0][1] * P_temp[1][1]) + L_inv[1][0] * (C[1][0] * P_temp[1][0] + C[1][1] * P_temp[1][1]);
    K[1][1] = L_inv[0][1] * (C[0][0] * P_temp[1][0] + C[0][1] * P_temp[1][1]) + L_inv[1][1] * (C[1][0] * P_temp[1][0] + C[1][1] * P_temp[1][1]);

    // update state:
    float inov[2] = { baro->altitude - (C[0][0] * x_e[0] + C[0][1] * x_e[1]), baro->vel_raw - (C[1][0] * x_e[0] + C[1][1] * x_e[1]) };
    x[0] = x_e[0] + K[0][0] * inov[0] + K[0][1] * inov[1];
    x[1] = x_e[1] + K[1][0] * inov[0] + K[1][1] * inov[1];

    baro->altitude = x[0];
    baro->ver_vel = x[1];

    // update covariance matrix:
    float temp[2][2];
    temp[0][0] = 1 - (K[0][0] * C[0][0] + K[0][1] * C[1][0]);
    temp[0][1] = -(K[0][0] * C[0][1] + K[0][1] * C[1][1]);
    temp[1][0] = -(K[1][0] * C[0][0] + K[1][1] * C[1][0]);
    temp[1][1] = 1 - (K[1][0] * C[0][1] + K[1][1] * C[1][1]);

    P[0][0] = temp[0][0] * P_temp[0][0] + temp[0][1] * P_temp[1][0];
    P[0][1] = temp[0][0] * P_temp[0][1] + temp[0][1] * P_temp[1][1];
    P[1][0] = temp[1][0] * P_temp[0][0] + temp[1][1] * P_temp[1][0];
    P[1][1] = temp[1][0] * P_temp[0][1] + temp[1][1] * P_temp[1][1];
}

void baro_set_h0_preasure(baro_t* baro) {
    baro->h0_preasure = baro->filtered_preasure;
}

void alt_hold(timeUs_t dt_us) {

    float dt = US_TO_SEC(dt_us);

    static uint8_t alt_hold;
    threef_t acc_vec = { Gyro_Acc[3], Gyro_Acc[4], Gyro_Acc[5] };

    // transform acc values into global frame and add filtering:
    acc_z = acc_z * 0.9f + 0.1f * quaternion_rotate_vector(acc_vec, quaternion_conjugate(q_global_attitude)).yaw;

    if (Arming_status == ARMED) {
        if (alt_hold == 1) {
            float vertical_vel_desired = 0;
            static float vertical_vel_desired_prev;
            // if throttle is out of center bounds velocity will be set and desired altitiude will be updated to current altitude
            if (receiver.Throttle > THROTTLE_HOLD_UP) {
                vertical_vel_desired = (float)(receiver.Throttle - THROTTLE_HOLD_UP) / (THROTTLE_MAX - THROTTLE_HOLD_UP) * MAX_RISING_RATE;
                desired_altitude = 0.1f * desired_altitude + 0.9f * baro_1.altitude;// smooth out 
            }
            else if (receiver.Throttle < THROTTLE_HOLD_LOW) {
                vertical_vel_desired = (float)(receiver.Throttle - THROTTLE_HOLD_LOW) / (THROTTLE_HOLD_LOW - THROTTLE_MIN) * MAX_DECLINE_RATE;
                desired_altitude = 0.1f * desired_altitude + 0.9f * baro_1.altitude;// smooth out 
            }
            else {
                // if throttle is in the middle position altitude will be hold and desired vertical speed is received from altitude error
                vertical_vel_desired = (desired_altitude - baro_1.altitude) * THR_ALT_P;
                if (vertical_vel_desired > MAX_RISING_RATE) {
                    vertical_vel_desired = MAX_RISING_RATE;
                }
                else if (vertical_vel_desired < -MAX_DECLINE_RATE) {
                    vertical_vel_desired = -MAX_DECLINE_RATE;
                }
            }

            float err_rate = vertical_vel_desired - baro_1.ver_vel;

            // PID controll for rate:
            corr_rate_throttle.P = rate_throttle_PIDF.P * err_rate;
            corr_rate_throttle.I += rate_throttle_PIDF.I * err_rate * dt;
            corr_rate_throttle.D = rate_throttle_PIDF.D * (1 - acc_z);
            corr_rate_throttle.F = rate_throttle_PIDF.F * (vertical_vel_desired - vertical_vel_desired_prev) / dt;

            vertical_vel_desired_prev = vertical_vel_desired;


            // if (corr_rate_throttle.I < 0) {
            //     corr_rate_throttle.I = 0;
            // }
            // else if (corr_rate_throttle.I > THROTTLE_MAX - THROTTLE_MIN) {
            //     corr_rate_throttle.I = THROTTLE_MAX - THROTTLE_MIN;
            // }

            if (corr_rate_throttle.I < -0.5) {
                corr_rate_throttle.I = -0.5;
            }
            else if (corr_rate_throttle.I > 0.5) {
                corr_rate_throttle.I = 0.5;
            }

            float acc_z_desired = 1 + corr_rate_throttle.P + corr_rate_throttle.I + corr_rate_throttle.D + corr_rate_throttle.F;

            if (acc_z_desired > 1.5f)
            {
                acc_z_desired = 1.5f;
            }
            else if (acc_z_desired < 0.5f) {
                acc_z_desired = 0.5f;
            }

            baro_rate_filtering(&acc_z_desired, acc_z_desired);

            // only for debuging in blackbox:
            debug_variable[0] = desired_altitude;
            debug_variable[1] = acc_z_desired * 100;

            float err_acc = acc_z_desired - acc_z;
            static float err_acc_prev;
            corr_acc_throttle.P = acc_throttle_PIDF.P * err_acc;
            corr_acc_throttle.I += acc_throttle_PIDF.I * err_acc * dt;
            corr_acc_throttle.D = acc_throttle_PIDF.D * (err_acc - err_acc_prev) / dt;
            err_acc_prev = err_acc;
            baro_D_term_filtering();
            throttle = THROTTLE_MIN + corr_acc_throttle.P + corr_acc_throttle.I + corr_acc_throttle.D;


            // throttle = THROTTLE_MIN + corr_rate_throttle.P + corr_rate_throttle.I + corr_rate_throttle.D + corr_rate_throttle.F;

             //  limit throttle value
            if (throttle > THROTTLE_MAX) {
                throttle = THROTTLE_MAX;
            }
            else if (throttle < THROTTLE_MIN) {
                throttle = THROTTLE_MIN;
            }
        }
        else if (alt_hold == 0)
        {
            // if drone starts to move up activate Alt hold
            if (acc_z >= 1.1f) {
                alt_hold = 1;
                // set current altitude as desired level:
                desired_altitude = baro_1.altitude;
                // set I correction as current throtle value so quad will stay in position:

                // corr_rate_throttle.I = receiver.Throttle - THROTTLE_MIN;
                corr_acc_throttle.I = receiver.Throttle - THROTTLE_MIN;

                throttle = receiver.Throttle;
            }
            else {
                // use throttle from receiver (normal stabilize mode)
                throttle = receiver.Throttle;
            }
        }
    }
    else {
        // reset corrections:
        corr_baro.I = 0;
        corr_rate_throttle.I = 0;
        corr_acc_throttle.I = 0;
        throttle = 0;
        alt_hold = 0;
        desired_altitude = 0;
    }

}



