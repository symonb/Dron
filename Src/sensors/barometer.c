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

    baro->vel_raw = (current_alt - baro->altitude) / dt;// + 0.8 * (baro_1.ver_vel + 40 * dt * (acc_z - 1));

    baro_vel_filtering();

    baro->altitude = current_alt;

}

void baro_set_h0_preasure(baro_t* baro) {
    baro->h0_preasure = baro->filtered_preasure;
}

void alt_hold(timeUs_t current_time) {

    static timeUs_t last_time;
    timeUs_t dt_us = current_time - last_time;
    //	prevent too big dt (e.g first iteration after being inactive):
    if (dt_us > 1.5f * TASK_PERIOD_HZ(FREQUENCY_ALT_HOLD))
    {
        dt_us = TASK_PERIOD_HZ(FREQUENCY_ALT_HOLD);
    }
    last_time = current_time;
    float dt = US_TO_SEC(dt_us);

    static uint8_t alt_hold;
    threef_t acc_vec = { Gyro_Acc[3] ,Gyro_Acc[4],Gyro_Acc[5] };

    // transform acc values into global frame and add filtering:
    acc_z = acc_z * 0.9 + 0.1 * quaternion_rotate_vector(acc_vec, quaternion_conjugate(q_global_attitude)).yaw;

    if (Arming_status == ARMED) {
        if (alt_hold == 1) {
            float vertical_vel_desired = 0;
            static float vertical_vel_desired_prev;
            // if throttle is out of center bounds velocity will be set and desired altitiude will be updated to current altitude
            if (receiver.Throttle > THROTTLE_HOLD_UP) {
                vertical_vel_desired = (float)(receiver.Throttle - THROTTLE_HOLD_UP) / (THROTTLE_MAX - THROTTLE_HOLD_UP) * MAX_RISING_RATE;
                desired_altitude = baro_1.altitude;
            }
            else if (receiver.Throttle < THROTTLE_HOLD_LOW) {
                vertical_vel_desired = (float)(receiver.Throttle - THROTTLE_HOLD_LOW) / (THROTTLE_HOLD_LOW - THROTTLE_MIN) * MAX_DECLINE_RATE;
                desired_altitude = baro_1.altitude;
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
            // only for debuging in blackbox:
            corr_rate_throttle.P = acc_z_desired;

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



