#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include <math.h>
#include "battery.h"
#include "mixer.h"



void set_motors(threef_t corr)
{
    // MIXER:
    if (flight_mode == FLIGHT_MODE_ACRO || flight_mode == FLIGHT_MODE_STABLE) {
        throttle = receiver.Throttle;
    }

    //	right back:
    motor_value[0] = (throttle - corr.roll + corr.pitch - corr.yaw) * 2;
    //	right front:
    motor_value[1] = (throttle - corr.roll - corr.pitch + corr.yaw) * 2;
    //	left back:
    motor_value[2] = (throttle + corr.roll + corr.pitch + corr.yaw) * 2;
    //	left front:
    motor_value[3] = (throttle + corr.roll - corr.pitch - corr.yaw) * 2;

    // AIRMODE:
    // When motor is below min throttle it would be clipped.
    // Let's adjust a throttle to set min motor to idle value and others accordingly higher.
    // This also holds true for max throttle (if max value is higher than possible set throttle lower).

    float max_delta = 0;
    float min_delta = 0;
    for (uint8_t i = 0; i < MOTORS_COUNT; ++i) {

        if (motor_value[i] < MOTOR_OUTPUT_MIN * 2) {
            min_delta = MIN(motor_value[i] - MOTOR_OUTPUT_MIN * 2, min_delta);
        }
        else if (motor_value[i] > MOTOR_OUTPUT_MAX * 2) {
            max_delta = MAX(motor_value[i] - MOTOR_OUTPUT_MAX * 2, max_delta);
        }
    }

    // adjust motors values if necessary: 
    if (min_delta < 0) {
        // whem min value is too small increase all values:
        for (uint8_t i = 0; i < MOTORS_COUNT; ++i) {
            motor_value[i] = MIN(MOTOR_OUTPUT_MAX * 2, motor_value[i] - min_delta);
        }
    }
    else if (max_delta > 0) {
        // when max value is too big reduce all values:
        for (uint8_t i = 0; i < MOTORS_COUNT; ++i) {
            motor_value[i] = MAX(MOTOR_OUTPUT_MIN * 2, motor_value[i] - max_delta);
        }
    }
#if defined(USE_BATTERY_SAG_COMPENSATION)
    float attenuation = MIN(1 - (main_battery.cell_voltage - BATTERY_CELL_MIN_VOLTAGE) / BATTERY_CELL_MAX_VOLTAGE, 1);

    for (uint8_t i = 0; i < MOTORS_COUNT; ++i) {
        motor_value[i] = MOTOR_OUTPUT_MIN * 2 + (motor_value[i] - MOTOR_OUTPUT_MIN * 2) * attenuation;
    }
#endif
}