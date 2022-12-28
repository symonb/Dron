#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include "rx.h"

void failsafe_RX()
{
#if defined(USE_PREARM)

    if (ARMING_STATUS == DISARMED)
    {
        //	if prearm switch is set, arming switch is off, throttle is low:
        if (channels[PREARM_CHANNEL] >= PREARM_VALUE && channels[ARM_CHANNEL] < ARM_VALUE && Throttle <= MAX_ARM_THROTTLE_VAL)
        {
            ARMING_STATUS = PREARMED;
        }
        else if (channels[ARM_CHANNEL] > ARM_VALUE)
        {
            FailSafe_status = NO_PREARM;
            EXTI->SWIER |= EXTI_SWIER_SWIER15;
        }
    }

    if (ARMING_STATUS == PREARMED)
    {
        if (channels[ARM_CHANNEL] > ARM_VALUE)
        {
            ARMING_STATUS = ARMED;
        }
        else if (channels[PREARM_CHANNEL] < PREARM_VALUE || Throttle > MAX_ARM_THROTTLE_VAL)
        {
            ARMING_STATUS = DISARMED;
        }
    }

#else
    if (ARMING_STATUS == DISARMED)
    {
        if (Throttle <= MAX_ARM_THROTTLE_VAL && channels[ARM_CHANNEL] < ARM_VALUE)
        {
            ARMING_STATUS = PREARMED;
        }
        else
        {
            ARMING_STATUS = DISARMED;
        }
    }
    if (ARMING_STATUS == PREARMED)
    {
        if (channels[ARM_CHANNEL] > ARM_VALUE)
        {
            ARMING_STATUS = ARMED;
        }
        else if (Throttle > MAX_ARM_THROTTLE_VAL)
        {
            ARMING_STATUS = DISARMED;
        }
    }

#endif

    else if (ARMING_STATUS == ARMED)
    {
        // Arming switch:
        if (channels[4] < ARM_VALUE)
        {
            ARMING_STATUS = DISARMED;
            motor_1_value_pointer = &MOTOR_OFF;
            motor_2_value_pointer = &MOTOR_OFF;
            motor_3_value_pointer = &MOTOR_OFF;
            motor_4_value_pointer = &MOTOR_OFF;
        }
        else if (channels[0] <= MIN_RX_SIGNAL ||
                 channels[0] >= MAX_RX_SIGNAL ||
                 channels[1] <= MIN_RX_SIGNAL ||
                 channels[1] >= MAX_RX_SIGNAL ||
                 channels[2] <= MIN_RX_SIGNAL ||
                 channels[2] >= MAX_RX_SIGNAL ||
                 channels[3] <= MIN_RX_SIGNAL ||
                 channels[3] >= MAX_RX_SIGNAL)
        {

            for (uint8_t i = 0; i < CHANNELS; i++)
            {
                channels[i] = channels_previous_values[i];
            }

            FailSafe_status = INCORRECT_CHANNELS_VALUES;
            EXTI->SWIER |= EXTI_SWIER_SWIER15;
        }
        else
        {
            motor_1_value_pointer = &motor_1_value;
            motor_2_value_pointer = &motor_2_value;
            motor_3_value_pointer = &motor_3_value;
            motor_4_value_pointer = &motor_4_value;
        }
    }
}

void RX_handling()
{

    if (channels[FLIGHT_MODE_CHANNEL] < 1400)
    {
        flight_mode = FLIGHT_MODE_ACRO;
        turn_OFF_RED_LED();
        turn_ON_BLUE_LED();
    }
    else if (channels[FLIGHT_MODE_CHANNEL] > 1450)
    {
        flight_mode = FLIGHT_MODE_STABLE;
        turn_OFF_BLUE_LED();
        turn_ON_RED_LED();
    }

    // Rates applying:
    Throttle = channels[2];
    static float x;
#if defined(RATES_USE_EXPO)
    //  y = x*b + (c*x^5 + x^3*(1-c))*(a-b)

    if (flight_mode == FLIGHT_MODE_ACRO)
    {
        // normalize input (-1; 1)
        x = (float)(channels[0] - 1500) / 500.f;
        desired_rotation_speed.roll = x * RATES_CENTER_RATE_R + x * x * x * (RATES_EXPO_R * x * x + 1 - RATES_EXPO_R) * (RATES_MAX_RATE_R - RATES_CENTER_RATE_R);
        x = (float)(channels[1] - 1500) / 500.f;
        desired_rotation_speed.pitch = x * RATES_CENTER_RATE_P + x * x * x * (RATES_EXPO_R * x * x + 1 - RATES_EXPO_R) * (RATES_MAX_RATE_R - RATES_CENTER_RATE_R);
        x = (float)(channels[3] - 1500) / 500.f;
        desired_rotation_speed.yaw = x * RATES_CENTER_RATE_R + x * x * x * (RATES_EXPO_R * x * x + 1 - RATES_EXPO_R) * (RATES_MAX_RATE_R - RATES_CENTER_RATE_R);
    }
    else if (flight_mode == FLIGHT_MODE_STABLE)
    {
    }

#elif defined(RATES_USE_NO_EXPO)

    // y = x * b + x^3 * (a - b)
    if (flight_mode == FLIGHT_MODE_ACRO)
    {
        x = (float)(channels[0] - 1500) / 500.f;
        desired_rotation_speed.roll = x * RATES_CENTER_RATE_R + x * x * x * (RATES_MAX_RATE_R - RATES_CENTER_RATE_R);
        x = (float)(channels[1] - 1500) / 500.f;
        desired_rotation_speed.pitch = x * RATES_CENTER_RATE_P + x * x * x * (RATES_MAX_RATE_P - RATES_CENTER_RATE_P);
        x = (float)(channels[3] - 1500) / 500.f;
        desired_rotation_speed.yaw = x * RATES_CENTER_RATE_Y + x * x * x * (RATES_MAX_RATE_Y - RATES_CENTER_RATE_Y);
    }
    else if (flight_mode == FLIGHT_MODE_STABLE)
    {
        x = (float)(channels[0] - 1500) / 500.f;
        desired_angles.roll = (x * RATES_CENTER_RATE_R + x * x * x * (RATES_MAX_RATE_R - RATES_CENTER_RATE_R)) / (float)RATES_MAX_RATE_R * MAX_ROLL_ANGLE;
        x = (float)(channels[1] - 1500) / 500.f;
        desired_angles.pitch = (x * RATES_CENTER_RATE_P + x * x * x * (RATES_MAX_RATE_P - RATES_CENTER_RATE_P)) / (float)RATES_MAX_RATE_P * MAX_PITCH_ANGLE;
        // yaw keep as in acro:
        x = (float)(channels[3] - 1500) / 500.f;
        desired_rotation_speed.yaw = x * RATES_CENTER_RATE_Y + x * x * x * (RATES_MAX_RATE_Y - RATES_CENTER_RATE_Y);
    }
#endif

    if (channels[BLACKBOX_CHANNEL] >= 1400 && channels[BLACKBOX_CHANNEL] < 1700)
    {
        BLACKBOX_STATUS = BLACKBOX_COLLECT_DATA;
    }
    else if (channels[BLACKBOX_CHANNEL] >= 1700)
    {
        if (BLACKBOX_STATUS != BLACKBOX_ERASE)
        {
            BLACKBOX_STATUS = BLACKBOX_ERASE;
            flash_full_chip_erase();
        }
    }
    else if (BLACKBOX_STATUS != BLACKBOX_SEND_DATA)
    {
        BLACKBOX_STATUS = BLACKBOX_IDLE;
    }
};