#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include "flash.h"
#include "stdlib.h"
#include "sensors/barometer.h"
#include "filters.h"
#include "ibus.h"
#include "rx.h"


static bool RX_initialization(rx_t* receiver, uint8_t channels_to_use);

void failsafe_rx(timeUs_t current_time)
{
    if ((current_time - receiver.last_time) >= SEC_TO_US(MAX_NO_SIGNAL_TIME))
    {
        FailSafe_status = FAILSAFE_RX_TIMEOUT;
        EXTI->SWIER |= EXTI_SWIER_SWIER15;
    }
}

void rx_handling(timeUs_t current_time)
{
    if (Ibus_save(&receiver)) {
        // update time of the most recent read frame:
        receiver.last_time = current_time;

        switch (Arming_status)
        {
        case DISARMED:
#if defined(USE_PREARM)
            //	if prearm switch is set, arming switch is off, throttle is low:
            if (receiver.channels[PREARM_CHANNEL] >= PREARM_VALUE && receiver.channels[ARM_CHANNEL] < ARM_VALUE)
            {
                if (receiver.Throttle <= MAX_ARM_THROTTLE_VAL) {
                    Arming_status = PREARMED;
                }
                else {
                    FailSafe_status = FAILSAFE_THROTTLE_PREARM;
                    EXTI->SWIER |= EXTI_SWIER_SWIER15;
                }
            }
            else if (receiver.channels[ARM_CHANNEL] > ARM_VALUE)
            {
                FailSafe_status = FAILSAFE_NO_PREARM;
                EXTI->SWIER |= EXTI_SWIER_SWIER15;
            }
#else
            if (receiver.channels[ARM_CHANNEL] >= ARM_VALUE)
            {
                if (gyro_1.calibrated == false) {
                    FailSafe_status = FAILSAFE_GYRO_CALIBRATION;
                    EXTI->SWIER |= EXTI_SWIER_SWIER15;
                }
                else if (receiver.Throttle > MAX_ARM_THROTTLE_VAL)
                {
                    FailSafe_status = FAILSAFE_THROTTLE_PREARM;
                    EXTI->SWIER |= EXTI_SWIER_SWIER15;
                }
                else
                {
                    Arming_status = ARMED;
                }
            }
#endif
            break;
        case PREARMED:
            if (gyro_1.calibrated == false) {
                FailSafe_status = FAILSAFE_GYRO_CALIBRATION;
                EXTI->SWIER |= EXTI_SWIER_SWIER15;
            }
            else if (receiver.channels[ARM_CHANNEL] >= ARM_VALUE)
            {
                Arming_status = ARMED;
#if defined(USE_BARO)
                baro_set_h0_preasure(&baro_1);
#endif
            }
            else if (receiver.channels[PREARM_CHANNEL] < PREARM_VALUE || receiver.Throttle > MAX_ARM_THROTTLE_VAL)
            {
                Arming_status = DISARMED;
            }
            break;

        case ARMED:
            // Arming switch:
            if (receiver.channels[4] < ARM_VALUE)
            {
                Arming_status = DISARMED;
                motor_1_value_pointer = &MOTOR_OFF;
                motor_2_value_pointer = &MOTOR_OFF;
                motor_3_value_pointer = &MOTOR_OFF;
                motor_4_value_pointer = &MOTOR_OFF;
            }
            else if (receiver.channels[0] <= MIN_RX_SIGNAL ||
                receiver.channels[0] >= MAX_RX_SIGNAL ||
                receiver.channels[1] <= MIN_RX_SIGNAL ||
                receiver.channels[1] >= MAX_RX_SIGNAL ||
                receiver.channels[2] <= MIN_RX_SIGNAL ||
                receiver.channels[2] >= MAX_RX_SIGNAL ||
                receiver.channels[3] <= MIN_RX_SIGNAL ||
                receiver.channels[3] >= MAX_RX_SIGNAL)
            {
                FailSafe_status = FAILSAFE_INCORRECT_CHANNELS_VALUES;
                EXTI->SWIER |= EXTI_SWIER_SWIER15;
            }
            else
            {
                motor_1_value_pointer = &motor_value[0];
                motor_2_value_pointer = &motor_value[1];
                motor_3_value_pointer = &motor_value[2];
                motor_4_value_pointer = &motor_value[3];
            }
        default:
            break;
        }


        // FLIGHT MODE:
        if (receiver.channels[FLIGHT_MODE_CHANNEL] < 1400)
        {
            flight_mode = FLIGHT_MODE_ACRO;
            turn_OFF_RED_LED();
            turn_ON_BLUE_LED();
        }
        else if (receiver.channels[FLIGHT_MODE_CHANNEL] > 1450 && receiver.channels[FLIGHT_MODE_CHANNEL] < 1750)
        {
            flight_mode = FLIGHT_MODE_STABLE;
            turn_OFF_BLUE_LED();
            turn_ON_RED_LED();
        }
        else if (receiver.channels[FLIGHT_MODE_CHANNEL] >= 1750)
        {
            flight_mode = FLIGHT_MODE_ALT_HOLD;
            toggle_RED_LED();
        }

        // BLACKBOX:
        if (receiver.channels[BLACKBOX_CHANNEL] >= 1400 && receiver.channels[BLACKBOX_CHANNEL] < 1700) {
            //blackbox switch active:
            if (Blackbox_status == BLACKBOX_IDLE) {
                Blackbox_status = BLACKBOX_COLLECTING_DATA;
            }
            else if (Blackbox_status == BLACKBOX_SUSPENDED) {
                Blackbox_status = BLACKBOX_COLLECTING_DATA;
            }

        }   //blackbox switch disable:
        else if (Blackbox_status == BLACKBOX_COLLECTING_DATA)
        {
            Blackbox_status = BLACKBOX_SUSPENDED;
        }
        else if (Blackbox_status == BLACKBOX_SUSPENDED && Arming_status == DISARMED) {
            Blackbox_status = BLACKBOX_STOPPED;
        }

        // reset flag for new data:
        receiver.new_data_flag = false;


#if defined(USE_RC_SMOOTHING)
        // if rc smoothing turn on rc channels will be filtered and new setpoints will be calculated for each iteration
        // otherwise only for new values action will be taken
    }
    rc_filtering();
#endif

    // Rates applying:
    receiver.Throttle = receiver.channels[2];
    if (receiver.Throttle > TPA_BREAKPOINT) {
        tpa_coef = 1 - TPA_MAX_VALUE * (float)(receiver.Throttle - TPA_BREAKPOINT) / (THROTTLE_MAX - TPA_BREAKPOINT);
    }
    else {
        tpa_coef = 1;
    }
    // variable for calculations:
    float temp;
#if defined(RATES_USE_EXPO)
    //  y = temp*b + (c*temp^5 + temp^3*(1-c))*(a-b)
    switch (flight_mode) {
    case FLIGHT_MODE_ACRO:
        // normalize input (-1; 1)
        temp = (float)(receiver.channels[0] - 1500) / 500.f;
        desired_rotation_speed.roll = temp * RATES_CENTER_RATE_R + temp * temp * temp * (RATES_EXPO_R * temp * temp + 1 - RATES_EXPO_R) * (RATES_MAX_RATE_R - RATES_CENTER_RATE_R);
        temp = (float)(receiver.channels[1] - 1500) / 500.f;
        desired_rotation_speed.pitch = temp * RATES_CENTER_RATE_P + temp * temp * temp * (RATES_EXPO_P * temp * temp + 1 - RATES_EXPO_P) * (RATES_MAX_RATE_P - RATES_CENTER_RATE_P);
        temp = (float)(receiver.channels[3] - 1500) / 500.f;
        desired_rotation_speed.yaw = temp * RATES_CENTER_RATE_Y + temp * temp * temp * (RATES_EXPO_Y * temp * temp + 1 - RATES_EXPO_Y) * (RATES_MAX_RATE_Y - RATES_CENTER_RATE_Y);

    case FLIGHT_MODE_STABLE || FLIGHT_MODE_ALT_HOLD:
        temp = (float)(receiver.channels[0] - 1500) / 500.f;
        desired_angles.roll = (temp * RATES_CENTER_RATE_R + temp * temp * temp * (RATES_EXPO_R * temp * temp + 1 - RATES_EXPO_R) * (RATES_MAX_RATE_R - RATES_CENTER_RATE_R)) / (float)RATES_MAX_RATE_R * MAX_ROLL_ANGLE;
        temp = (float)(receiver.channels[1] - 1500) / 500.f;
        desired_angles.pitch = (temp * RATES_CENTER_RATE_P + temp * temp * temp * (RATES_EXPO_P * temp * temp + 1 - RATES_EXPO_P) * (RATES_MAX_RATE_P - RATES_CENTER_RATE_P)) / (float)RATES_MAX_RATE_P * MAX_PITCH_ANGLE;
        // yaw keep as in acro:
        temp = (float)(receiver.channels[3] - 1500) / 500.f;
        desired_rotation_speed.yaw = temp * RATES_CENTER_RATE_Y + temp * temp * temp * (RATES_EXPO_Y * temp * temp + 1 - RATES_EXPO_Y) * (RATES_MAX_RATE_Y - RATES_CENTER_RATE_Y);
    default:
        break;
    }

#elif defined(RATES_USE_NO_EXPO)

    // y = temp * b + temp^3 * (a - b)
    switch (flight_mode) {
    case FLIGHT_MODE_ACRO:
        temp = (float)(receiver.channels[0] - 1500) / 500.f;
        desired_rotation_speed.roll = temp * RATES_CENTER_RATE_R + temp * temp * temp * (RATES_MAX_RATE_R - RATES_CENTER_RATE_R);
        temp = (float)(receiver.channels[1] - 1500) / 500.f;
        desired_rotation_speed.pitch = temp * RATES_CENTER_RATE_P + temp * temp * temp * (RATES_MAX_RATE_P - RATES_CENTER_RATE_P);
        temp = (float)(receiver.channels[3] - 1500) / 500.f;
        desired_rotation_speed.yaw = temp * RATES_CENTER_RATE_Y + temp * temp * temp * (RATES_MAX_RATE_Y - RATES_CENTER_RATE_Y);
        break;

    case FLIGHT_MODE_STABLE:
        temp = (float)(receiver.channels[0] - 1500) / 500.f;
        desired_angles.roll = (temp * RATES_CENTER_RATE_R + temp * temp * temp * (RATES_MAX_RATE_R - RATES_CENTER_RATE_R)) / (float)RATES_MAX_RATE_R * MAX_ROLL_ANGLE;
        temp = (float)(receiver.channels[1] - 1500) / 500.f;
        desired_angles.pitch = (temp * RATES_CENTER_RATE_P + temp * temp * temp * (RATES_MAX_RATE_P - RATES_CENTER_RATE_P)) / (float)RATES_MAX_RATE_P * MAX_PITCH_ANGLE;
        // yaw keep as in acro:
        temp = (float)(receiver.channels[3] - 1500) / 500.f;
        desired_rotation_speed.yaw = temp * RATES_CENTER_RATE_Y + temp * temp * temp * (RATES_MAX_RATE_Y - RATES_CENTER_RATE_Y);
        break;
    case FLIGHT_MODE_ALT_HOLD:
        temp = (float)(receiver.channels[0] - 1500) / 500.f;
        desired_angles.roll = (temp * RATES_CENTER_RATE_R + temp * temp * temp * (RATES_MAX_RATE_R - RATES_CENTER_RATE_R)) / (float)RATES_MAX_RATE_R * MAX_ROLL_ANGLE;
        temp = (float)(receiver.channels[1] - 1500) / 500.f;
        desired_angles.pitch = (temp * RATES_CENTER_RATE_P + temp * temp * temp * (RATES_MAX_RATE_P - RATES_CENTER_RATE_P)) / (float)RATES_MAX_RATE_P * MAX_PITCH_ANGLE;
        // yaw keep as in acro:
        temp = (float)(receiver.channels[3] - 1500) / 500.f;
        desired_rotation_speed.yaw = temp * RATES_CENTER_RATE_Y + temp * temp * temp * (RATES_MAX_RATE_Y - RATES_CENTER_RATE_Y);
        break;
    default:
        break;

    }

#endif
#if !defined(USE_RC_SMOOTHING)
}   // if rc smoothing turn off new setpoints calculated only for new channels values
#endif

failsafe_rx(current_time);
};

void setup_rx() {
    RX_initialization(&receiver, CHANNELS);
}

static bool RX_initialization(rx_t* receiver, uint8_t channels_to_use) {
    //	Allocate memory for arrays
    receiver->channels = malloc(sizeof(*(receiver->channels)) * receiver->number_of_channels);
    receiver->number_of_used_channels = channels_to_use;
    for (uint8_t i = 0; i < receiver->number_of_used_channels; i++) {
        receiver->channels[i] = MIN_RX_SIGNAL;
    }
    receiver->Throttle = MIN_RX_SIGNAL;
    receiver->new_data_flag = false;
    return true;
}