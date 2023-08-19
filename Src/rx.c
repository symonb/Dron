#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include "flash.h"
#include "stdlib.h"
#include "rx.h"


static bool RX_initialization(rx_t* receiver, uint8_t channels_to_use);

void failsafe_RX()
{
#if defined(USE_PREARM)

    if (ARMING_STATUS == DISARMED)
    {
        //	if prearm switch is set, arming switch is off, throttle is low:
        if (receiver.channels[PREARM_CHANNEL] >= PREARM_VALUE && receiver.channels[ARM_CHANNEL] < ARM_VALUE && receiver.Throttle <= MAX_ARM_THROTTLE_VAL)
        {
            ARMING_STATUS = PREARMED;
        }
        else if (receiver.channels[ARM_CHANNEL] > ARM_VALUE)
        {
            FailSafe_status = FS_NO_PREARM;
            EXTI->SWIER |= EXTI_SWIER_SWIER15;
        }
    }

    if (ARMING_STATUS == PREARMED)
    {
        if (gyro_1.calibrated == false) {
            FailSafe_status = FS_GYRO_CALIBRATION;
            EXTI->SWIER |= EXTI_SWIER_SWIER15;
        }
        else if (receiver.channels[ARM_CHANNEL] > ARM_VALUE)
        {
            ARMING_STATUS = ARMED;
        }
        else if (receiver.channels[PREARM_CHANNEL] < PREARM_VALUE || receiver.Throttle > MAX_ARM_THROTTLE_VAL)
        {
            ARMING_STATUS = DISARMED;
        }
    }

#else
    if (ARMING_STATUS == DISARMED)
    {
        if (Throttle <= MAX_ARM_THROTTLE_VAL && receiver.channels[ARM_CHANNEL] < ARM_VALUE)
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
        if (gyro_1.calibrated == false) {
            FailSafe_status = FS_GYRO_CALIBRATION;
            EXTI->SWIER |= EXTI_SWIER_SWIER15;
        }
        else if (receiver.channels[ARM_CHANNEL] > ARM_VALUE)
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
        if (receiver.channels[4] < ARM_VALUE)
        {
            ARMING_STATUS = DISARMED;
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

            for (uint8_t i = 0; i < CHANNELS; i++)
            {
                receiver.channels[i] = receiver.channels_previous_values[i];
            }

            FailSafe_status = FS_INCORRECT_CHANNELS_VALUES;
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

    if (receiver.channels[FLIGHT_MODE_CHANNEL] < 1400)
    {
        flight_mode = FLIGHT_MODE_ACRO;
        turn_OFF_RED_LED();
        turn_ON_BLUE_LED();
    }
    else if (receiver.channels[FLIGHT_MODE_CHANNEL] > 1450)
    {
        flight_mode = FLIGHT_MODE_STABLE;
        turn_OFF_BLUE_LED();
        turn_ON_RED_LED();
    }

    // Rates applying:
    receiver.Throttle = receiver.channels[2];
    // variable for calculations:
    float temp;
#if defined(RATES_USE_EXPO)
    //  y = temp*b + (c*temp^5 + temp^3*(1-c))*(a-b)

    if (flight_mode == FLIGHT_MODE_ACRO)
    {
        // normalize input (-1; 1)
        temp = (float)(receiver.channels[0] - 1500) / 500.f;
        desired_rotation_speed.roll = temp * RATES_CENTER_RATE_R + temp * temp * temp * (RATES_EXPO_R * temp * temp + 1 - RATES_EXPO_R) * (RATES_MAX_RATE_R - RATES_CENTER_RATE_R);
        temp = (float)(receiver.channels[1] - 1500) / 500.f;
        desired_rotation_speed.pitch = temp * RATES_CENTER_RATE_P + temp * temp * temp * (RATES_EXPO_R * temp * temp + 1 - RATES_EXPO_R) * (RATES_MAX_RATE_R - RATES_CENTER_RATE_R);
        temp = (float)(receiver.channels[3] - 1500) / 500.f;
        desired_rotation_speed.yaw = temp * RATES_CENTER_RATE_R + temp * temp * temp * (RATES_EXPO_R * temp * temp + 1 - RATES_EXPO_R) * (RATES_MAX_RATE_R - RATES_CENTER_RATE_R);
    }
    else if (flight_mode == FLIGHT_MODE_STABLE)
    {
    }

#elif defined(RATES_USE_NO_EXPO)

    // y = temp * b + temp^3 * (a - b)
    if (flight_mode == FLIGHT_MODE_ACRO)
    {
        temp = (float)(receiver.channels[0] - 1500) / 500.f;
        desired_rotation_speed.roll = temp * RATES_CENTER_RATE_R + temp * temp * temp * (RATES_MAX_RATE_R - RATES_CENTER_RATE_R);
        temp = (float)(receiver.channels[1] - 1500) / 500.f;
        desired_rotation_speed.pitch = temp * RATES_CENTER_RATE_P + temp * temp * temp * (RATES_MAX_RATE_P - RATES_CENTER_RATE_P);
        temp = (float)(receiver.channels[3] - 1500) / 500.f;
        desired_rotation_speed.yaw = temp * RATES_CENTER_RATE_Y + temp * temp * temp * (RATES_MAX_RATE_Y - RATES_CENTER_RATE_Y);
    }
    else if (flight_mode == FLIGHT_MODE_STABLE)
    {
        temp = (float)(receiver.channels[0] - 1500) / 500.f;
        desired_angles.roll = (temp * RATES_CENTER_RATE_R + temp * temp * temp * (RATES_MAX_RATE_R - RATES_CENTER_RATE_R)) / (float)RATES_MAX_RATE_R * MAX_ROLL_ANGLE;
        temp = (float)(receiver.channels[1] - 1500) / 500.f;
        desired_angles.pitch = (temp * RATES_CENTER_RATE_P + temp * temp * temp * (RATES_MAX_RATE_P - RATES_CENTER_RATE_P)) / (float)RATES_MAX_RATE_P * MAX_PITCH_ANGLE;
        // yaw keep as in acro:
        temp = (float)(receiver.channels[3] - 1500) / 500.f;
        desired_rotation_speed.yaw = temp * RATES_CENTER_RATE_Y + temp * temp * temp * (RATES_MAX_RATE_Y - RATES_CENTER_RATE_Y);
    }
#endif

    if (receiver.channels[BLACKBOX_CHANNEL] >= 1400 && receiver.channels[BLACKBOX_CHANNEL] < 1700)
    {
        BLACKBOX_STATUS = BLACKBOX_COLLECT_DATA;
    }
    else if (receiver.channels[BLACKBOX_CHANNEL] >= 1700)
    {
        if (BLACKBOX_STATUS != BLACKBOX_ERASE)
        {
            BLACKBOX_STATUS = BLACKBOX_ERASE;
            W25Q128_erase_full_chip();
        }
    }
    else if (BLACKBOX_STATUS != BLACKBOX_SEND_DATA)
    {
        BLACKBOX_STATUS = BLACKBOX_IDLE;
    }
};

void setup_RX() {
    RX_initialization(&receiver, CHANNELS);
}

static bool RX_initialization(rx_t* receiver, uint8_t channels_to_use) {
    //	Allocate memory for arrays
    receiver->channels = (uint16_t*)malloc(sizeof(*(receiver->channels)) * receiver->number_of_channels);
    receiver->channels_previous_values = (uint16_t*)malloc(sizeof(*(receiver->channels_previous_values)) * receiver->number_of_channels);
    receiver->number_of_used_channels = channels_to_use;
    for (uint8_t i = 0; i < receiver->number_of_used_channels; i++) {
        receiver->channels[i] = MIN_RX_SIGNAL;
    }
    receiver->Throttle = MIN_RX_SIGNAL;
    receiver->new_data_flag = false;
    return true;
}