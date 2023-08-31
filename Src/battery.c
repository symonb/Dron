
#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include "stdlib.h"
#include "math.h"
#include "adc1.h"
#include "battery.h"


const float V_25 = 0.76f;    // [V]
const float AVG_SLOPE = 400; // [C/V]

//-------------BATTERY------------

battery_t main_battery = {
    .voltage = 0,
    .ADC_value = {0,0},
    .voltage_filtered = 0,
    .voltage_stable = false,
    .cells_number = 0,
    .cell_voltage = 0,
    .status = BATTERY_NOT_CONNECTED };

void ADC1_start();

void battery_manage()
{
    //	read MCU temp. and battery voltage:
    ADC1_start();

    if (main_battery.status == BATTERY_NOT_CONNECTED)
    {
        // wait for voltage to stabilize:
        if (fabs(main_battery.voltage_filtered - main_battery.voltage) < 0.1 && main_battery.voltage_filtered > 0)
        {
            main_battery.voltage_stable = true;
            // calculate number of cells:

            if (main_battery.voltage_filtered <= BATTERY_CELL_MAX_VOLTAGE)
            {
                main_battery.cells_number = 1;
            }
            else if (main_battery.voltage_filtered <= BATTERY_CELL_MAX_VOLTAGE * 2)
            {
                main_battery.cells_number = 2;
            }
            else if (main_battery.voltage_filtered <= BATTERY_CELL_MAX_VOLTAGE * 3)
            {
                main_battery.cells_number = 3;
            }
            else if (main_battery.voltage_filtered <= BATTERY_CELL_MAX_VOLTAGE * 4)
            {
                main_battery.cells_number = 4;
            }
            else if (main_battery.voltage_filtered <= BATTERY_CELL_MAX_VOLTAGE * 5)
            {
                main_battery.cells_number = 5;
            }
            else if (main_battery.voltage_filtered <= BATTERY_CELL_MAX_VOLTAGE * 6)
            {
                main_battery.cells_number = 6;
            }
            if (main_battery.voltage < BATTERY_CELL_MIN_VOLTAGE)
            {
                main_battery.status = BATTERY_NOT_CONNECTED;
                main_battery.cells_number = 0;
                main_battery.cell_voltage = 0;
            }
            else
            {
                main_battery.status = BATTERY_OK;
            }
        }
    }
    else if (main_battery.status >= BATTERY_OK)
    {
        main_battery.cell_voltage = main_battery.voltage_filtered / main_battery.cells_number;
        if (main_battery.cell_voltage < BATTERY_CELL_WARNING_VOLTAGE) {
            main_battery.status = BATTERY_LOW;
        }
        else if (main_battery.cell_voltage < BATTERY_CELL_MIN_VOLTAGE)
        {
            main_battery.status = BATTERY_VERY_LOW;
        }

        //  if battery disconnected:
        if (main_battery.voltage_filtered < BATTERY_CELL_MIN_VOLTAGE)
        {
            main_battery.status = BATTERY_NOT_CONNECTED;
            main_battery.voltage_stable = false;
        }
    };
}


void ADC1_start()
{
    //  convert previous measurements:
    main_battery.voltage = main_battery.ADC_value[0] * ADC_REFERENCE_VOLTAGE / 0xFFF * BATTERY_SCALE;
    main_battery.voltage_filtered = main_battery.voltage_filtered * 0.9 + main_battery.voltage * 0.1;
    MCU_temperature = MCU_temperature * 0.7f + ((main_battery.ADC_value[1] * ADC_REFERENCE_VOLTAGE / 0xFFF - V_25) * AVG_SLOPE + 25) * 0.3f;

    //  turn on DMA:
    DMA2_Stream4->CR |= DMA_SxCR_EN;
    //  start conversion:
    ADC1->CR2 |= ADC_CR2_SWSTART;
}

float battery_get_voltage() {
    return main_battery.voltage_filtered;

}