
#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include "stdlib.h"

#include "battery.h"

//-------------BATTERY------------

battery_t main_battery = {
    .voltage = 0,
    .voltage_filtered = 0,
    .voltage_stable = false,
    .cells_number = 0,
    .cell_voltage = 0,
    .BATTERY_STATUS = BATTERY_NOT_CONNECTED};

void battery_manage()
{
    if (main_battery.BATTERY_STATUS == BATTERY_NOT_CONNECTED)
    {
        // wait for voltage to stabilize:
        if (abs(main_battery.voltage_filtered - main_battery.voltage) < 0.1 && main_battery.voltage_filtered > 0)
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
                main_battery.BATTERY_STATUS = BATTERY_NOT_CONNECTED;
                main_battery.cells_number = 0;
                main_battery.cell_voltage = 0;
            }
            else
            {
                main_battery.BATTERY_STATUS = BATTERY_OK;
            }
        }
    }
    else if (main_battery.BATTERY_STATUS >= BATTERY_OK)
    {
        main_battery.cell_voltage = main_battery.voltage_filtered / main_battery.cells_number;
        if (main_battery.cell_voltage < BATTERY_CELL_MIN_VOLTAGE)
        {
            main_battery.BATTERY_STATUS = BATTERY_LOW;
        }

        //  if battery disconnected:
        if (main_battery.voltage_filtered < BATTERY_CELL_MIN_VOLTAGE)
        {
            main_battery.BATTERY_STATUS = BATTERY_NOT_CONNECTED;
            main_battery.voltage_stable = false;
        }
    };
}