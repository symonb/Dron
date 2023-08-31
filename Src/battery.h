#ifndef BATTERY_H_
#define BATTERY_H_

typedef enum
{
    BATTERY_NOT_CONNECTED,
    BATTERY_OK,
    BATTERY_LOW,
    BATTERY_VERY_LOW
}battery_e;

typedef struct
{
    uint32_t ADC_value[2];
    float voltage;
    float voltage_filtered;
    bool voltage_stable;
    uint8_t cells_number;
    float cell_voltage;
    battery_e status;

} battery_t;

extern battery_t main_battery;

void battery_manage();
float battery_get_voltage();

#endif //  BATTERY_H_