#ifndef BATTERY_H_
#define BATTERY_H_

enum battery_s
{
    BATTERY_NOT_CONNECTED,
    BATTERY_OK,
    BATTERY_WARNING
};

typedef struct
{
    float voltage;
    float voltage_filtered;
    bool voltage_stable;
    uint8_t cells_number;
    float cell_voltage;
    enum battery_s BATTERY_STATUS;

} battery_t;

extern battery_t main_battery;

void battery_manage();

#endif //  BATTERY_H_