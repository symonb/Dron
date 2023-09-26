#ifndef BLACKBOX_H_
#define BLACKBOX_H_
#include "global_variables.h"
void blackbox_update(timeUs_t time);
bool blackbox_reserve_partition(uint32_t size);
bool blackbox_end_partition();
bool blackbox_init();

typedef struct {
    uint32_t fields_disabled_mask;
    uint8_t sample_rate;
}blackbox_config_t;

typedef enum {
    BLACKBOX_RESERVE_SUCCESS,
    BLACKBOX_RESERVE_TEMPORARY_FAILURE,
    BLACKBOX_RESERVE_PERMANENT_FAILURE
} blackboxBufferReserveStatus_e;

typedef enum BlackboxState {
    BLACKBOX_STATE_DISABLED = 0,
    BLACKBOX_STATE_STOPPED,
    BLACKBOX_STATE_PREPARE_LOG_FILE,
    BLACKBOX_STATE_SEND_HEADER,
    BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER,
    BLACKBOX_STATE_SEND_GPS_H_HEADER,
    BLACKBOX_STATE_SEND_GPS_G_HEADER,
    BLACKBOX_STATE_SEND_SLOW_HEADER,
    BLACKBOX_STATE_SEND_SYSINFO,
    BLACKBOX_STATE_CACHE_FLUSH,
    BLACKBOX_STATE_PAUSED,
    BLACKBOX_STATE_RUNNING,
    BLACKBOX_STATE_SHUTTING_DOWN,
    BLACKBOX_STATE_START_ERASE,
    BLACKBOX_STATE_ERASING,
    BLACKBOX_STATE_ERASED
} BlackboxState;

typedef struct blackboxMainState_s
{
    timeUs_t time;

    int32_t axisPID_P[3];
    int32_t axisPID_I[3];
    int32_t axisPID_D[3];
    int32_t axisPID_F[3];

    int16_t rcCommand[4];
    int16_t setpoint[4];
    int16_t gyroADC[3];
    int16_t accADC[3];
    int16_t debug[3];
    int16_t motor[MOTORS_COUNT];
    int16_t servo[1];

    uint16_t vbatLatest;
    int32_t amperageLatest;

#ifdef USE_BARO
    int32_t baroAlt;
#endif
#ifdef USE_MAG
    int16_t magADC[XYZ_AXIS_COUNT];
#endif

    uint16_t rssi;
} blackboxMainState_t;

// This data is updated really infrequently:
typedef struct blackboxSlowState_s
{
    uint32_t flightModeFlags; // extend this data size (from uint16_t)
    uint8_t stateFlags;
    uint8_t failsafePhase;
    bool rxSignalReceived;
    bool rxFlightChannelsValid;
} __attribute__((__packed__)) blackboxSlowState_t; // We pack this struct so that padding doesn't interfere with memcmp()

#define BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION 64

/*
 * We want to limit how bursty our writes to the device are. Note that this will also restrict the maximum size of a
 * header write we can make:
 */
#define BLACKBOX_MAX_ACCUMULATED_HEADER_BUDGET 256

#endif //  BLACKBOX_H_