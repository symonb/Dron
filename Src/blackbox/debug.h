#ifndef DEBUG_H_
#define DEBUG_H_


typedef enum {
    DEBUG_NONE = -1,
    DEBUG_ACC_RAW = 4,
    DEBUG_GYRO_SCALED = 6,
    DEBUG_RC_SMOOTHING = 39,
    DEBUG_DSHOT_RPM_ERRORS = 51,
    DEBUG_BARO = 0,
    DEBUG_ATTITUDE = 75,
}blackbox_debug_e;

#endif