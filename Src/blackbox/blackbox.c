/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "flash.h"
#include "ff.h"
#include "common/encoding.h"
#include "battery.h"
#include "math/quaternions.h"
#include "blackbox/blackbox_encoding.h"
#include "blackbox_fielddefs.h"
#include "blackbox/debug.h"
#include "blackbox/blackbox.h"

#define CONCAT(x,y) x ## y 

#define PREDICT(x) CONCAT(FLIGHT_LOG_FIELD_PREDICTOR_, x)
#define ENCODING(x) CONCAT(FLIGHT_LOG_FIELD_ENCODING_, x)
#define CONDITION(x) CONCAT(FLIGHT_LOG_FIELD_CONDITION_, x)
#define FIELD_SELECT(x) CONCAT(FLIGHT_LOG_FIELD_SELECT_, x)
#define UNSIGNED false
#define SIGNED true



static bool isFieldEnabled(FlightLogFieldSelect_e field);
static void blackboxBuildConditionCache(void);
static bool testBlackboxConditionUncached(FlightLogFieldCondition condition);
static bool testBlackboxCondition(FlightLogFieldCondition condition);
static bool sendFieldDefinition(char mainFrameChar, char deltaFrameChar, const void* fieldDefinitions,
    const void* secondFieldDefinition, int fieldCount, const uint8_t* conditions, const uint8_t* secondCondition);
static bool blackboxShouldLogIFrame(void);
static bool blackboxShouldLogPFrame(void);
static bool blackboxIsOnlyLoggingIntraframes(void);
static void loadSlowState(blackboxSlowState_t* slow);
static bool writeSlowFrameIfNeeded(void);
static void blackboxLogIteration(timeUs_t currentTimeUs);
static void blackboxAdvanceIterationTimers(void);
static void blackboxResetIterationTimers(void);
static void loadMainState(timeUs_t currentTimeUs);
blackboxBufferReserveStatus_e blackboxDeviceReserveBufferSpace(int32_t bytes);
void blackboxReplenishHeaderBudget(void);
static void blackboxSetState(BlackboxState newState);
static bool blackboxWriteSysinfo(void);
static void blackboxInit(void);
static void blackboxStart(void);
bool blackboxDeviceOpen(void);
static void writeInterframe(void);
static void writeIntraframe(void);
static void writeSlowFrame(void);
static void blackboxWriteMainStateArrayUsingAveragePredictor(int arrOffsetInHistory, int count);
static void blackboxCheckAndLogFlightMode(void);
void blackboxLogEvent(FlightLogEvent event, flightLogEventData_t* data);


// an I-frame is written every 32ms
// blackboxUpdate() is run in synchronisation with the PID loop
uint16_t blackboxIInterval;
uint16_t blackboxPInterval;
uint16_t blackboxSInterval;
// How many bytes can we write *this* iteration without overflowing transmit buffers or overstressing the OpenLog?
int32_t blackboxHeaderBudget;
// How many bytes can we transmit per loop iteration when writing headers?
static uint8_t blackboxMaxHeaderBytesPerIteration;
static BlackboxState blackboxState = BLACKBOX_STATE_DISABLED;
static uint32_t blackboxIteration;
static uint16_t blackboxLoopIndex;
static uint16_t blackboxPFrameIndex;
static uint16_t blackboxIFrameIndex;
int32_t blackboxSlowFrameIterationTimer;
static bool blackboxLoggedAnyFrames;
// Cache for FLIGHT_LOG_FIELD_CONDITION_* test results:
static uint32_t blackboxConditionCache;
static uint32_t blackboxLastFlightModeFlags = 0; // New event tracking of flight modes
// Keep a history of length 2, plus a buffer for MW to store the new values into
static blackboxMainState_t blackboxHistoryRing[3];
static blackboxSlowState_t slowHistory;

// These point into blackboxHistoryRing, use them to know where to store history of a given age (0, 1 or 2 generations old)
static blackboxMainState_t* blackboxHistory[3];


/*
 * We store voltages in I-frames relative to this, which was the voltage when the blackbox was activated.
 * This helps out since the voltage is only expected to fall from that point and we can reduce our diffs
 * to encode:
 */
static uint16_t vbatReference;

const blackbox_debug_e blackbox_debug =
#if defined(BLACKBOX_DEBUG_ACC_RAW)
DEBUG_ACC_RAW;
#elif defined(BLACKBOX_DEBUG_GYRO_RAW)
DEBUG_GYRO_SCALED;
#elif defined(BLACKBOX_DEBUG_CHANNELS_RAW)
DEBUG_RC_SMOOTHING;
#elif defined(BLACKBOX_DEBUG_BARO)
DEBUG_BARO;
#elif defined(BLACKBOX_DEBUG_RPM_ERR)
DEBUG_DSHOT_RPM_ERRORS;
#elif defined(BLACKBOX_DEBUG_ATTITIUDE)
DEBUG_ATTITUDE;
#else
DEBUG_NONE;
#endif

blackbox_config_t blackbox_config = {
    .fields_disabled_mask = ~(
      1 << (FIELD_SELECT(BATTERY))
    | 1 << (FIELD_SELECT(GYRO))
    | 1 << (FIELD_SELECT(ACC))
    | 1 << (FIELD_SELECT(MOTOR))
    | 1 << (FIELD_SELECT(RC_COMMANDS))
    | 1 << (FIELD_SELECT(SETPOINT))
    | 1 << (FIELD_SELECT(PID))
    | 1 << (FIELD_SELECT(ALTITUDE))
    | ((blackbox_debug != DEBUG_NONE) * 1) << (FIELD_SELECT(DEBUG_LOG))
    ),
    .sample_rate = BLACKBOX_SAMPLE_RATE };

static const char blackboxHeader[] =
"H Product:Blackbox flight data recorder by Nicholas Sherlock\n"
"H Data version:2\n";

static const char* const blackboxFieldHeaderNames[] = {
    "name",
    "signed",
    "predictor",
    "encoding",
    "predictor",
    "encoding"
};

/* All field definition structs should look like this (but with longer arrs): */
typedef struct blackboxFieldDefinition_s {
    const char* name;
    // If the field name has a number to be included in square brackets [1] afterwards, set it here, or -1 for no brackets:
    int8_t fieldNameIndex;

    // Each member of this array will be the value to print for this field for the given header index
    uint8_t arr[1];
} blackboxFieldDefinition_t;


#define ARRAYLEN(x) (sizeof(x) / sizeof((x)[0]))
#define BLACKBOX_DELTA_FIELD_HEADER_COUNT       ARRAYLEN(blackboxFieldHeaderNames)
#define BLACKBOX_SIMPLE_FIELD_HEADER_COUNT      (BLACKBOX_DELTA_FIELD_HEADER_COUNT - 2)
#define BLACKBOX_CONDITIONAL_FIELD_HEADER_COUNT (BLACKBOX_DELTA_FIELD_HEADER_COUNT - 2)
#ifndef BLACKBOX_PRINT_HEADER_LINE
#define BLACKBOX_PRINT_HEADER_LINE(name, format, ...) case __COUNTER__: blackboxPrintfHeaderLine(name, format, __VA_ARGS__);                                    
#endif


typedef struct blackboxSimpleFieldDefinition_s {
    const char* name;
    int8_t fieldNameIndex;

    uint8_t isSigned;
    uint8_t predict;
    uint8_t encode;
} blackboxSimpleFieldDefinition_t;

typedef struct blackboxDeltaFieldDefinition_s {
    const char* name;
    int8_t fieldNameIndex;

    bool isSigned;
    uint8_t Ipredict;
    uint8_t Iencode;
    uint8_t Ppredict;
    uint8_t Pencode;
    uint8_t condition; // Decide whether this field should appear in the log
} blackboxDeltaFieldDefinition_t;

static struct {
    uint32_t headerIndex;

    /* Since these fields are used during different blackbox states (never simultaneously) we can
     * overlap them to save on RAM
     */
    union {
        int fieldIndex;
        timeUs_t startTime;
    } u;
} xmitState;

/**
 * Description of the blackbox fields we are writing in our main intra (I) and inter (P) frames. This description is
 * written into the flight log header so the log can be properly interpreted (but these definitions don't actually cause
 * the encoding to happen, we have to encode the flight log ourselves in write{Inter|Intra}frame() in a way that matches
 * the encoding we've promised here).
 */
static const blackboxDeltaFieldDefinition_t blackboxMainFields[] = {
    /* loopIteration doesn't appear in P frames since it always increments */
    {"loopIteration",-1, UNSIGNED, .Ipredict = PREDICT(0),     .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(INC),           .Pencode = FLIGHT_LOG_FIELD_ENCODING_NULL, CONDITION(ALWAYS)},
    /* Time advances pretty steadily so the P-frame prediction is a straight line */
    {"time",       -1, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(STRAIGHT_LINE), .Pencode = ENCODING(SIGNED_VB), CONDITION(ALWAYS)},
    {"axisP",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(PID)},
    {"axisP",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(PID)},
    {"axisP",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(PID)},
    /* I terms get special packed encoding in P frames: */
    {"axisI",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32), CONDITION(PID)},
    {"axisI",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32), CONDITION(PID)},
    {"axisI",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG2_3S32), CONDITION(PID)},
    {"axisD",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(NONZERO_PID_D_0)},
    {"axisD",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(NONZERO_PID_D_1)},
    {"axisD",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(NONZERO_PID_D_2)},
    {"axisF",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(PID)},
    {"axisF",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(PID)},
    {"axisF",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(SIGNED_VB), CONDITION(PID)},
    /* rcCommands are encoded together as a group in P-frames: */
    {"rcCommand",   0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16), CONDITION(RC_COMMANDS)},
    {"rcCommand",   1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16), CONDITION(RC_COMMANDS)},
    {"rcCommand",   2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16), CONDITION(RC_COMMANDS)},
    {"rcCommand",   3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16), CONDITION(RC_COMMANDS)},

    // setpoint - define 4 fields like rcCommand to use the same encoding. setpoint[4] contains the mixer throttle
    {"setpoint",    0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16), CONDITION(SETPOINT)},
    {"setpoint",    1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16), CONDITION(SETPOINT)},
    {"setpoint",    2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16), CONDITION(SETPOINT)},
    {"setpoint",    3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_4S16), CONDITION(SETPOINT)},

    {"vbatLatest", -1, UNSIGNED, .Ipredict = PREDICT(VBATREF), .Iencode = ENCODING(NEG_14BIT), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(TAG8_8SVB), CONDITION(VBAT)},
    {"amperageLatest",-1, SIGNED,   .Ipredict = PREDICT(0),        .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(PREVIOUS),  .Pencode = ENCODING(TAG8_8SVB), CONDITION(AMPERAGE_ADC)},


#ifdef USE_MAG
    {"magADC", 0, SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(TAG8_8SVB), CONDITION(MAG)},
    {"magADC", 1, SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(TAG8_8SVB), CONDITION(MAG)},
    {"magADC", 2, SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(TAG8_8SVB), CONDITION(MAG)},
#endif
#ifdef USE_BARO
    {"baroAlt", -1, SIGNED, .Ipredict = PREDICT(0), .Iencode = ENCODING(SIGNED_VB), .Ppredict = PREDICT(PREVIOUS), .Pencode = ENCODING(TAG8_8SVB), CONDITION(BARO)},
#endif

    {"rssi",       -1, UNSIGNED, .Ipredict = PREDICT(0),       .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(PREVIOUS),      .Pencode = ENCODING(TAG8_8SVB), CONDITION(RSSI)},

    /* Gyros and accelerometers base their P-predictions on the average of the previous 2 frames to reduce noise impact */
    {"gyroADC",     0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(GYRO)},
    {"gyroADC",     1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(GYRO)},
    {"gyroADC",     2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(GYRO)},
    {"accSmooth",   0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ACC)},
    {"accSmooth",   1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ACC)},
    {"accSmooth",   2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(ACC)},
    {"debug",       0, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(DEBUG_LOG)},
    {"debug",       1, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(DEBUG_LOG)},
    {"debug",       2, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(DEBUG_LOG)},
    {"debug",       3, SIGNED,   .Ipredict = PREDICT(0),       .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(DEBUG_LOG)},
    /* Motors only rarely drops under minthrottle (when stick falls below mincommand), so predict minthrottle for it and use *unsigned* encoding (which is large for negative numbers but more compact for positive ones): */
    {"motor",       0, UNSIGNED, .Ipredict = PREDICT(MINMOTOR), .Iencode = ENCODING(UNSIGNED_VB), .Ppredict = PREDICT(AVERAGE_2), .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_1)},
    /* Subsequent motors base their I-frame values on the first one, P-frame values on the average of last two frames: */
    {"motor",       1, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_2)},
    {"motor",       2, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_3)},
    {"motor",       3, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_4)},
    {"motor",       4, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_5)},
    {"motor",       5, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_6)},
    {"motor",       6, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_7)},
    {"motor",       7, UNSIGNED, .Ipredict = PREDICT(MOTOR_0), .Iencode = ENCODING(SIGNED_VB),   .Ppredict = PREDICT(AVERAGE_2),     .Pencode = ENCODING(SIGNED_VB), CONDITION(AT_LEAST_MOTORS_8)},
};

// Rarely-updated fields
static const blackboxSimpleFieldDefinition_t blackboxSlowFields[] = {
    {"flightModeFlags",       -1, UNSIGNED, PREDICT(0),      ENCODING(UNSIGNED_VB)},
    {"stateFlags",            -1, UNSIGNED, PREDICT(0),      ENCODING(UNSIGNED_VB)},

    {"failsafePhase",         -1, UNSIGNED, PREDICT(0),      ENCODING(TAG2_3S32)},
    {"rxSignalReceived",      -1, UNSIGNED, PREDICT(0),      ENCODING(TAG2_3S32)},
    {"rxFlightChannelsValid", -1, UNSIGNED, PREDICT(0),      ENCODING(TAG2_3S32)}
};



/**
 * Write the given event to the log immediately
 */
void blackboxLogEvent(FlightLogEvent event, flightLogEventData_t* data)
{
    // Only allow events to be logged after headers have been written
    if (!(blackboxState == BLACKBOX_STATE_RUNNING || blackboxState == BLACKBOX_STATE_PAUSED))
    {
        return;
    }

    // Shared header for event frames
    blackboxWrite('E');
    blackboxWrite(event);

    // Now serialize the data for this specific frame type
    switch (event)
    {
    case FLIGHT_LOG_EVENT_SYNC_BEEP:
        blackboxWriteUnsignedVB(data->syncBeep.time);
        break;
    case FLIGHT_LOG_EVENT_FLIGHTMODE: // New flightmode flags write
        blackboxWriteUnsignedVB(data->flightMode.flags);
        blackboxWriteUnsignedVB(data->flightMode.lastFlags);
        break;
    case FLIGHT_LOG_EVENT_DISARM:
        blackboxWriteUnsignedVB(data->disarm.reason);
        break;
    case FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT:
        if (data->inflightAdjustment.floatFlag)
        {
            blackboxWrite(data->inflightAdjustment.adjustmentFunction + FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT_FUNCTION_FLOAT_VALUE_FLAG);
            blackboxWriteFloat(data->inflightAdjustment.newFloatValue);
        }
        else
        {
            blackboxWrite(data->inflightAdjustment.adjustmentFunction);
            blackboxWriteSignedVB(data->inflightAdjustment.newValue);
        }
        break;
    case FLIGHT_LOG_EVENT_LOGGING_RESUME:
        blackboxWriteUnsignedVB(data->loggingResume.logIteration);
        blackboxWriteUnsignedVB(data->loggingResume.currentTime);
        break;
    case FLIGHT_LOG_EVENT_LOG_END:
        blackboxWriteString("End of log");
        blackboxWrite(0);
        break;
    default:
        break;
    }
}

static bool testBlackboxConditionUncached(FlightLogFieldCondition condition)
{
    switch (condition) {
    case CONDITION(ALWAYS):
        return true;

    case CONDITION(AT_LEAST_MOTORS_1):
    case CONDITION(AT_LEAST_MOTORS_2):
    case CONDITION(AT_LEAST_MOTORS_3):
    case CONDITION(AT_LEAST_MOTORS_4):
    case CONDITION(AT_LEAST_MOTORS_5):
    case CONDITION(AT_LEAST_MOTORS_6):
    case CONDITION(AT_LEAST_MOTORS_7):
    case CONDITION(AT_LEAST_MOTORS_8):
        return (4 >= condition - FLIGHT_LOG_FIELD_CONDITION_AT_LEAST_MOTORS_1 + 1) && isFieldEnabled(FIELD_SELECT(MOTOR));

    case CONDITION(TRICOPTER):
        return false;
        // return (mixerConfig()->mixerMode == MIXER_TRI || mixerConfig()->mixerMode == MIXER_CUSTOM_TRI) && isFieldEnabled(FIELD_SELECT(MOTOR));

    case CONDITION(PID):
        return isFieldEnabled(FIELD_SELECT(PID));

    case CONDITION(NONZERO_PID_D_0):
        return R_PIDF.D != 0 && isFieldEnabled(FIELD_SELECT(PID));
    case CONDITION(NONZERO_PID_D_1):
        return P_PIDF.D != 0 && isFieldEnabled(FIELD_SELECT(PID));
    case CONDITION(NONZERO_PID_D_2):
        return Y_PIDF.D != 0 && isFieldEnabled(FIELD_SELECT(PID));
        // return (currentPidProfile->pid[condition - FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0].D != 0) && isFieldEnabled(FIELD_SELECT(PID));

    case CONDITION(RC_COMMANDS):
        return isFieldEnabled(FIELD_SELECT(RC_COMMANDS));

    case CONDITION(SETPOINT):
        return isFieldEnabled(FIELD_SELECT(SETPOINT));

    case CONDITION(MAG):
#ifdef USE_MAG
        return sensors(SENSOR_MAG) && isFieldEnabled(FIELD_SELECT(MAG));
#else
        return false;
#endif

    case CONDITION(BARO):
#ifdef USE_BARO
        return  isFieldEnabled(FIELD_SELECT(ALTITUDE));
#else
        return false;
#endif

    case CONDITION(VBAT):
        return isFieldEnabled(FIELD_SELECT(BATTERY));
        // return (batteryConfig()->voltageMeterSource != VOLTAGE_METER_NONE) && isFieldEnabled(FIELD_SELECT(BATTERY));

    case CONDITION(AMPERAGE_ADC):
        return false;
        // return (batteryConfig()->currentMeterSource != CURRENT_METER_NONE) && (batteryConfig()->currentMeterSource != CURRENT_METER_VIRTUAL) && isFieldEnabled(FIELD_SELECT(BATTERY));

    case CONDITION(RSSI):
        return isFieldEnabled(FIELD_SELECT(RSSI));
        // return isRssiConfigured() && isFieldEnabled(FIELD_SELECT(RSSI));

    case FLIGHT_LOG_FIELD_CONDITION_NOT_LOGGING_EVERY_FRAME:
        return blackboxPInterval != blackboxIInterval;

    case CONDITION(GYRO):
        return isFieldEnabled(FIELD_SELECT(GYRO));

    case CONDITION(ACC):
        return isFieldEnabled(FIELD_SELECT(ACC));
        // return sensors(SENSOR_ACC) && isFieldEnabled(FIELD_SELECT(ACC));

    case CONDITION(DEBUG_LOG):
        return isFieldEnabled(FIELD_SELECT(DEBUG_LOG));
        // return (debugMode != DEBUG_NONE) && isFieldEnabled(FIELD_SELECT(DEBUG_LOG));

    case CONDITION(NEVER):
        return false;

    default:
        return false;
    }
}

static void blackboxBuildConditionCache(void)
{
    blackboxConditionCache = 0;
    for (FlightLogFieldCondition cond = FLIGHT_LOG_FIELD_CONDITION_FIRST; cond <= FLIGHT_LOG_FIELD_CONDITION_LAST; cond++)
    {
        if (testBlackboxConditionUncached(cond))
        {
            blackboxConditionCache |= 1 << cond;
        }
    }
}

static bool testBlackboxCondition(FlightLogFieldCondition condition)
{
    return (blackboxConditionCache & (1 << condition)) != 0;
}


static bool isFieldEnabled(FlightLogFieldSelect_e field)
{
    return (blackbox_config.fields_disabled_mask & (1 << field)) == 0;
}


static void blackboxResetIterationTimers(void)
{
    blackboxIteration = 0;
    blackboxLoopIndex = 0;
    blackboxIFrameIndex = 0;
    blackboxPFrameIndex = 0;
    blackboxSlowFrameIterationTimer = 0;
}



/**
 * Fill the current state of the blackbox using values read from the flight controller
 */
static void loadMainState(timeUs_t currentTimeUs)
{
    blackboxMainState_t* blackboxCurrent = blackboxHistory[0];

    blackboxCurrent->time = currentTimeUs;

    for (int i = 0; i < 3; i++)
    {
        blackboxCurrent->axisPID_P[i] = lrintf(corr_att[i].P);
        blackboxCurrent->axisPID_I[i] = lrintf(corr_att[i].I);
        blackboxCurrent->axisPID_D[i] = lrintf(corr_att[i].D);
        blackboxCurrent->axisPID_F[i] = lrintf(corr_att[i].F);

        blackboxCurrent->gyroADC[i] = Gyro_Acc[i];
        blackboxCurrent->accADC[i] = Gyro_Acc[i + 3] / ACC_TO_GRAVITY;

#ifdef USE_MAG
        blackboxCurrent->magADC[i] = lrintf(mag.magADC[i]);
#endif
    }

    blackboxCurrent->rcCommand[0] = receiver.channels[0] - 1500;
    blackboxCurrent->rcCommand[1] = receiver.channels[1] - 1500;
    blackboxCurrent->rcCommand[2] = receiver.channels[3] - 1500;
    blackboxCurrent->rcCommand[3] = receiver.channels[2];

    // log the currentPidSetpoint values applied to the PID controller
    blackboxCurrent->setpoint[0] = lrintf(corr_sum.roll);
    blackboxCurrent->setpoint[1] = lrintf(corr_sum.pitch);
    blackboxCurrent->setpoint[2] = lrintf(corr_sum.yaw);
    // log the final throttle value used in the mixer
    blackboxCurrent->setpoint[3] = (throttle);

    // debuging 
#if defined(BLACKBOX_DEBUG_BARO)
    blackboxCurrent->debug[0] = lrintf(corr_acc_throttle.P);
    blackboxCurrent->debug[1] = lrintf(debug_variable[1]);
    blackboxCurrent->debug[2] = lrint(debug_variable[0] * 100);
    blackboxCurrent->debug[3] = lrintf(baro_1.ver_vel * 100);
#elif defined(BLACKBOX_DEBUG_CHANNELS_RAW)
    blackboxCurrent->debug[0] = (int16_t)(receiver.channels_raw[0] - 1500);
    blackboxCurrent->debug[1] = (int16_t)(receiver.channels_raw[1] - 1500);
    blackboxCurrent->debug[2] = (int16_t)(receiver.channels_raw[3] - 1500);
    blackboxCurrent->debug[3] = (int16_t)(receiver.channels_raw[2]);
#elif defined(BLACKBOX_DEBUG_GYRO_RAW)
    // raw data only teansformed into body frame:
    threef_t temp = { GYRO_TO_DPS * gyro_1.raw_data[0], GYRO_TO_DPS * gyro_1.raw_data[1],GYRO_TO_DPS * gyro_1.raw_data[2] };
    // transform raw data from sensor frame into drone frame:
    temp = quaternion_rotate_vector(temp, q_trans_sensor_to_body_frame);
    blackboxCurrent->debug[0] = temp.roll;
    blackboxCurrent->debug[1] = temp.pitch;
    blackboxCurrent->debug[2] = temp.yaw;
    blackboxCurrent->debug[3] = 0;
#elif defined(BLACKBOX_DEBUG_ACC_RAW)
        // raw data (no offsets, no scaling only transform into body frame):
    threef_t temp = { acc_1.raw_data[0],acc_1.raw_data[1],acc_1.raw_data[2] };
    // trnansform raw data from sensor frame into drone frame:
    temp = quaternion_rotate_vector(temp, q_trans_sensor_to_body_frame);
    blackboxCurrent->debug[0] = temp.roll;
    blackboxCurrent->debug[1] = temp.pitch;
    blackboxCurrent->debug[2] = temp.yaw;
    blackboxCurrent->debug[3] = 0;
#elif defined(BLACKBOX_DEBUG_RPM_ERR)
    blackboxCurrent->debug[0] = motors_error[0] * 1000;
    blackboxCurrent->debug[1] = motors_error[1] * 1000;
    blackboxCurrent->debug[2] = motors_error[2] * 1000;
    blackboxCurrent->debug[3] = motors_error[3] * 1000;
#elif defined(BLACKBOX_DEBUG_ATTITIUDE)
    blackboxCurrent->debug[0] = global_euler_angles.roll * 100;
    blackboxCurrent->debug[1] = global_euler_angles.pitch * 100;
    blackboxCurrent->debug[2] = global_euler_angles.yaw * 100;
    blackboxCurrent->debug[3] = 0;
#endif

    const int motorCount = MOTORS_COUNT;
    for (int i = 0; i < motorCount; i++)
    {
        blackboxCurrent->motor[i] = (motor_value[i] / 2);
    }

    blackboxCurrent->vbatLatest = battery_get_voltage() * 100;
    blackboxCurrent->amperageLatest = 0;

#ifdef USE_BARO
    blackboxCurrent->baroAlt = baro_1.altitude * 100;
#endif

    blackboxCurrent->rssi = 0;

}

/**
 * Transmit the header information for the given field definitions. Transmitted header lines look like:
 *
 * H Field I name:a,b,c
 * H Field I predictor:0,1,2
 *
 * For all header types, provide a "mainFrameChar" which is the name for the field and will be used to refer to it in the
 * header (e.g. P, I etc). For blackboxDeltaField_t fields, also provide deltaFrameChar, otherwise set this to zero.
 *
 * Provide an array 'conditions' of FlightLogFieldCondition enums if you want these conditions to decide whether a field
 * should be included or not. Otherwise provide NULL for this parameter and NULL for secondCondition.
 *
 * Set xmitState.headerIndex to 0 and xmitState.u.fieldIndex to -1 before calling for the first time.
 *
 * secondFieldDefinition and secondCondition element pointers need to be provided in order to compute the stride of the
 * fieldDefinition and secondCondition arrays.
 *
 * Returns true if there is still header left to transmit (so call again to continue transmission).
 */
static bool sendFieldDefinition(char mainFrameChar, char deltaFrameChar, const void* fieldDefinitions,
    const void* secondFieldDefinition, int fieldCount, const uint8_t* conditions, const uint8_t* secondCondition)
{
    const blackboxFieldDefinition_t* def;
    unsigned int headerCount;
    static bool needComma = false;
    size_t definitionStride = (char*)secondFieldDefinition - (char*)fieldDefinitions;
    size_t conditionsStride = (char*)secondCondition - (char*)conditions;

    if (deltaFrameChar) {
        headerCount = BLACKBOX_DELTA_FIELD_HEADER_COUNT;
    }
    else {
        headerCount = BLACKBOX_SIMPLE_FIELD_HEADER_COUNT;
    }

    /*
     * We're chunking up the header data so we don't exceed our datarate. So we'll be called multiple times to transmit
     * the whole header.
     */

     // On our first call we need to print the name of the header and a colon
    if (xmitState.u.fieldIndex == -1) {
        if (xmitState.headerIndex >= headerCount) {
            return false; //Someone probably called us again after we had already completed transmission
        }

        uint32_t charsToBeWritten = strlen("H Field x :") + strlen(blackboxFieldHeaderNames[xmitState.headerIndex]);

        if (blackboxDeviceReserveBufferSpace(charsToBeWritten) != BLACKBOX_RESERVE_SUCCESS) {
            return true; // Try again later
        }

        blackboxHeaderBudget -= blackboxPrintf("H Field %c %s:", xmitState.headerIndex >= BLACKBOX_SIMPLE_FIELD_HEADER_COUNT ? deltaFrameChar : mainFrameChar, blackboxFieldHeaderNames[xmitState.headerIndex]);

        xmitState.u.fieldIndex++;
        needComma = false;
    }

    // The longest we expect an integer to be as a string:
    const uint32_t LONGEST_INTEGER_STRLEN = 2;

    for (; xmitState.u.fieldIndex < fieldCount; xmitState.u.fieldIndex++) {
        def = (const blackboxFieldDefinition_t*)((const char*)fieldDefinitions + definitionStride * xmitState.u.fieldIndex);

        if (!conditions || testBlackboxCondition(conditions[conditionsStride * xmitState.u.fieldIndex])) {
            // First (over)estimate the length of the string we want to print

            int32_t bytesToWrite = 1; // Leading comma

            // The first header is a field name
            if (xmitState.headerIndex == 0) {
                bytesToWrite += strlen(def->name) + strlen("[]") + LONGEST_INTEGER_STRLEN;
            }
            else {
                //The other headers are integers
                bytesToWrite += LONGEST_INTEGER_STRLEN;
            }

            // Now perform the write if the buffer is large enough
            if (blackboxDeviceReserveBufferSpace(bytesToWrite) != BLACKBOX_RESERVE_SUCCESS) {
                // Ran out of space!
                return true;
            }

            blackboxHeaderBudget -= bytesToWrite;

            if (needComma) {
                blackboxWrite(',');
            }
            else {
                needComma = true;
            }

            // The first header is a field name
            if (xmitState.headerIndex == 0) {
                blackboxWriteString(def->name);

                // Do we need to print an index in brackets after the name?
                if (def->fieldNameIndex != -1) {
                    blackboxPrintf("[%d]", def->fieldNameIndex);
                }
            }
            else {
                //The other headers are integers
                blackboxPrintf("%d", def->arr[xmitState.headerIndex - 1]);
            }
        }
    }

    // Did we complete this line?
    if (xmitState.u.fieldIndex == fieldCount && blackboxDeviceReserveBufferSpace(1) == BLACKBOX_RESERVE_SUCCESS) {
        blackboxHeaderBudget--;
        blackboxWrite('\n');
        xmitState.headerIndex++;
        xmitState.u.fieldIndex = -1;
    }

    return xmitState.headerIndex < headerCount;
}
static bool blackboxShouldLogPFrame(void)
{
    return blackboxPFrameIndex == 0 && blackboxPInterval != 0;
}

static bool blackboxShouldLogIFrame(void)
{
    return blackboxLoopIndex == 0;
}

static bool blackboxIsOnlyLoggingIntraframes(void)
{
    return blackboxPInterval == 0;
}

/**
 * Load rarely-changing values from the FC into the given structure
 */
static void loadSlowState(blackboxSlowState_t* slow)
{
    slow->flightModeFlags = flight_mode == FLIGHT_MODE_STABLE ? 0b10 : 0b01;
    slow->stateFlags = 0;
    slow->failsafePhase = 0;
    slow->rxSignalReceived = true;
    slow->rxFlightChannelsValid = true;
}

/**
 * If the data in the slow frame has changed, log a slow frame.
 *
 * If allowPeriodicWrite is true, the frame is also logged if it has been more than blackboxSInterval logging iterations
 * since the field was last logged.
 */
static bool writeSlowFrameIfNeeded(void)
{
    // Write the slow frame peridocially so it can be recovered if we ever lose sync
    bool shouldWrite = blackboxSlowFrameIterationTimer >= blackboxSInterval;

    if (shouldWrite)
    {
        loadSlowState(&slowHistory);
    }
    else
    {
        blackboxSlowState_t newSlowState;

        loadSlowState(&newSlowState);

        // Only write a slow frame if it was different from the previous state
        if (memcmp(&newSlowState, &slowHistory, sizeof(slowHistory)) != 0)
        {
            // Use the new state as our new history
            memcpy(&slowHistory, &newSlowState, sizeof(slowHistory));
            shouldWrite = true;
        }
    }

    if (shouldWrite)
    {
        writeSlowFrame();
    }
    return shouldWrite;
}


// Called once every FC loop in order to log the current state
static void blackboxLogIteration(timeUs_t currentTimeUs)
{
    // Write a keyframe every blackboxIInterval frames so we can resynchronise upon missing frames
    if (blackboxShouldLogIFrame())
    {
        /*
         * Don't log a slow frame if the slow data didn't change ("I" frames are already large enough without adding
         * an additional item to write at the same time). Unless we're *only* logging "I" frames, then we have no choice.
         */
        if (blackboxIsOnlyLoggingIntraframes())
        {
            writeSlowFrameIfNeeded();
        }

        loadMainState(currentTimeUs);
        writeIntraframe();
    }
    else
    {
        // blackboxCheckAndLogArmingBeep();
        blackboxCheckAndLogFlightMode(); // Check for FlightMode status change event

        if (blackboxShouldLogPFrame())
        {
            /*
             * We assume that slow frames are only interesting in that they aid the interpretation of the main data stream.
             * So only log slow frames during loop iterations where we log a main frame.
             */
            writeSlowFrameIfNeeded();

            loadMainState(currentTimeUs);
            writeInterframe();
        }
    }

}


/* monitor the flight mode event status and trigger an event record if the state changes */
static void blackboxCheckAndLogFlightMode(void)
{
    // Use != so that we can still detect a change if the counter wraps
    if (flight_mode != blackboxLastFlightModeFlags)
    {
        flightLogEvent_flightMode_t eventData; // Add new data for current flight mode flags
        eventData.lastFlags = blackboxLastFlightModeFlags;
        blackboxLastFlightModeFlags = flight_mode;
        eventData.flags = flight_mode;
        blackboxLogEvent(FLIGHT_LOG_EVENT_FLIGHTMODE, (flightLogEventData_t*)&eventData);
    }
}

// Called once every FC loop in order to keep track of how many FC loop iterations have passed
static void blackboxAdvanceIterationTimers(void)
{
    ++blackboxSlowFrameIterationTimer;
    ++blackboxIteration;

    if (++blackboxLoopIndex >= blackboxIInterval)
    {
        blackboxLoopIndex = 0;
        blackboxIFrameIndex++;
        blackboxPFrameIndex = 0;
    }
    else if (++blackboxPFrameIndex >= blackboxPInterval)
    {
        blackboxPFrameIndex = 0;
    }
}

/**
 * Call each flight loop iteration to perform blackbox logging.
 */
void blackbox_update(timeUs_t currentTimeUs)
{
    static BlackboxState cacheFlushNextState;

    switch (blackboxState)
    {
    case BLACKBOX_STATE_STOPPED:
        blackboxDeviceOpen();
        if (Blackbox_status == BLACKBOX_COLLECTING_DATA) {
            blackboxStart();
            blackboxSetState(BLACKBOX_STATE_SEND_HEADER);
        }
        break;
    case BLACKBOX_STATE_SEND_HEADER:

        blackboxReplenishHeaderBudget();
        //On entry of this state, xmitState.headerIndex is 0 and startTime is intialised

        /*
         * Once the UART has had time to init, transmit the header in chunks so we don't overflow its transmit
         * buffer, overflow the OpenLog's buffer, or keep the main loop busy for too long.
         */
        if (currentTimeUs > xmitState.u.startTime + 100000) {
            if (blackboxDeviceReserveBufferSpace(BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION) == BLACKBOX_RESERVE_SUCCESS) {
                for (int i = 0; i < BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION && blackboxHeader[xmitState.headerIndex] != '\0'; i++, xmitState.headerIndex++) {
                    blackboxWrite(blackboxHeader[xmitState.headerIndex]);
                    blackboxHeaderBudget--;
                }
                if (blackboxHeader[xmitState.headerIndex] == '\0') {
                    blackboxSetState(BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER);
                }
            }
        }

        break;
    case BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER:
        blackboxReplenishHeaderBudget();
        //On entry of this state, xmitState.headerIndex is 0 and xmitState.u.fieldIndex is -1
        if (!sendFieldDefinition('I', 'P', blackboxMainFields, blackboxMainFields + 1, ARRAYLEN(blackboxMainFields),
            &blackboxMainFields[0].condition, &blackboxMainFields[1].condition)) {
            blackboxSetState(BLACKBOX_STATE_SEND_SLOW_HEADER);
        }
        break;

    case BLACKBOX_STATE_SEND_SLOW_HEADER:
        blackboxReplenishHeaderBudget();
        //On entry of this state, xmitState.headerIndex is 0 and xmitState.u.fieldIndex is -1
        if (!sendFieldDefinition('S', 0, blackboxSlowFields, blackboxSlowFields + 1, ARRAYLEN(blackboxSlowFields),
            NULL, NULL)) {
            // cacheFlushNextState = BLACKBOX_STATE_SEND_SYSINFO; // no need
            blackboxSetState(BLACKBOX_STATE_SEND_SYSINFO);
        }
        break;
    case BLACKBOX_STATE_SEND_SYSINFO:
        blackboxReplenishHeaderBudget();
        //On entry of this state, xmitState.headerIndex is 0

        //Keep writing chunks of the system info headers until it returns true to signal completion
        if (blackboxWriteSysinfo()) {
            blackboxSetState(BLACKBOX_STATE_RUNNING);
        }
        break;
    case BLACKBOX_STATE_CACHE_FLUSH:
        // Flush the cache and wait until all possible entries have been written to the media
        flash_flush();
        blackboxSetState(cacheFlushNextState);
        break;

        break;
    case BLACKBOX_STATE_RUNNING:
        // On entry to this state, blackboxIteration, blackboxPFrameIndex and blackboxIFrameIndex are reset to 0
        if (Blackbox_status != BLACKBOX_IDLE) {
            if (Blackbox_status == BLACKBOX_SUSPENDED || Blackbox_status == BLACKBOX_STOPPED) {
                blackboxSetState(BLACKBOX_STATE_PAUSED);
            }
            else {
                blackboxLogIteration(currentTimeUs);
            }
            blackboxAdvanceIterationTimers();
        }
        else {
            blackboxResetIterationTimers();
        }
        break;

    case BLACKBOX_STATE_PAUSED:
        // Only allow resume to occur during an I-frame iteration, so that we have an "I" base to work from
        if (Blackbox_status == BLACKBOX_COLLECTING_DATA && blackboxShouldLogIFrame())
        {
            // Write a log entry so the decoder is aware that our large time/iteration skip is intended
            flightLogEvent_loggingResume_t resume;

            resume.logIteration = blackboxIteration;
            resume.currentTime = currentTimeUs;

            blackboxLogEvent(FLIGHT_LOG_EVENT_LOGGING_RESUME, (flightLogEventData_t*)&resume);
            blackboxSetState(BLACKBOX_STATE_RUNNING);

            blackboxLogIteration(currentTimeUs);
        }
        else if (Blackbox_status == BLACKBOX_STOPPED) {
            blackboxSetState(BLACKBOX_STATE_SHUTTING_DOWN);
        }
        // Keep the logging timers ticking so our log iteration continues to advance
        blackboxAdvanceIterationTimers();
        break;
    case BLACKBOX_STATE_SHUTTING_DOWN:
        if (Arming_status == DISARMED && Blackbox_status != BLACKBOX_IDLE) {
            if (blackbox_end_partition()) {
                Blackbox_status = BLACKBOX_IDLE;
                blackboxSetState(BLACKBOX_STATE_STOPPED);
            }
        }
        break;

    default:
        break;
    }
}
/**
 * Call once every loop iteration in order to maintain the global blackboxHeaderBudget with the number of bytes we can
 * transmit this iteration.
 */
void blackboxReplenishHeaderBudget(void)
{
    int32_t freeSpace = flash_get_write_buffer_free_space();

    blackboxHeaderBudget = MIN(MIN(freeSpace, blackboxHeaderBudget + blackboxMaxHeaderBytesPerIteration), BLACKBOX_MAX_ACCUMULATED_HEADER_BUDGET);
}


/**
 * You must call this function before attempting to write Blackbox header bytes to ensure that the write will not
 * cause buffers to overflow. The number of bytes you can write is capped by the blackboxHeaderBudget. Calling this
 * reservation function doesn't decrease blackboxHeaderBudget, so you must manually decrement that variable by the
 * number of bytes you actually wrote.
 *
 * When the Blackbox device is FlashFS, a successful return code guarantees that no data will be lost if you write that
 * many bytes to the device (i.e. FlashFS's buffers won't overflow).
 *
 * When the device is a serial port, a successful return code guarantees that Cleanflight's serial Tx buffer will not
 * overflow, and the outgoing bandwidth is likely to be small enough to give the OpenLog time to absorb MicroSD card
 * latency. However the OpenLog could still end up silently dropping data.
 *
 * Returns:
 *  BLACKBOX_RESERVE_SUCCESS - Upon success
 *  BLACKBOX_RESERVE_TEMPORARY_FAILURE - The buffer is currently too full to service the request, try again later
 *  BLACKBOX_RESERVE_PERMANENT_FAILURE - The buffer is too small to ever service this request
 */
blackboxBufferReserveStatus_e blackboxDeviceReserveBufferSpace(int32_t bytes)
{
    if (bytes <= blackboxHeaderBudget) {
        return BLACKBOX_RESERVE_SUCCESS;
    }


#ifdef USE_FLASHFS

    if (bytes > (int32_t)flash_get_write_buffer_size()) {
        return BLACKBOX_RESERVE_PERMANENT_FAILURE;
    }

    if (bytes > (int32_t)flash_get_write_buffer_free_space()) {
        /*
         * The write doesn't currently fit in the buffer, so try to make room for it. Our flushing here means
         * that the Blackbox header writing code doesn't have to guess about the best time to ask flashfs to
         * flush, and doesn't stall waiting for a flush that would otherwise not automatically be called.
         */
        return BLACKBOX_RESERVE_SUCCESS;
    }
    return BLACKBOX_RESERVE_TEMPORARY_FAILURE;
#endif // USE_FLASHFS
    return BLACKBOX_RESERVE_PERMANENT_FAILURE;

}

static void blackboxSetState(BlackboxState newState)
{
    // Perform initial setup required for the new state
    switch (newState)
    {
    case BLACKBOX_STATE_SEND_HEADER:
        blackboxHeaderBudget = 0;
        xmitState.headerIndex = 0;
        xmitState.u.startTime = get_Global_Time();
        break;
    case BLACKBOX_STATE_SEND_MAIN_FIELD_HEADER:
    case BLACKBOX_STATE_SEND_GPS_G_HEADER:
    case BLACKBOX_STATE_SEND_GPS_H_HEADER:
    case BLACKBOX_STATE_SEND_SLOW_HEADER:
        xmitState.headerIndex = 0;
        xmitState.u.fieldIndex = -1;
        break;
    case BLACKBOX_STATE_SEND_SYSINFO:
        xmitState.headerIndex = 0;
        break;
    case BLACKBOX_STATE_RUNNING:
        blackboxSlowFrameIterationTimer = blackboxSInterval; // Force a slow frame to be written on the first iteration
        break;
    case BLACKBOX_STATE_SHUTTING_DOWN:
        xmitState.u.startTime = get_Global_Time();
        break;
    default:;
    }
    blackboxState = newState;
}

/**
 * Transmit a portion of the system information headers. Call the first time with xmitState.headerIndex == 0. Returns
 * true iff transmission is complete, otherwise call again later to continue transmission.
 */
static bool blackboxWriteSysinfo(void)
{
    switch (xmitState.headerIndex)
    {
        BLACKBOX_PRINT_HEADER_LINE("Firmware type", "%s", "DRON");
        BLACKBOX_PRINT_HEADER_LINE("Log start datetime", "%s", "0000-01-01T00:00:00.000");
        BLACKBOX_PRINT_HEADER_LINE("Craft name", "%s", "SYMON");
        BLACKBOX_PRINT_HEADER_LINE("I interval", "%d", blackboxIInterval);
        BLACKBOX_PRINT_HEADER_LINE("P interval", "%d", blackboxPInterval);
        BLACKBOX_PRINT_HEADER_LINE("P ratio", "%d", (uint16_t)(blackboxIInterval / blackboxPInterval));
        BLACKBOX_PRINT_HEADER_LINE("minthrottle", "%d", MOTOR_OUTPUT_MIN);
        BLACKBOX_PRINT_HEADER_LINE("maxthrottle", "%d", MOTOR_OUTPUT_MAX);
        BLACKBOX_PRINT_HEADER_LINE("gyro_scale", "0x%x", castFloatBytesToInt(1.74532925199432E-08f));
        BLACKBOX_PRINT_HEADER_LINE("acc_1G", "%u", (uint16_t)(1.f / ACC_TO_GRAVITY));
        BLACKBOX_PRINT_HEADER_LINE("rates_type", "%u", 0);
        BLACKBOX_PRINT_HEADER_LINE("rc_rates", "%d,%d,%d", RATES_CENTER_RATE_R, RATES_CENTER_RATE_P, RATES_CENTER_RATE_Y);
        BLACKBOX_PRINT_HEADER_LINE("rc_expo", "%d,%d,%d", RATES_EXPO_R, RATES_EXPO_P, RATES_EXPO_Y);
        BLACKBOX_PRINT_HEADER_LINE("rate_limits", "%d,%d,%d", RATES_MAX_RATE_R, RATES_MAX_RATE_P, RATES_MAX_RATE_Y);
        BLACKBOX_PRINT_HEADER_LINE("rollPID", "%d,%d,%d", (int)R_PIDF.P, (int)R_PIDF.I, (int)R_PIDF.D);
        BLACKBOX_PRINT_HEADER_LINE("pitchPID", "%d,%d,%d", (int)P_PIDF.P, (int)P_PIDF.I, (int)P_PIDF.D);
        BLACKBOX_PRINT_HEADER_LINE("yawPID", "%d,%d,%d", (int)Y_PIDF.P, (int)Y_PIDF.I, (int)Y_PIDF.D);
        BLACKBOX_PRINT_HEADER_LINE("vbat_scale", "%u", 124);
        BLACKBOX_PRINT_HEADER_LINE("vbatcellvoltage", "%u,%u,%u", (uint16_t)(BATTERY_CELL_MIN_VOLTAGE * 100), (uint16_t)(BATTERY_CELL_WARNING_VOLTAGE * 100), (uint16_t)(BATTERY_CELL_MAX_VOLTAGE * 100));
        BLACKBOX_PRINT_HEADER_LINE("vbatref", "%u", vbatReference);
        BLACKBOX_PRINT_HEADER_LINE("looptime", "%d", TASK_PERIOD_HZ(FREQUENCY_MAIN_LOOP));
        BLACKBOX_PRINT_HEADER_LINE("debug_mode", "%d", blackbox_debug);


    default:
        return true;
    }
    xmitState.headerIndex++;

    return false;
}

/**
 * Call during system startup to initialize the blackbox.
 */
static void blackboxInit(void)
{
    blackboxResetIterationTimers();

    // an I-frame is written every 32ms
    // blackboxUpdate() is run in synchronisation with the PID loop
    // targetPidLooptime is 1000 for 1kHz loop, 500 for 2kHz loop etc, targetPidLooptime is rounded for short looptimes
    blackboxIInterval = (uint16_t)(32 * 1000 / TASK_PERIOD_HZ(FREQUENCY_MAIN_LOOP));

    blackboxPInterval = 1 << blackbox_config.sample_rate;
    if (blackboxPInterval > blackboxIInterval)
    {
        blackboxPInterval = 0; // log only I frames if logging frequency is too low
    }
    blackboxSetState(BLACKBOX_STATE_STOPPED);
    blackboxSInterval = blackboxIInterval * 256; // S-frame is written every 256*32 = 8192ms, approx every 8 seconds

}

/**
 * Start Blackbox logging if it is not already running. Intended to be called upon arming.
 */
static void blackboxStart(void)
{
    blackboxHistory[0] = &blackboxHistoryRing[0];
    blackboxHistory[1] = &blackboxHistoryRing[1];
    blackboxHistory[2] = &blackboxHistoryRing[2];

    vbatReference = battery_get_voltage() * 100;
    // No need to clear the content of blackboxHistoryRing since our first frame will be an intra which overwrites it
    blackboxBuildConditionCache();
    blackboxResetIterationTimers();

}


/**
 * Attempt to open the logging device. Returns true if successful.
 */
bool blackboxDeviceOpen(void)
{
#ifdef USE_FLASHFS
    blackboxMaxHeaderBytesPerIteration = BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION;
    return true;
#endif 
    return false;
}

static void writeInterframe(void)
{
    blackboxMainState_t* blackboxCurrent = blackboxHistory[0];
    blackboxMainState_t* blackboxLast = blackboxHistory[1];

    blackboxWrite('P');

    // No need to store iteration count since its delta is always 1

    /*
     * Since the difference between the difference between successive times will be nearly zero (due to consistent
     * looptime spacing), use second-order differences.
     */
    blackboxWriteSignedVB((int32_t)(blackboxHistory[0]->time - 2 * blackboxHistory[1]->time + blackboxHistory[2]->time));

    int32_t deltas[8];
    int32_t setpointDeltas[4];

    if (testBlackboxCondition(CONDITION(PID)))
    {

        for (int i = 0;i < 3;++i) {
            deltas[i] = blackboxCurrent->axisPID_P[i] - blackboxLast->axisPID_P[i];
        }
        blackboxWriteSignedVBArray(deltas, 3);

        /*
         * The PID I field changes very slowly, most of the time +-2, so use an encoding
         * that can pack all three fields into one byte in that situation.
         */
        for (int i = 0;i < 3;++i) {
            deltas[i] = blackboxCurrent->axisPID_I[i] - blackboxLast->axisPID_I[i];
        }
        blackboxWriteTag2_3S32(deltas);

        /*
         * The PID D term is frequently set to zero for yaw, which makes the result from the calculation
         * always zero. So don't bother recording D results when PID D terms are zero.
         */
        for (int x = 0; x < 3; x++)
        {
            if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_NONZERO_PID_D_0 + x))
            {
                blackboxWriteSignedVB(blackboxCurrent->axisPID_D[x] - blackboxLast->axisPID_D[x]);
            }
        }
        for (int i = 0;i < 3;++i) {
            deltas[i] = blackboxCurrent->axisPID_F[i] - blackboxLast->axisPID_F[i];
        }
        blackboxWriteSignedVBArray(deltas, 3);
    }
    /*
     * RC tends to stay the same or fairly small for many frames at a time, so use an encoding that
     * can pack multiple values per byte:
     */
    for (int x = 0; x < 4; x++)
    {
        deltas[x] = blackboxCurrent->rcCommand[x] - blackboxLast->rcCommand[x];
        setpointDeltas[x] = blackboxCurrent->setpoint[x] - blackboxLast->setpoint[x];
    }

    if (testBlackboxCondition(CONDITION(RC_COMMANDS)))
    {
        blackboxWriteTag8_4S16(deltas);
    }
    if (testBlackboxCondition(CONDITION(SETPOINT)))
    {
        blackboxWriteTag8_4S16(setpointDeltas);
    }

    // Check for sensors that are updated periodically (so deltas are normally zero)
    int optionalFieldCount = 0;

    if (testBlackboxCondition(CONDITION(VBAT)))
    {
        deltas[optionalFieldCount++] = (int32_t)blackboxCurrent->vbatLatest - blackboxLast->vbatLatest;
    }

    if (testBlackboxCondition(CONDITION(AMPERAGE_ADC)))
    {
        deltas[optionalFieldCount++] = blackboxCurrent->amperageLatest - blackboxLast->amperageLatest;
    }

#ifdef USE_MAG
    if (testBlackboxCondition(CONDITION(MAG)))
    {
        for (int x = 0; x < XYZ_AXIS_COUNT; x++)
        {
            deltas[optionalFieldCount++] = blackboxCurrent->magADC[x] - blackboxLast->magADC[x];
        }
    }
#endif

#ifdef USE_BARO
    if (testBlackboxCondition(CONDITION(BARO)))
    {
        deltas[optionalFieldCount++] = blackboxCurrent->baroAlt - blackboxLast->baroAlt;
    }
#endif

    blackboxWriteTag8_8SVB(deltas, optionalFieldCount);

    // Since gyros, accs and motors are noisy, base their predictions on the average of the history:
    if (testBlackboxCondition(CONDITION(GYRO)))
    {
        blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, gyroADC), 3);
    }
    if (testBlackboxCondition(CONDITION(ACC)))
    {
        blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, accADC), 3);
    }
    if (testBlackboxCondition(CONDITION(DEBUG_LOG)))
    {
        blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, debug), 4);
    }
    if (isFieldEnabled(FIELD_SELECT(MOTOR)))
    {
        blackboxWriteMainStateArrayUsingAveragePredictor(offsetof(blackboxMainState_t, motor), MOTORS_COUNT);
    }



    // Rotate our history buffers
    blackboxHistory[2] = blackboxHistory[1];
    blackboxHistory[1] = blackboxHistory[0];
    blackboxHistory[0] = ((blackboxHistory[0] - blackboxHistoryRing + 1) % 3) + blackboxHistoryRing;

    blackboxLoggedAnyFrames = true;

}

static void writeIntraframe(void)
{
    blackboxMainState_t* blackboxCurrent = blackboxHistory[0];

    blackboxWrite('I');

    blackboxWriteUnsignedVB(blackboxIteration);
    blackboxWriteUnsignedVB(blackboxCurrent->time);

    if (testBlackboxCondition(CONDITION(PID)))
    {
        blackboxWriteSignedVBArray(blackboxCurrent->axisPID_P, 3);
        blackboxWriteSignedVBArray(blackboxCurrent->axisPID_I, 3);

        // Don't bother writing the current D term if the corresponding PID setting is zero
        for (int x = 0; x < 3; x++)
        {
            if (testBlackboxCondition(CONDITION(NONZERO_PID_D_0) + x))
            {
                blackboxWriteSignedVB(blackboxCurrent->axisPID_D[x]);
            }
        }

        blackboxWriteSignedVBArray(blackboxCurrent->axisPID_F, 3);
    }

    if (testBlackboxCondition(CONDITION(RC_COMMANDS)))
    {
        // Write roll, pitch and yaw first:
        blackboxWriteSigned16VBArray(blackboxCurrent->rcCommand, 3);

        /*
         * Write the throttle separately from the rest of the RC data as it's unsigned.
         * Throttle lies in range [PWM_RANGE_MIN..PWM_RANGE_MAX]:
         */
        blackboxWriteSignedVB(blackboxCurrent->rcCommand[3]);
    }

    if (testBlackboxCondition(CONDITION(SETPOINT)))
    {
        // Write setpoint roll, pitch, yaw, and throttle
        blackboxWriteSigned16VBArray(blackboxCurrent->setpoint, 4);
    }

    if (testBlackboxCondition(CONDITION(VBAT)))
    {
        /*
         * Our voltage is expected to decrease over the course of the flight, so store our difference from
         * the reference:
         *
         * Write 14 bits even if the number is negative (which would otherwise result in 32 bits)
         */
        blackboxWriteUnsignedVB((vbatReference - blackboxCurrent->vbatLatest) & 0x3FFF);
    }

    // if (testBlackboxCondition(CONDITION(AMPERAGE_ADC)))
    // {
    //     // 12bit value directly from ADC
    //     blackboxWriteSignedVB(blackboxCurrent->amperageLatest);
    // }

#ifdef USE_MAG
    if (testBlackboxCondition(CONDITION(MAG)))
    {
        blackboxWriteSigned16VBArray(blackboxCurrent->magADC, XYZ_AXIS_COUNT);
    }
#endif

#ifdef USE_BARO
    if (testBlackboxCondition(CONDITION(BARO)))
    {
        blackboxWriteSignedVB(blackboxCurrent->baroAlt);
    }
#endif

    // if (testBlackboxCondition(CONDITION(RSSI)))
    // {
    //     blackboxWriteUnsignedVB(blackboxCurrent->rssi);
    // }

    if (testBlackboxCondition(CONDITION(GYRO)))
    {
        blackboxWriteSigned16VBArray(blackboxCurrent->gyroADC, 3);
    }

    if (testBlackboxCondition(CONDITION(ACC)))
    {
        blackboxWriteSigned16VBArray(blackboxCurrent->accADC, 3);
    }

    if (testBlackboxCondition(CONDITION(DEBUG_LOG)))
    {
        blackboxWriteSigned16VBArray(blackboxCurrent->debug, 4);

    }

    if (isFieldEnabled(FIELD_SELECT(MOTOR)))
    {
        // Motors can be below minimum output when disarmed, but that doesn't happen much
        blackboxWriteUnsignedVB(blackboxCurrent->motor[0] - MOTOR_OUTPUT_MIN);

        // Motors tend to be similar to each other so use the first motor's value as a predictor of the others
        const int motorCount = MOTORS_COUNT;
        for (int x = 1; x < motorCount; x++)
        {
            blackboxWriteSignedVB(blackboxCurrent->motor[x] - blackboxCurrent->motor[0]);
        }

        if (testBlackboxCondition(FLIGHT_LOG_FIELD_CONDITION_TRICOPTER))
        {
            // Assume the tail spends most of its time around the center
            blackboxWriteSignedVB(blackboxCurrent->servo[0] - 1500);
        }
    }



    // Rotate our history buffers:

    // The current state becomes the new "before" state
    blackboxHistory[1] = blackboxHistory[0];
    // And since we have no other history, we also use it for the "before, before" state
    blackboxHistory[2] = blackboxHistory[0];
    // And advance the current state over to a blank space ready to be filled
    blackboxHistory[0] = ((blackboxHistory[0] - blackboxHistoryRing + 1) % 3) + blackboxHistoryRing;

    blackboxLoggedAnyFrames = true;
}


/* Write the contents of the global "slowHistory" to the log as an "S" frame. Because this data is logged so
 * infrequently, delta updates are not reasonable, so we log independent frames. */
static void writeSlowFrame(void)
{
    int32_t values[3];

    blackboxWrite('S');

    blackboxWriteUnsignedVB(slowHistory.flightModeFlags);
    blackboxWriteUnsignedVB(slowHistory.stateFlags);

    /*
     * Most of the time these three values will be able to pack into one byte for us:
     */
    values[0] = slowHistory.failsafePhase;
    values[1] = slowHistory.rxSignalReceived ? 1 : 0;
    values[2] = slowHistory.rxFlightChannelsValid ? 1 : 0;
    blackboxWriteTag2_3S32(values);

    blackboxSlowFrameIterationTimer = 0;
}

static void blackboxWriteMainStateArrayUsingAveragePredictor(int arrOffsetInHistory, int count)
{
    int16_t* curr = (int16_t*)((char*)(blackboxHistory[0]) + arrOffsetInHistory);
    int16_t* prev1 = (int16_t*)((char*)(blackboxHistory[1]) + arrOffsetInHistory);
    int16_t* prev2 = (int16_t*)((char*)(blackboxHistory[2]) + arrOffsetInHistory);

    for (int i = 0; i < count; i++)
    {
        // Predictor is the average of the previous two history states
        int32_t predictor = (prev1[i] + prev2[i]) / 2;

        blackboxWriteSignedVB(curr[i] - predictor);
    }
}




// ----------------------------------END OF BETAFLIGHT BLACKBOX CODE-------------------------------------

/**
  * @brief  function for craeting log files
  * @retval none
  * @note Each power-up, a new log file is created.
  * If the last one is empty, will be overwritten.
  * Contiguous memory block is reserved and header is written to file.
  * Blackbox data will be written using low-level flash functions (see blackbox_update()).
  */

static bool find_last_log(char* path, char* name, char* prefix);
void blackbox_print_header();

static char file_name[20] = "log_*.txt";
static FIL file;
static FATFS fs;

bool blackbox_init() {

#if defined(USE_BLACKBOX)
    FRESULT res;
    res = f_mount(&fs, "", 0);
    if (res != FR_OK) {
        return false;
    }
    if (find_last_log("", file_name, "log_")) {
        // check if the last file is not empty:
        res = f_open(&file, file_name, FA_OPEN_EXISTING | FA_WRITE);
        if (res != FR_OK) {
            return false;
        }
        // if file is empty (only header), it will be overwritten:
        if (f_size(&file) > 4096) {
            // file not empty - create a next file:
            res = f_close(&file);
            if (res != FR_OK) {
                return false;
            }
            uint16_t number = atoi(&file_name[strlen("log_")]);
            number += 1;
            snprintf(file_name, sizeof file_name, "log_%d.txt", number);
            res = f_open(&file, file_name, FA_WRITE | FA_CREATE_ALWAYS);
            if (res != FR_OK) {
                return false;
            }
        }
        else {
            // delete header() maybe some changed so we want a new header), it will be written at the begining of blackbox writing

            // uint8_t buff_temp[4096];
            // memset(buff_temp, 0xFF, 4096);
            // res = f_write(&file, );
            flash_global_write_address = (file.obj.fs->database + file.obj.fs->csize * (file.obj.sclust - 2)) * FF_MAX_SS;
            res = f_close(&file);
            if (res != FR_OK) {
                return false;
            }
            W25Q128_erase(FLASH_ERASE_SECTOR_4KB, flash_global_write_address);
            blackboxInit();
            return true;
        }
    }
    else {
        // no file found so create a first one:
        snprintf(file_name, sizeof file_name, "log_1.txt");
        res = f_open(&file, file_name, FA_WRITE | FA_OPEN_ALWAYS);
        if (res != FR_OK) {
            return false;
        }
    }

    res = f_expand(&file, FF_MAX_SS, 1);
    if (res != FR_OK) {
        return false;
    }
    flash_global_write_address = (file.obj.fs->database + file.obj.fs->csize * (file.obj.sclust - 2)) * FF_MAX_SS;
    res = f_close(&file);
    if (res != FR_OK) {
        return false;
    }
    blackboxInit();

#endif
    return true;
};

/**
*@brief  function searching for file with biggest number
*@param path string for directory to search
*@param name string for file's name to search
*@param prefix string before number to compare
*@retval - "true" if file was found "false" otherwise,
*@retval - param "name" will be changed to name of matched file with the biggest number after prefix.
*@note If you want search home dir send "path" == "".
*@note Put "name" ==
*@note	"?" - to search for any character,
*@note	"???" - to search for a string in length of three characters,
*@note	"*" - to search for any string in length of zero or longer,
*@note	"????*" - to search for any string in length of four characters or longer,
*@note	"*.*" - to search for any name with or without extension (but has to have "." in name).
*/
static bool find_last_log(char* path, char* name, char* prefix) {

    FRESULT fr;     /* Return value */
    DIR dj;         /* Directory object */
    FILINFO fno;    /* File information */
    char pattern[50];	// save pattern because name will be overwritten with name of found file
    strcpy(pattern, name);
    fr = f_findfirst(&dj, &fno, path, pattern); 	/* Start to search for files */
    if (!fno.fname[0] || fr != FR_OK) {
        f_closedir(&dj);
        return false;
    }
    while (fr == FR_OK && fno.fname[0]) {         /* Repeat while an item is found */
        if (atoi(&fno.fname[strlen(prefix)]) > atoi(&name[strlen(prefix)])) {
            memcpy(name, fno.fname, strlen(fno.fname) + 1);
        }
        fr = f_findnext(&dj, &fno);               /* Search for next item */
    }
    f_closedir(&dj);
    return true;
}

/**
*@brief  function allocating contigous block in flash
*@param size number of bytes to define size of contigous block
*@retval - "true" if allocation succeed "false" otherwise,
*@retval - global variable flash writing address will be set to the begining of this block of memory.
*@note after allocating you can use low level functions for saving data to flash, and it will be for sure contigous block.
After you finished writing you can resize file (f_truncate()) and set write/read pointer to end of your data.
Otherwise file will be as big as you defined here and data after your will be as bytes in flash (probably 0xFF if you erased flash).
*/
bool blackbox_reserve_partition(uint32_t size) {
    //  nOt finished yet - don't use 
    FRESULT fr;
    fr = f_open(&file, file_name, FA_OPEN_APPEND | FA_WRITE);
    if (fr != FR_OK) {
        return false;
    }
    // to expand file, it has to be truncated (if a file is not modified it is not necessary):
    fr = f_truncate(&file);
    if (fr != FR_OK) {
        return false;
    }
    fr = f_expand(&file, size, 1);
    if (fr != FR_OK) {
        return false;
    }
    // contigous block off memory allocated. Now it is possible to acces directly with Flash functions
    // set address where you can acces data in flash:
    flash_global_write_address = (file.obj.fs->database + file.obj.fs->csize * (file.obj.sclust - 2)) * FF_MAX_SS;
    f_close(&file);

    return true;
}

bool blackbox_end_partition() {
    static uint8_t ended_succesfully;
    if (ended_succesfully == 0) {
        blackboxPrintf("E End of log\n");
        uint16_t remaining_bytes = flash_flush();
        // add padding to align data:
        for (uint16_t i = 0;i < remaining_bytes; ++i) {
            flash_save(' ');
        }
        remaining_bytes = flash_flush();
        ended_succesfully = 1;
    }
    if (ended_succesfully == 1) {
        FRESULT res = f_open(&file, file_name, FA_OPEN_APPEND | FA_WRITE);
        uint32_t offset = flash_global_write_address - (file.obj.fs->database + file.obj.fs->csize * (file.obj.sclust - 2)) * FF_MAX_SS;
        res = f_lseek(&file, offset);
        if (res != FR_OK || f_tell(&file) != offset) {
            return false;
        }
        // f_truncate(&file);
        res = f_close(&file);
        if (res != FR_OK) {
            return false;
        }
        ended_succesfully = 0;
    }
    return true;
}







