#include "../includes/OpenPID.h"

#define KP          1.0f
#define KI          2.5f
#define KD          0.8f
#define TS          0.004f
#define FC          40.0f
#define OUT_MIN     100.0f
#define OUT_MAX     -100.0f

bool main(void) {
    bool error = false;

    /* Basic initialization. */
    // By default, the low-pass filter on derivative action is enabled and the conditional clamping method is used.
    //PID myPID = InitPID(NULL, KP, KI, KD, TS, FC, OUT_MIN, OUT_MAX, NULL, NULL, &error);
    // The code below is the same as the code above.
    /*
    CreatePID(myPID);
    InitPID(&myPID, KP, KI, KD, TS, FC, OUT_MIN, OUT_MAX, &error);
    */

    /* Advanced initialization. */
    // Configure the clamping method.
    const AntiWindupMode antiWindupMode = AUTO;                 // Automatic (conditional clamping method).
    //AntiWindupMode antiWindupMode = MANUAL;                   // Manual (clamping method).
    // Enable or disable low-pass filter on derivative action.
    const LowPassFilterStatus lowPassFilterStatus = ENABLED;    // Enable low-pass filter.
    //LowPassFilterStatus lowPassFilterStatus = DISABLED;       // Disable low-pass filter.
    PID myPID = InitPID(NULL, KP, KI, KD, TS, FC, OUT_MIN, OUT_MAX, &antiWindupMode, &lowPassFilterStatus, &error);
    // The code below is the same as the code above.
    /*
    CreatePID(myPID);
    InitPID(NULL, KP, KI, KD, TS, FC, OUT_MIN, OUT_MAX, &antiWindupMode, &lowPassFilterStatus, &error);
    */

    return error;
}
