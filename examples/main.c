#include "../inc/OpenPID.h"

#define KP          1.0f
#define KI          2.5f
#define KD          0.8f
#define TS          0.004f
#define FC          40.0f
#define OUT_MIN     100.0f
#define OUT_MAX     -100.0f

int main(void) {
    bool error = false;

    // Configure the clamping method.
    const AntiWindupMode antiWindupMode = AUTO_CLAMPING;        // Automatic (conditional clamping method).
    //const AntiWindupMode antiWindupMode = MANUAL_CLAMPING;    // Manual (clamping method).
    // Enable or disable low-pass filter on derivative action.
    const bool lowPassFilterStatus = ENABLED;                   // Enable low-pass filter.
    //const bool lowPassFilterStatus = DISABLED;                // Disable low-pass filter.
    PID myPID = InitPID(KP, KI, KD, TS, FC, OUT_MIN, OUT_MAX, antiWindupMode, lowPassFilterStatus, &error);

    if (error) {
        printf("Error.\n");
        printf("%d\n", _PID_ErrorCode);

        return 1;
    }

    //return error;
    return 0;
}
