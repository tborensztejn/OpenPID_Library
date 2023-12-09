#include "../inc/OpenPID.h"

#define KP          1.0f
#define KI          2.5f
#define KD          0.8f
#define TS          0.004f
#define FC          40.0f
#define OUT_MIN     100.0f
#define OUT_MAX     -100.0f

void PrintError(PID_ErrorCode error) {
    switch(error) {
        case NO_ERR:
            printf("No error.\n");
            break;
        case ERR_NULL_PTR:
            printf("Null pointer error.\n");
            break;
        case ERR_NOT_INIT:
            printf("Non-initialized error.\n");
            break;
        case ERR_NAN:
            printf("NaN value error.\n");
            break;
        case ERR_INF:
            printf("Inf value error.\n");
            break;
        case ERR_NEGATIVE:
            printf("Negative value error.\n");
            break;
        case ERR_EXCEED_FLT:
            printf("Float overflow error.\n");
            break;
        case ERR_CRITICAL:
            printf("Critical error.\n");
            break;
        default:
            printf("Unknown error code.\n");
    }
}

int main(void) {
    bool error = false;

    // Configure the clamping method.
    const AntiWindupMode antiWindupMode = AUTO_CLAMPING;        // Automatic (conditional clamping method).
    //const AntiWindupMode antiWindupMode = MANUAL_CLAMPING;    // Manual (clamping method).
    // Enable or disable low-pass filter on derivative action.
    const bool lowPassFilterStatus = ENABLED;                   // Enable low-pass filter.
    //const bool lowPassFilterStatus = DISABLED;                // Disable low-pass filter.
    PID myPID = InitPID(KP, KI, KD, TS, FC, OUT_MIN, OUT_MAX, antiWindupMode, lowPassFilterStatus, &error);

    if (error || _error) {
        PrintError(_PID_ErrorCode);

        return 1;
    }

    //return error;
    return 0;
}











































bool GetKpValue(const PID *const pid, float *Kp) {
    bool error = false;         // Initialize the error flag to false.
    //_PID_ErrorCode = NO_ERR;    // Set the error code to NO_ERR.

    // Check that the pointer is not NULL.
    if (Kp == NULL) {
        _PID_ErrorCode = ERR_NULL_PTR;  // Set the error code to ERR_NULL_PTR.
        error = true;                   // Set the error flag to true.

        return error;   // Return the state of the error flag.
    }

    // Check that the pointer is not NULL.
    if (pid == NULL) {
        // The pointer is NULL.
        _PID_ErrorCode = ERR_NULL_PTR;  // Set the error code to ERR_NULL_PTR.
        error = true;                   // Set the error flag to true.

        return error;   // Return the state of the error flag.
    }

    // Check that the PID controller is initialized.
    if (!pid->initialized) {
        // The PID controller is not initialized.
        _PID_ErrorCode = ERR_NOT_INIT;  // Set the error code to ERR_NOT_INIT.
        error = true;                   // Set the error flag to true.

        return error;   // Return the state of the error flag.
    }

    error = CheckParam(pid->Kp);    // Check that the value can be used.

    if (error) {
        return error;   // Return the state of the error flag.
    }

    *Kp = pid->Kp;
}
