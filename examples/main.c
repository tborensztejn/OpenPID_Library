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

    PID myPID = InitPID(NULL, KP, KI, KD, TS, FC, OUT_MIN, OUT_MAX, &error);
    // The code below is the same as the code above.
    /*
    CreatePID(myPID);
    InitPID(&myPID, KP, KI, KD, TS, FC, OUT_MIN, OUT_MAX, &error);
    */



    return error;
}
