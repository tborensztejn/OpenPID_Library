/********************************************************
* OpenPID                                               *
* $ Id: OpenPID.h v1.0 28/11/2023 22:10 t.borensztejn $ *
********************************************************/

/*
    MIT License.
    Copyright 2023 Titouan Borensztejn <borensztejn.titouan@gmail.com>

    Permission is hereby granted, free of charge, to any person obtaining a copy of this software
    and associated documentation files (the "Software"), to deal in the Software without restriction,
    including without limitation the rights to use, copy, modify, merge, publish, distribute,
    sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all copies or
    substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
    BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
    DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

ifndef OPEN_PID_H
#define OPEN_PID_H

#include "../../OpenAHRS_9DOF/includes/Utils.h"


/*
#define ROUTINE_CHECK       1   // Enables routine verification steps.
#define NO_ROUTINE_CHECK    0   // Disables routine verification steps.

typedef enum {
    NO_ERROR,
    NULL_PTR,
    ERROR_NAN,
    UNINITIALIZED,
    ALREADY_INIT,
    INVALID_VALUE,
} ErrorType;
*/

typedef struct {
    float Kp;           // Gain of the proportional action.
    float Ki;           // Gain of the integral action.
    float Kd;           // Gain of the derivative action.
    float Ts;           // Sampling period.
    float Fc;           // Low-pass filter cutoff frequency of the derivative action.
    float tau;          // Low-pass filter time constant.
    float satMin;       // Minimum saturator output value.
    float satMax;       // Maximum saturator output value.
    bool initialized;   // Initialization flag.
} PID;

#define PID_INITIALIZER { \
    .Kp = 0.0f, \
    .Ki = 0.0f, \
    .Kd = 0.0f, \
    .Ts = 0.0f, \
    .Fc = 0.0f, \
    .tau = 0.0f, \
    .satMin = 0.0f, \
    .satMax = 0.0f, \
    .initialized = false \
} \

#define CreatePID(name) PID name = PID_INITIALIZER

/*** Functions with advanced safety and error management (under development). ***/
/*
bool InitPID(PID *pid, const float Kp, const float Ki, const float Kd, const float Te, const float Fc, ErrorType *errorType);
bool ApplyPIDRoutineChecks(const PID *const pid, ErrorType *errorType);
float GetKpValue(const PID *const pid, bool *error, ErrorType *errorType, const bool check);
bool UpdatePID(const PID *const pid, const float measure, ErrorType *errorType, const bool check);
*/

bool SetKpValue(PID *pid, const float Kp);
float GetKpValue(const PID *const pid, bool *error);
bool SetKiValue(PID *pid, const float Ki);
float GetKiValue(const PID *const pid, bool *error);
bool SetKdValue(PID *pid, const float Kd);
float GetKdValue(const PID *const pid, bool *error);


PID InitPID(PID *pid, const float Kp, const float Ki, const float Kd, const float Te, const float Fc, const float satMin, const float satMax, bool *error);
bool UpdatePID(const PID *const pid, const float measure);
