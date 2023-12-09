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

// clear && gcc -std=c11 -Wall -Wextra -pedantic -lm ../../OpenAHRS_9DOF/src/Utils.c ../examples/main.c ../src/OpenPID.c -o OpenPID && ./OpenPID
// clear && git checkout main && git add --all && git commit -m "Automatic update - $(date)" && git push origin main
// cp -fvr /mnt/c/Users/boren/Desktop/IMU/OpenPID/* ./
// clear && gcc src/Linalg.c ../src/Utils.c src/Common.c src/Matrix.c src/Vector.c -o linalg -lm && ./linalg
//  make clean && make && python3 generate_raw_measurements.py && cat build/raw_measurements.csv && cd build/ && ./OpenAHRS_9DOF && cd .. && find * -type f -exec md5sum {} + | while read sum file; do md5sum_dest=$(md5sum "/home/titoune/github/OpenAHRS/$file" 2>/dev/null | awk '{print $1}'); [ "$sum" != "$md5sum_dest" ] && echo "Copy of : $file" && cp "$file" "/home/titoune/github/OpenAHRS/$file"; done
// clear && git checkout main && git add --all && git commit -m "Automatic update - $(date)" && git push origin main


#ifndef OPEN_PID_H
#define OPEN_PID_H

#include "../../OpenAHRS_9DOF/includes/Utils.h"

#define DISABLED    false
#define ENABLED     true

typedef enum {
    NO_ERR,         // No error.
    ERR_NULL_PTR,   // Null pointer error.
    ERR_NOT_INIT,   // Non-initialized error.
    ERR_NAN,        // NaN value error.
    ERR_INF,        // Inf value error.
    ERR_NEGATIVE,   // Negative value error.
    ERR_EXCEED_FLT, // Float overflow error.
    ERR_CRITICAL,   // Critical error.
} PID_ErrorCode;

extern PID_ErrorCode _PID_ErrorCode;
extern bool _criticalErrorFlag;

/* Further strategies will be added later. */
typedef enum {
    MANUAL_CLAMPING,    // Manual mode of the anti-windup strategy (clamping method).
    AUTO_CLAMPING,      // Automatic mode of the anti-windup strategy (conditional clamping method).
    //OBSERVER,         // Future implementation.
} AntiWindupMode;

typedef struct {
    float Kp;                           // Gain of the proportional action.
    float Ki;                           // Gain of the integral action.
    float Kd;                           // Gain of the derivative action.
    float Ts;                           // Sampling period.
    float Fc;                           // Low-pass filter cut-off frequency of the derivative action.
    float tau;                          // Low-pass filter time constant.
    float outSatMin;                    // Minimum saturator output value.
    float outSatMax;                    // Maximum saturator output value.
    float integratorSatMin;             // Minimum integrator saturator output value.
    float integratorSatMax;             // Maximum integrator saturator output value.
    float setpoint;                     // Setpoint of the PID controller.
    float output;                       // The ouput calculated by the PID controller.
    float previousError;                // The previous error calculated.
    float sumErrors;                    // Sum of errors.
    float previousDerivativeActionGain; // The previous derivative action gain.
    float previousIntegralActionGain;   // The previous integral action gain.

    /*
        Note:

        During initialization, if this parameter is not specified, automatic mode will be used (conditional clamping).
        If the user wishes to use manual mode, this must be specified. Note that in manual mode, the integrator clamping
        saturator defaults to the PID controller's output saturator limits. Use of manual mode requires a good mathematical
        understanding of the subject. The choice of not being able to modify these parameters during initialization is
        voluntary. It is always possible to modify them using the dedicated functions.
    */

    AntiWindupMode antiWindupMode;  // Anti-windup strategy mode.

    /*
        Note:

        This filter attenuates the noise or rapid fluctuations that may be present in the measurement of the system output.

        The derivative action of a PID controller amplifies the high frequencies of the error signal, which can lead to
        noisy responses, particularly when sensors are measuring signals subject to rapid variations. These fluctuations
        may be due to imperfect sensors, external disturbances or other sources of noise.

        Adding a low-pass filter to the derived action attenuates the high-frequency components of the derived signal,
        reducing the potential noise introduced into the system. This can help stabilize the controller and improve its
        performance in situations where noise might otherwise lead to undesirable responses.

        However, the addition of a low-pass filter also introduces a delay in the controller's response, as it attenuates
        rapid variations. It is therefore important to find a compromise between noise reduction and system responsiveness
        when adjusting the low-pass filter parameters. The design of a PID controller with a low-pass filter often depends
        on specific system characteristics and performance requirements.

        To sum up (in a pedagogical way): this filter attenuates the gain of the derivative action at high frequencies,
        thus avoiding saturation of the output. For fast-moving dynamic systems, it is preferable to activate it,
        and to choose the cut-off frequency carefully.
    */

    bool lowPassFilterStatus;       // Enable or disable low-pass filter on PID controller derivative action (enabled by default).
    bool conditionalIntegration;    // Conditional intergration status.
    bool initialized;               // Initialization flag.
} PID;

#define PID_INITIALIZER { \
    .Kp = 0.0f, \
    .Ki = 0.0f, \
    .Kd = 0.0f, \
    .Ts = 0.0f, \
    .Fc = 0.0f, \
    .tau = 0.0f, \
    .outSatMin = 0.0f, \
    .outSatMax = 0.0f, \
    .integratorSatMin = 0.0f, \
    .integratorSatMax = 0.0f, \
    .setpoint = 0.0f, \
    .output = 0.0f, \
    .previousError = 0.0f, \
    .sumErrors = 0.0f, \
    .previousDerivativeActionGain = 0.0f, \
    .previousIntegralActionGain = 0.0f, \
    .antiWindupMode = AUTO_CLAMPING, \
    .lowPassFilterStatus = ENABLED, \
    .conditionalIntegration = true, \
    .initialized = false \
} \

/*** Declaration of macros. ***/

#define CreatePID(name) PID name = PID_INITIALIZER

/*** Declaration of function prototypes. ***/

/*
    Note:

    The choice of providing functions for modifying PID corrector gains is deliberate.
    This enables an adaptive control strategy to be implemented. The other parameters
    are deliberately made inaccessible, as modifying them during operation seems
    irrelevant and very risky. Perhaps one day this will be changed if the need arises.
*/

// This function is used to modify the gain value of the PID controller's proportional action (Kp).
bool SetKpValue(PID *pid, const float Kp);
// This function is used to retrieve the gain value of the PID controller's proportional action (Kp).
//float GetKpValue(const PID *const pid, bool *error);
float GetKpValue(const PID *const pid, bool *error);
// This function is used to modify the gain value of the PID controller's integral action (Ki).
bool SetKiValue(PID *pid, const float Ki);
// This function is used to retrieve the gain value of the PID controller's integral action (Ki).
float GetKiValue(const PID *const pid, bool *error);
// This function is used to modify the gain value of the PID controller's derivative action (Kd).
bool SetKdValue(PID *pid, const float Kd);
// This function is used to retrieve the gain value of the PID controller's derivative action (Kd).
float GetKdValue(const PID *const pid, bool *error);
// This function is used to modify the cut-off frequency of the low-pass filter (Fc).
bool SetFcValue(PID *pid, const float Fc);
// This function is used to retrieve the cut-off frequency of the low-pass filter (Fc).
float GetFcValue(const PID *const pid, bool *error);
// This function is used to retrieve the time constant of the low-pass filter (tau).
float GetTauValue(const PID *const pid, bool *error);

// Add a description here.
bool SetIntegratorSatMinValue(PID *pid, const float integratorSatMin);
// Add a description here.
bool SetIntegratorSatMaxValue(PID *pid, const float integratorSatMax);
// Add a description here.
float GetOutputValue(const PID *const pid, bool *error);

// This function initializes a PID controller with all its parameters.
PID InitPID(const float Kp, const float Ki, const float Kd, const float Ts, const float Fc, const float outSatMin, const float outSatMax, const AntiWindupMode antiWindupMode, const bool lowPassFilterStatus, bool *error);
// This function calculates the output of the PID corrector according to its parameters and the measurement.
bool UpdatePID(PID *pid, const float measurement);

#endif
