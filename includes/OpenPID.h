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

#ifndef OPEN_PID_H
#define OPEN_PID_H

#include "../../OpenAHRS_9DOF/includes/Utils.h"

/* Further strategies will be added later. */
typedef enum {
    MANUAL, // Manual mode of the anti-windup strategy (clamping method).
    AUTO,   // Automatic mode of the anti-windup strategy (conditional clamping method).
} AntiWindupMode;

typedef enum {
    ENABLED,
    DISABLED,
} LowPassFilterStatus;

typedef struct {
    float Kp;               // Gain of the proportional action.
    float Ki;               // Gain of the integral action.
    float Kd;               // Gain of the derivative action.
    float Ts;               // Sampling period.
    float Fc;               // Low-pass filter cut-off frequency of the derivative action.
    float tau;              // Low-pass filter time constant.
    float satMin;           // Minimum saturator output value.
    float satMax;           // Maximum saturator output value.
    float setpoint;         // Setpoint of the PID controller.
    float previousError;    // The previous error calculated.

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

    LowPassFilterStatus lowPassFilterStatus;    // Enable or disable low-pass filter on PID controller derivative action (enabled by default).
    bool initialized;                           // Initialization flag.
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
    .setpoint = 0.0f, \
    .previousError = 0.0f, \
    .antiWindupMode = AUTO, \
    .lowPassFilterStatus = ENABLED, \
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
// This function initializes a PID controller with all its parameters.
PID InitPID(PID *pid, const float Kp, const float Ki, const float Kd, const float Ts, const float Fc, const float satMin, const float satMax, const AntiWindupMode *const antiWindupMode, const LowPassFilterStatus *const lowPassFilterStatus, bool *error);
// This function calculates the output of the PID corrector according to its parameters and the measurement.
bool UpdatePID(PID *pid, const float measurement);



#endif
