/********************************************************
* OpenPID                                               *
* $ Id: OpenPID.c v1.0 28/11/2023 22:10 t.borensztejn $ *
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

#include "../inc/OpenPID.h"

/*** Declaration of function prototypes. ***/

// Add an example here.
bool SetKpValue(PID *pid, const float Kp) {
    bool error = false;  // Initialize the error flag to false.

    // Check that the PID controller is initialized.
    if (pid->initialized) {
        pid->Kp = Kp;
    } else {
        // The PID controller is not initialized.
        error = true;   // Set the error flag to true.
    }

    return error;
}

/*
// Add an example here.
float GetKpValue(const PID *const pid, bool *error) {
    float Kp;       // Declare the variable that will store the returned value.
    *error = false; // Set the error flag to false.

    // Check that the PID controller is initialized.
    if (pid->initialized) {
        Kp = pid->Kp;
    } else {
        // The PID controller is not initialized.
        *error = true;   // Set the error flag to true.
        Kp = NAN;
    }

    return Kp;
}
*/

// Add an example here.
float GetKpValue(const PID *const pid, bool *error, ErrorCode *errorCode) {
    float Kp;       // Declare the variable that will store the returned value.
    *error = false; // Set the error flag to false.

    // Check that the PID controller is initialized.
    if (pid == NULL) {
        // The pointer is NULL.
        
    }

    // Check that the PID controller is initialized.
    if (pid->initialized) {
        Kp = pid->Kp;
    } else {
        // The PID controller is not initialized.
        *error = true;   // Set the error flag to true.
        Kp = NAN;
    }

    return Kp;
}

// Add an example here.
bool SetKiValue(PID *pid, const float Ki) {
    bool error = false;  // Initialize the error flag to false.

    // Check that the PID controller is initialized.
    if (pid->initialized) {
        pid->Ki = Ki;
    } else {
        // The PID controller is not initialized.
        error = true;   // Set the error flag to true.
    }

    return error;
}

// Add an example here.
float GetKiValue(const PID *const pid, bool *error) {
    float Ki;       // Declare the variable that will store the returned value.
    *error = false; // Set the error flag to false.

    // Check that the PID controller is initialized.
    if (pid->initialized) {
        Ki = pid->Ki;
    } else {
        // The PID controller is not initialized.
        *error = true;   // Set the error flag to true.
        Ki = NAN;
    }

    return Ki;
}

// Add an example here.
bool SetKdValue(PID *pid, const float Kd) {
    bool error = false;  // Initialize the error flag to false.

    // Check that the PID controller is initialized.
    if (pid->initialized) {
        pid->Kd = Kd;
    } else {
        // The PID controller is not initialized.
        error = true;   // Set the error flag to true.
    }

    return error;
}

// Add an example here.
float GetKdValue(const PID *const pid, bool *error) {
    float Kd;       // Declare the variable that will store the returned value.
    *error = false; // Set the error flag to false.

    // Check that the PID controller is initialized.
    if (pid->initialized) {
        Kd = pid->Kd;
    } else {
        // The PID controller is not initialized.
        *error = true;   // Set the error flag to true.
        Kd = NAN;
    }

    return Kd;
}

// Add an example here.
bool SetFcValue(PID *pid, const float Fc) {
    bool error = false;  // Initialize the error flag to false.

    // Check that the PID controller is initialized.
    if (pid->initialized) {
        // Check that the cut-off frequency is not too low (avoid NaN type errors).
        if (Fc > EPSILON) {
            pid->Fc = Fc;   // Set the Fc value.
        } else {
            // The cut-off frequency is too low.
            pid->Fc = EPSILON;  // Set the Fc value.
        }

        pid->tau = 1.0f / (2.0f * M_PI * pid->Fc);  // Set the tau value.
    } else {
        // The PID controller is not initialized.
        error = true;   // Set the error flag to true.
    }

    return error;
}

// Add an example here.
float GetFcValue(const PID *const pid, bool *error) {
    float Fc;       // Declare the variable that will store the returned value.
    *error = false; // Set the error flag to false.

    // Check that the PID controller is initialized.
    if (pid->initialized) {
        Fc = pid->Fc;
    } else {
        // The PID controller is not initialized.
        *error = true;   // Set the error flag to true.
        Fc = NAN;
    }

    return Fc;
}

// Add an example here.
float GetTauValue(const PID *const pid, bool *error) {
    float tau;       // Declare the variable that will store the returned value.
    *error = false; // Set the error flag to false.

    // Check that the PID controller is initialized.
    if (pid->initialized) {
        tau = pid->tau;
    } else {
        // The PID controller is not initialized.
        *error = true;   // Set the error flag to true.
        tau = NAN;
    }

    return tau;
}

// Add an example here.
PID InitPID(PID *pid, const float Kp, const float Ki, const float Kd, const float Ts, const float Fc, const float outSatMin, const float outSatMax, const AntiWindupMode *const antiWindupMode, const bool lowPassFilterStatus, bool *error) {
    *error = true;                  // Set the error flag to true.

    // Check for null pointer.
    if (pid == NULL) {
        // The pointer is null.
        //PID NewPID = PID_INITIALIZER;
        CreatePID(NewPID);

        NewPID.Kp = Kp;                 // Set the Kp value.
        NewPID.Ki = Ki;                 // Set the Ki value.
        NewPID.Kd = Kd;                 // Set the Kd value.
        NewPID.Ts = Ts;                 // Set the Ts value.
        NewPID.outSatMin = outSatMin;   // Set the outSatMin value.
        NewPID.outSatMax = outSatMax;   // Set the outSatMax value.

        // The integrator clamping saturator defaults to the PID controller's output saturator limits.
        NewPID.integratorSatMin = outSatMin;   // Set the integratorSatMin value.
        NewPID.integratorSatMax = outSatMax;   // Set the integratorSatMax value.

        // Check that the pointer is not null.
        // By default, the conditional clamping method is used.
        if (antiWindupMode != NULL) {
            NewPID.antiWindupMode = *antiWindupMode;    // Set the anti-windup method.
        }

        // Check that the cut-off frequency is not too low (avoid NaN type errors).
        if (Fc > EPSILON) {
            NewPID.Fc = Fc; // Set the Fc value.
        } else {
            // The cut-off frequency is too low.
            NewPID.Fc = EPSILON;    // Set the Fc value to EPSILON.
        }

        NewPID.tau = 1.0f / (2.0f * M_PI * NewPID.Fc);  // Set the tau value.

        NewPID.initialized = true;  // Set the initialization flag to true.
        *error = false;             // Set the error flag to false.

        return NewPID;  // Return the initialized new PID controller.
    }

    // Check whether the PID controller has already been initialized.
    if (pid->initialized) {
        // The PID controller has already been initialized.
        return *pid;
    }

    pid->Kp = Kp;               // Set the Kp value.
    pid->Ki = Ki;               // Set the Ki value.
    pid->Kd = Kd;               // Set the Kd value.
    pid->Ts = Ts;               // Set the Ts value.
    pid->outSatMin = outSatMin; // Set the outSatMin value.
    pid->outSatMax = outSatMax; // Set the outSatMax value.

    // The integrator clamping saturator defaults to the PID controller's output saturator limits.
    pid->integratorSatMin = outSatMin;   // Set the integratorSatMin value.
    pid->integratorSatMax = outSatMax;   // Set the integratorSatMax value.

    // Check that the pointer is not null.
    // By default, the conditional clamping method is used.
    if (antiWindupMode != NULL) {
        pid->antiWindupMode = *antiWindupMode;  // Set the anti-windup method.
    }

    // Check that the low-pass filter is enabled.
    if (!lowPassFilterStatus) {
        // The low-pass filter is disabled.
        pid->Fc = 0.0f;                                 // Set the Fc value.
        pid->tau = 0.0f;                                // Set the tau value.
        pid->lowPassFilterStatus = lowPassFilterStatus; // Set the low-pass filter status.
        pid->initialized = true;                        // Set the initialization flag to true.
        *error = false;                                 // Set the error flag to false.

        return *pid;
    }

    // Check that the cut-off frequency is not too low (avoid NaN type errors).
    if (Fc > EPSILON) {
        pid->Fc = Fc;   // Set the Fc value.
    } else {
        // The cut-off frequency is too low.
        pid->Fc = EPSILON;  // Set the Fc value.
    }

    pid->tau = 1.0f / (2.0f * M_PI * pid->Fc);      // Set the tau value.
    pid->lowPassFilterStatus = lowPassFilterStatus; // Set the low-pass filter status.
    pid->initialized = true;                        // Set the initialization flag to true.
    *error = false;                                 // Set the error flag to false.

    return *pid;
}

/*
    PID controller using manual clamping, without low-pass filter and Euler backward method.

    Pseudocode:
    error = setpoint - measurements
    sum_errors = sum_errors + error
    sum_errors = sat(sum_errors, min_sum_errors, max_sum_errors)
    delta_error = error - previous_error
    previous_error = error
    output = Kp * error + Ki + Ts * sum_errors + Kd * delta_error / Ts
    output = sat(output, min_output, max_output)
 */

bool UpdatePID(PID *pid, const float measurement) {
    bool error = false; // Initialize the error flag to false.
    float derivativeActionGain = 0.0f;

    // Check whether the PID controller is initialized.
    if (!pid->initialized) {
        // The PID controller is not initialized.
        return error;
    }

    float pidError = pid->setpoint - measurement;       // Calculate error from setpoint and measurement.
    float deltaError = pidError - pid->previousError;   // Calculate the error variation.
    pid->previousError = pidError;                      // Update previous error for next iteration.

    float proportionalActionGain = pid->Kp * pidError;  // Calculate proportionnal action gain.

    if (pid->lowPassFilterStatus) {
        // Bilinear transformation (Tustin method).
        derivativeActionGain = 2.0f * pid->Kd * deltaError / (2.0f * pid->tau + pid->Ts) + ((2.0f * pid->tau - pid->Ts) / (2.0f * pid->tau + pid->Ts)) * pid->previousDerivativeActionGain;
        pid->previousDerivativeActionGain = derivativeActionGain;
    } else {
        // Add some code here.
    }

    if (pid->antiWindupMode == AUTO_CLAMPING) {
        if (pid->conditionalIntegration) {
            // Bilinear transformation (Tustin method).
            float integralActionGain = pid->Ki * pid->Ts * (pidError + pid->previousError) / 2.0f + pid->previousIntegralActionGain;
        } else {
            // Add some code here.
        }
    } else if (pid->antiWindupMode == MANUAL_CLAMPING) {
        pid->sumErrors += pidError;
        pid->sumErrors = Sat(pid->sumErrors, pid->integratorSatMin, pid->integratorSatMax);
        // Euler backward transformation.
        pid->output = pid->Kp * pidError + pid->Ki + pid->Ts * pid->sumErrors + pid->Kd * deltaError / pid->Ts;
        pid->output = Sat(pid->output, pid->outSatMin, pid->outSatMax);
    }

    return error;
}
