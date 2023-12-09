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

PID_ErrorCode _PID_ErrorCode;
bool _error = false;

/*** Declaration of function prototypes. ***/

// Add an example here.
bool CheckValue(const float value) {
    bool error = false;         // Initialize the error flag to false.
    //_PID_ErrorCode = NO_ERR;    // Set the error code to NO_ERR.

    // Check that the value is valid.
    if (isnan(value)) {
        // The value is not valid.
        _PID_ErrorCode = ERR_NAN;   // Set the error code to ERR_NAN.
        error = true;               // Set the error flag to true.

        return error;   // Return the state of the error flag.
    }

    // Check that the value is finite.
    if (isinf(value)) {
        // The value is infinite.
        _PID_ErrorCode = ERR_INF;   // Set the error code to ERR_INF.
        error = true;               // Set the error flag to true.

        return error;   // Return the state of the error flag.
    }

    // Check for float overflow.
    if (value > FLT_MAX) {
        // The value is too big.
        _PID_ErrorCode = ERR_EXCEED_FLT;    // Set the error code to ERR_EXCEED_FLT.
        error = true;                       // Set the error flag to true.

        return error;   // Return the state of the error flag.
    }

    // Check for float overflow.
    if (value < -FLT_MAX) {
        // The value is too big.
        _PID_ErrorCode = ERR_EXCEED_FLT;    // Set the error code to ERR_EXCEED_FLT.
        error = true;                       // Set the error flag to true.

        return error;   // Return the state of the error flag.
    }

    return error;   // Return the state of the error flag.
}

// Add an example here.
bool CheckParam(const float value) {
    bool error = false;         // Initialize the error flag to false.
    //_PID_ErrorCode = NO_ERR;    // Set the error code to NO_ERR.

    error = CheckValue(value);  // Check that the value can be used.

    if (error) {
        return error;   // Return the state of the error flag.
    }

    // Check if the value is positive.
    if (value < 0.0f) {
        // The value is negative.
        _PID_ErrorCode = ERR_NEGATIVE;  // Set the error code to ERR_NEGATIVE.
        error = true;                   // Set the error flag to true.

        return error;   // Return the state of the error flag.
    }

    return error;   // Return the state of the error flag.
}

// Add an example here.
bool SetKpValue(PID *pid, const float Kp) {
    bool error = false;         // Initialize the error flag to false.
    _PID_ErrorCode = NO_ERR;    // Set the error code to NO_ERR.

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

    error = CheckParam(Kp); // Check that the value can be used.

    if (error) {
        return error;   // Return the state of the error flag.
    }

    pid->Kp = Kp;

    return error;
}

/*
// Add an example here.
float GetKpValue(const PID *const pid, bool *error) {
    // Check that the pointer is not NULL.
    if (error == NULL) {
        _PID_ErrorCode = ERR_CRITICAL;
        _error = true;

        return 0.0f;    // Return a zero value.
    }

    *error = false;             // Set the error flag to false.
    _PID_ErrorCode = NO_ERR;    // Set the error code to NO_ERR.

    // Check that the pointer is not NULL.
    if (pid == NULL) {
        // The pointer is NULL.
        _PID_ErrorCode = ERR_NULL_PTR;  // Set the error code to ERR_NULL_PTR.
        *error = true;                  // Set the error flag to true.

        return 0.0f;    // Return a zero value.
    }

    // Check that the PID controller is initialized.
    if (!pid->initialized) {
        // The PID controller is not initialized.
        _PID_ErrorCode = ERR_NOT_INIT;  // Set the error code to ERR_NOT_INIT.
        *error = true;                  // Set the error flag to true.

        return 0.0f;    // Return a zero value.
    }

    *error = CheckParam(pid->Kp);   // Check that the value can be used.

    if (*error) {
        return 0.0f;    // Return a zero value.
    }

    return pid->Kp;
}
*/

// Add an example here.
bool SetKiValue(PID *pid, const float Ki) {
    bool error = false;         // Initialize the error flag to false.
    _PID_ErrorCode = NO_ERR;    // Set the error code to NO_ERR.

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

    error = CheckParam(Ki); // Check that the value can be used.

    if (error) {
        return error;   // Return the state of the error flag.
    }

    pid->Ki = Ki;

    return error;
}

// Add an example here.
float GetKiValue(const PID *const pid, bool *error) {
    // Check that the pointer is not NULL.
    if (error == NULL) {
        _PID_ErrorCode = ERR_CRITICAL;
        _error = true;

        return 0.0f;    // Return a zero value.
    }

    *error = false;             // Set the error flag to false.
    _PID_ErrorCode = NO_ERR;    // Set the error code to NO_ERR.

    // Check that the pointer is not NULL.
    if (pid == NULL) {
        // The pointer is NULL.
        _PID_ErrorCode = ERR_NULL_PTR;  // Set the error code to ERR_NULL_PTR.
        *error = true;                  // Set the error flag to true.

        return 0.0f;    // Return a zero value.
    }

    // Check that the PID controller is initialized.
    if (!pid->initialized) {
        // The PID controller is not initialized.
        _PID_ErrorCode = ERR_NOT_INIT;  // Set the error code to ERR_NOT_INIT.
        *error = true;                  // Set the error flag to true.

        return 0.0f;    // Return a zero value.
    }

    *error = CheckParam(pid->Ki);   // Check that the value can be used.

    if (*error) {
        return 0.0f;    // Return a zero value.
    }

    return pid->Ki;
}

// Add an example here.
bool SetKdValue(PID *pid, const float Kd) {
    bool error = false;         // Initialize the error flag to false.
    _PID_ErrorCode = NO_ERR;    // Set the error code to NO_ERR.

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

    error = CheckParam(Kd); // Check that the value can be used.

    if (error) {
        return error;   // Return the state of the error flag.
    }

    pid->Kd = Kd;

    return error;
}

// Add an example here.
float GetKdValue(const PID *const pid, bool *error) {
    // Check that the pointer is not NULL.
    if (error == NULL) {
        _PID_ErrorCode = ERR_CRITICAL;
        _error = true;

        return 0.0f;    // Return a zero value.
    }

    *error = false;             // Set the error flag to false.
    _PID_ErrorCode = NO_ERR;    // Set the error code to NO_ERR.

    // Check that the pointer is not NULL.
    if (pid == NULL) {
        // The pointer is NULL.
        _PID_ErrorCode = ERR_NULL_PTR;  // Set the error code to ERR_NULL_PTR.
        *error = true;                  // Set the error flag to true.

        return 0.0f;    // Return a zero value.
    }

    // Check that the PID controller is initialized.
    if (!pid->initialized) {
        // The PID controller is not initialized.
        _PID_ErrorCode = ERR_NOT_INIT;  // Set the error code to ERR_NOT_INIT.
        *error = true;                  // Set the error flag to true.

        return 0.0f;    // Return a zero value.
    }

    *error = CheckValue(pid->Kd);   // Check that the value can be used.

    if (*error) {
        return 0.0f;    // Return a zero value.
    }

    return pid->Kd;
}

// Add an example here.
bool SetFcValue(PID *pid, const float Fc) {
    bool error = false;         // Initialize the error flag to false.
    _PID_ErrorCode = NO_ERR;    // Set the error code to NO_ERR.

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

    error = CheckParam(Fc); // Check that the value can be used.

    if (error) {
        return error;   // Return the state of the error flag.
    }

    // Check that the cut-off frequency is not too low (avoid NaN type errors).
    if (Fc > EPSILON) {
        pid->Fc = Fc;   // Set the Fc value.
    } else {
        // The cut-off frequency is too low.
        pid->Fc = EPSILON;  // Set the Fc value.
    }

    pid->tau = 1.0f / (2.0f * M_PI * pid->Fc);  // Set the tau value.

    error = CheckParam(pid->tau); // Check that the value can be used.

    if (error) {
        pid->tau = FLT_MAX;

        return error;   // Return the state of the error flag.
    }

    return error;
}

// Add an example here.
float GetFcValue(const PID *const pid, bool *error) {
    // Check that the pointer is not NULL.
    if (error == NULL) {
        _PID_ErrorCode = ERR_CRITICAL;
        _error = true;

        return 0.0f;    // Return a zero value.
    }

    *error = false;             // Set the error flag to false.
    _PID_ErrorCode = NO_ERR;    // Set the error code to NO_ERR.

    // Check that the pointer is not NULL.
    if (pid == NULL) {
        // The pointer is NULL.
        _PID_ErrorCode = ERR_NULL_PTR;  // Set the error code to ERR_NULL_PTR.
        *error = true;                  // Set the error flag to true.

        return 0.0f;    // Return a zero value.
    }

    // Check that the PID controller is initialized.
    if (!pid->initialized) {
        // The PID controller is not initialized.
        _PID_ErrorCode = ERR_NOT_INIT;  // Set the error code to ERR_NOT_INIT.
        *error = true;                  // Set the error flag to true.

        return 0.0f;    // Return a zero value.
    }

    *error = CheckParam(pid->Fc);   // Check that the value can be used.

    if (*error) {
        return 0.0f;    // Return a zero value.
    }

    return pid->Fc;
}

// Add an example here.
float GetTauValue(const PID *const pid, bool *error) {
    // Check that the pointer is not NULL.
    if (error == NULL) {
        _PID_ErrorCode = ERR_CRITICAL;
        _error = true;

        return 0.0f;    // Return a zero value.
    }

    *error = false;             // Set the error flag to false.
    _PID_ErrorCode = NO_ERR;    // Set the error code to NO_ERR.

    // Check that the pointer is not NULL.
    if (pid == NULL) {
        // The pointer is NULL.
        _PID_ErrorCode = ERR_NULL_PTR;  // Set the error code to ERR_NULL_PTR.
        *error = true;                  // Set the error flag to true.

        return 0.0f;    // Return a zero value.
    }

    // Check that the PID controller is initialized.
    if (!pid->initialized) {
        // The PID controller is not initialized.
        _PID_ErrorCode = ERR_NOT_INIT;  // Set the error code to ERR_NOT_INIT.
        *error = true;                  // Set the error flag to true.

        return 0.0f;    // Return a zero value.
    }

    *error = CheckParam(pid->tau);   // Check that the value can be used.

    if (*error) {
        return 0.0f;    // Return a zero value.
    }

    return pid->tau;
}

// Add an example here.
PID InitPID(const float Kp, const float Ki, const float Kd, const float Ts, const float Fc, const float outSatMin, const float outSatMax, const AntiWindupMode antiWindupMode, const bool lowPassFilterStatus, bool *error) {
    CreatePID(pid);                         // Create a non-initialized PID controller.
    pid.initialized = true;                 // Set the initialization flag to true.

    // Check that the pointer is not NULL.
    if (error == NULL) {
        _PID_ErrorCode = ERR_CRITICAL;
        _error = true;

        return pid;                         // Return the uninitialized PID controller.
    }

    *error = SetKpValue(&pid, Kp);          // Set Kp value.

    // Check for error.
    if (*error) {
        pid.initialized = false;            // Set the initialization flag to false.

        return pid;                         // Return the uninitialized or partially initialized PID controller.
    }

    *error = SetKiValue(&pid, Ki);          // Set Ki value.

    // Check for error.
    if (*error) {
        pid.initialized = false;            // Set the initialization flag to false.

        return pid;                         // Return the uninitialized or partially initialized PID controller.
    }

    *error = SetKdValue(&pid, Kd);          // Set Kd value.

    // Check for error.
    if (*error) {
        pid.initialized = false;            // Set the initialization flag to false.

        return pid;                         // Return the uninitialized or partially initialized PID controller.
    }

    *error = CheckParam(Ts);                // Check that the value can be used.

    // Check for error.
    if (*error) {
        pid.initialized = false;            // Set the initialization flag to false.

        return pid;                         // Return the uninitialized or partially initialized PID controller.
    }

    pid.Ts = Ts;                            // Set the Ts value.

    *error = CheckValue(outSatMin);         // Check that the value can be used.

    // Check for error.
    if (*error) {
        pid.initialized = false;            // Set the initialization flag to false.

        return pid;                         // Return the uninitialized or partially initialized PID controller.
    }

    pid.outSatMin = outSatMin;              // Set the outSatMin value.

    *error = CheckValue(outSatMax);         // Check that the value can be used.

    // Check for error.
    if (*error) {
        pid.initialized = false;            // Set the initialization flag to false.

        return pid;                         // Return the uninitialized or partially initialized PID controller.
    }

    pid.outSatMax = outSatMax;              // Set the outSatMax value.

    // The integrator clamping saturator defaults to the PID controller's output saturator limits.
    pid.integratorSatMin = outSatMin;       // Set the integratorSatMin value.
    pid.integratorSatMax = outSatMax;       // Set the integratorSatMax value.

    pid.antiWindupMode = antiWindupMode;    // Set the anti-windup method.

    *error = SetFcValue(&pid, Fc);          // Set Fc value.

    // Check for error.
    if (*error) {
        pid.initialized = false;            // Set the initialization flag to false.

        return pid;                         // Return the uninitialized or partially initialized PID controller.
    }

    return pid;                             // Return the initialized PID controller.
}

bool UpdatePID(PID *pid, const float measurement) {
    bool error = false; // Initialize the error flag to false.
    float derivativeActionGain = 0.0f;

    // Check whether the PID controller is initialized.
    if (!pid->initialized) {
        // The PID controller is not initialized.
        return error;
    }

    error = CheckValue(measurement);    // Check that the value can be used.

    // Check for error.
    if (error) {
        return error;   // Return the uninitialized or partially initialized PID.
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
