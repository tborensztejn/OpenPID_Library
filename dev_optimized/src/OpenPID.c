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

/*** Functions with advanced safety and error management (under development). ***/
/*
// Add an example here.
bool ApplyRoutineChecks(const PID *const pid, ErrorType *errorType) {
    bool error = true;  // Initialize the error flag to true.

    // Check that the pointer is not null.
    if (pid != NULL) {
        // Check if the PID controller is initialized.
        if (pid->initialized) {
            error = false;          // Set the error flag to false.
            *errorType = NO_ERROR;  // Set the error type.
        } else {
            // The  PID controller is not initialized.
            *errorType = UNINITIALIZED; // Set the error type.
        }

    return error;   // Return the state of the error flag.
}

// Add an example here.
PID InitPID(PID *pid, const float Kp, const float Ki, const float Kd, const float Te, const float Fc, bool *error, ErrorType *errorType) {
    *error = true;  // Set the error flag to true.

    // The check routine verifies that the pointer to the PID controller is not null, that the PID controller has not already been initialized.

    // Check that the pointer is not null.
    if (pid != NULL) {
        // Check if the PID controller has already been initialized.
        if (!pid->initialized) {
            // Avoid NaN type errors.
            // Check that the Kp value is valid.
            if (!isnan(Kp)) {
                // Check that the Ki value is valid.
                if (!isnan(Ki)) {
                    // Check that the Kd value is valid.
                    if (!isnan(Kd)) {
                        // Check that the Te value is valid.
                        if (!isnan(Te)) {
                            // Check that the Fc value is valid.
                            if (!isnan(Fc)) {
                                // Check that the cut-off frequency is not too low.
                                if (Fc > EPSILON) {
                                    pid->Kp = Kp;   // Set the Kp value.
                                    pid->Ki = Ki;   // Set the Ki value.
                                    pid->Kd = Kd;   // Set the Kd value.
                                    pid->Te = Te;   // Set the Te value.
                                    pid->Fc = Fc;   // Set the Fc value.

                                    pid->tau = 1.0f / (2.0f * M_PI * Fc);   // Set the tau value.

                                    pid->initialized = true;    // Set the initialization flag to true.

                                    *error = false;             // Set the error flag to false.
                                    *errorType = NO_ERROR;      // Set the error type.
                                } else {
                                    // The cut-off frequency is too low.
                                    *errorType = FC_TOO_LOW;  // Set the error type.
                                }
                            } else {
                                // Fc value is not valid.
                                *errorType = INVALID_FC_VALUE; // Set the error type.
                            }
                        } else {
                            // Te value is not valid.
                            *errorType = INVALID_TE_VALUE; // Set the error type.
                        }
                    } else {
                        // Kd value is not valid.
                        *errorType = INVALID_KD_VALUE; // Set the error type.
                    }
                } else {
                    // Ki value is not valid.
                    *errorType = INVALID_KI_VALUE; // Set the error type.
                }
            } else {
                // Kp value is not valid.
                *errorType = INVALID_KP_VALUE; // Set the error type.
            }
        } else {
            // The PID controller has already been initialized.
            *errorType = ALREADY_INIT;  // Set the error type.
        }
    } else {
        // The pointer is null.
        //PID NewPID = PID_INITIALIZER;
        CreatePID(NewPID);

        // Avoid NaN type errors.
        // Check that the Kp value is valid.
        if (!isnan(Kp)) {
            // Check that the Ki value is valid.
            if (!isnan(Ki)) {
                // Check that the Kd value is valid.
                if (!isnan(Kd)) {
                    // Check that the Te value is valid.
                    if (!isnan(Te)) {
                        // Check that the Fc value is valid.
                        if (!isnan(Fc)) {
                            // Check that the cut-off frequency is not too low.
                            if (Fc > EPSILON) {
                                NewPID.Kp = Kp;   // Set the Kp value.
                                NewPID.Ki = Ki;   // Set the Ki value.
                                NewPID.Kd = Kd;   // Set the Kd value.
                                NewPID.Te = Te;   // Set the Te value.
                                NewPID.Fc = Fc;   // Set the Fc value.

                                NewPID.tau = 1.0f / (2.0f * M_PI * Fc);   // Set the tau value.

                                NewPID.initialized = true;    // Set the initialization flag to true.

                                *error = false;             // Set the error flag to false.
                                *errorType = NO_ERROR;      // Set the error type.

                                return NewPID;  // Return the initialized new PID controller.
                            } else {
                                // The cut-off frequency is too low.
                                *errorType = FC_TOO_LOW;  // Set the error type.
                            }
                        } else {
                            // Fc value is not valid.
                            *errorType = INVALID_FC_VALUE; // Set the error type.
                        }
                    } else {
                        // Te value is not valid.
                        *errorType = INVALID_TE_VALUE; // Set the error type.
                    }
                } else {
                    // Kd value is not valid.
                    *errorType = INVALID_KD_VALUE; // Set the error type.
                }
            } else {
                // Ki value is not valid.
                *errorType = INVALID_KI_VALUE; // Set the error type.
            }
        } else {
            // Kp value is not valid.
            *errorType = INVALID_KP_VALUE; // Set the error type.
        }
    }
}

// Add an example here.
float GetKpValue(const PID *const pid, bool *error, ErrorType *errorType, const bool check) {
    *error = true;  // Set the error flag to true.
    float Kp;       // Declare the variable that will store the returned value.

    // If routine checks are required.
    if (check) {
        // Perform routine checks.
        *error = ApplyRoutineChecks(pid, errorType);

        if (!*error) {
            Kp = pid->Kp;   // Retrieve the value.

            // Check if the value is valid.
            if (!isnan(Kp)) {
                *errorType = NO_ERROR;  // Set the error type.
            } else {
                // The value is not valid.
                *error = true;          // Set the error flag to true.
                *errorType = ERROR_NAN; // Set the error type.
            }
        } else {
            // Critical error.
            // An error has occurred during the routine checking.
            Kp = NAN;
        }
    } else {
        // Bypass routine checks.
        Kp = pid->Kp;   // Retrieve the value.

        // Check if the value is valid.
        if (!isnan(Kp)) {
            *error = false;         // Set the error flag to false.
            *errorType = NO_ERROR;  // Set the error type.
        } else {
            // The value is not valid.
            *errorType = ERROR_NAN; // Set the error type.
        }
    }

    return Kp;  // Return the value.
}
*/

PID InitPID(PID *pid, const float Kp, const float Ki, const float Kd, const float Te, const float Fc, const float satMin, const float satMax, bool *error) {
    *error = true;  // Set the error flag to true.

    // The check routine verifies that the pointer to the PID controller is not null, that the PID controller has not already been initialized.

    // Check that the pointer is not null.
    if (pid != NULL) {
        // Check if the PID controller has already been initialized.
        if (!pid->initialized) {
            pid->Kp = Kp;           // Set the Kp value.
            pid->Ki = Ki;           // Set the Ki value.
            pid->Kd = Kd;           // Set the Kd value.
            pid->Te = Te;           // Set the Te value.
            pid->Fc = Fc;           // Set the Fc value.
            pid->satMin = satMin;   // Set the satMin value.
            pid->satMax = satMax;   // Set the satMax value.

            // Check that the cut-off frequency is not too low (avoid NaN type errors).
            if (Fc > EPSILON) {
                pid->Fc = Fc;   // Set the Fc value.
            } else {
                // The cut-off frequency is too low.
                pid->Fc = EPSILON;  // Set the Fc value.
            }

            pid->tau = 1.0f / (2.0f * M_PI * pid->Fc);  // Set the tau value.

            NewPID.initialized = true;  // Set the initialization flag to true.
            *error = false;             // Set the error flag to false.
        } else {
            // The PID controller has already been initialized.
            // Add some code here.
        }
    } else {
        // The pointer is null.
        //PID NewPID = PID_INITIALIZER;
        CreatePID(NewPID);

        NewPID.Kp = Kp;         // Set the Kp value.
        NewPID.Ki = Ki;         // Set the Ki value.
        NewPID.Kd = Kd;         // Set the Kd value.
        NewPID.Te = Te;         // Set the Te value.
        NewPID.satMin = satMin; // Set the satMin value.
        NewPID.satMax = satMax; // Set the satMax value.

        // Check that the cut-off frequency is not too low (avoid NaN type errors).
        if (Fc > EPSILON) {
            NewPID.Fc = Fc; // Set the Fc value.
        } else {
            // The cut-off frequency is too low.
            NewPID.Fc = EPSILON;    // Set the Fc value to EPSILON.
        }

        NewPID.tau = 1.0f / (2.0f * M_PI * NewPID.Fc);    // Set the tau value.

        NewPID.initialized = true;  // Set the initialization flag to true.
        *error = false;             // Set the error flag to false.

        return NewPID;  // Return the initialized new PID controller.
    }
}
