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

#include "../includes/OpenPID.h"

/*** Declaration of function prototypes. ***/

PID InitPID(PID *pid, const float Kp, const float Ki, const float Kd, const float Ts, const float Fc, const float satMin, const float satMax, bool *error) {
    *error = true;  // Set the error flag to true.

    // The check routine verifies that the pointer to the PID controller is not null, that the PID controller has not already been initialized.

    // Check that the pointer is not null.
    if (pid != NULL) {
        // Check if the PID controller has already been initialized.
        if (!pid->initialized) {
            pid->Kp = Kp;           // Set the Kp value.
            pid->Ki = Ki;           // Set the Ki value.
            pid->Kd = Kd;           // Set the Kd value.
            pid->Ts = Ts;           // Set the Ts value.
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

            pid->initialized = true;    // Set the initialization flag to true.
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
        NewPID.Ts = Ts;         // Set the Ts value.
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
