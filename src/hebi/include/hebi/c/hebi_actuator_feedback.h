/* This file has been automatically generated. Do not edit by hand. */

#ifndef HEBI_ACTUATOR_FEEDBACK_H
#define HEBI_ACTUATOR_FEEDBACK_H

#include "hebi_pid_feedback.h"

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

typedef struct _HebiActuatorFeedback {
} HebiActuatorFeedback;

// Define all submessages
HebiPidFeedback* hebiActuatorFeedbackGetPositionPidFeedback(const HebiActuatorFeedback*);
HebiPidFeedback* hebiActuatorFeedbackGetVelocityPidFeedback(const HebiActuatorFeedback*);
HebiPidFeedback* hebiActuatorFeedbackGetTorquePidFeedback(const HebiActuatorFeedback*);

// Define all fields
float hebiActuatorFeedbackGetPosition(const HebiActuatorFeedback*);
int hebiActuatorFeedbackHasPosition(const HebiActuatorFeedback*);

float hebiActuatorFeedbackGetVelocity(const HebiActuatorFeedback*);
int hebiActuatorFeedbackHasVelocity(const HebiActuatorFeedback*);

float hebiActuatorFeedbackGetTorque(const HebiActuatorFeedback*);
int hebiActuatorFeedbackHasTorque(const HebiActuatorFeedback*);

float hebiActuatorFeedbackGetDeflection(const HebiActuatorFeedback*);
int hebiActuatorFeedbackHasDeflection(const HebiActuatorFeedback*);

float hebiActuatorFeedbackGetDeflectionVelocity(const HebiActuatorFeedback*);
int hebiActuatorFeedbackHasDeflectionVelocity(const HebiActuatorFeedback*);

float hebiActuatorFeedbackGetMotorVelocity(const HebiActuatorFeedback*);
int hebiActuatorFeedbackHasMotorVelocity(const HebiActuatorFeedback*);

float hebiActuatorFeedbackGetMotorCurrent(const HebiActuatorFeedback*);
int hebiActuatorFeedbackHasMotorCurrent(const HebiActuatorFeedback*);

float hebiActuatorFeedbackGetMotorTemperature(const HebiActuatorFeedback*);
int hebiActuatorFeedbackHasMotorTemperature(const HebiActuatorFeedback*);

float hebiActuatorFeedbackGetMotorWindingCurrent(const HebiActuatorFeedback*);
int hebiActuatorFeedbackHasMotorWindingCurrent(const HebiActuatorFeedback*);

float hebiActuatorFeedbackGetMotorWindingTemperature(const HebiActuatorFeedback*);
int hebiActuatorFeedbackHasMotorWindingTemperature(const HebiActuatorFeedback*);

float hebiActuatorFeedbackGetActuatorTemperature(const HebiActuatorFeedback*);
int hebiActuatorFeedbackHasActuatorTemperature(const HebiActuatorFeedback*);

#ifdef __cplusplus /* End C linkage when compiling from C++ */
} // extern "C"
#endif

#endif // HEBI_ACTUATOR_FEEDBACK_H
