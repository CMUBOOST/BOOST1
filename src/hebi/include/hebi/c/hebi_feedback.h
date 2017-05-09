/* This file has been automatically generated. Do not edit by hand. */

#ifndef HEBI_FEEDBACK_H
#define HEBI_FEEDBACK_H

#include "hebi_actuator_feedback.h"
#include "hebi_led_feedback.h"
#include "hebi_imu_feedback.h"
#include "hebi_io_feedback.h"

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

typedef struct _HebiFeedback {
} HebiFeedback;

HebiFeedback* hebiFeedbackCreate();

// Define all submessages
HebiActuatorFeedback* hebiFeedbackGetActuatorFeedback(const HebiFeedback*);
HebiLedFeedback* hebiFeedbackGetLedFeedback(const HebiFeedback*);
HebiImuFeedback* hebiFeedbackGetImuFeedback(const HebiFeedback*);
HebiIoFeedback* hebiFeedbackGetIoFeedback(const HebiFeedback*);

// Define all fields
float hebiFeedbackGetAmbientTemperature(const HebiFeedback*);
int hebiFeedbackHasAmbientTemperature(const HebiFeedback*);

float hebiFeedbackGetProcessorTemperature(const HebiFeedback*);
int hebiFeedbackHasProcessorTemperature(const HebiFeedback*);

float hebiFeedbackGetVoltage(const HebiFeedback*);
int hebiFeedbackHasVoltage(const HebiFeedback*);

void hebiFeedbackDestroy(const HebiFeedback*);

#ifdef __cplusplus /* End C linkage when compiling from C++ */
} // extern "C"
#endif

#endif // HEBI_FEEDBACK_H
