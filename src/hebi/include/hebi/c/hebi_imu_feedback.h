/* This file has been automatically generated. Do not edit by hand. */

#ifndef HEBI_IMU_FEEDBACK_H
#define HEBI_IMU_FEEDBACK_H

#include "hebi_vector_3_f.h"

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

typedef struct _HebiImuFeedback {
} HebiImuFeedback;

// Define all fields
HebiVector3f hebiImuFeedbackGetAccelerometer(const HebiImuFeedback*);
int hebiImuFeedbackHasAccelerometer(const HebiImuFeedback*);

HebiVector3f hebiImuFeedbackGetGyro(const HebiImuFeedback*);
int hebiImuFeedbackHasGyro(const HebiImuFeedback*);

#ifdef __cplusplus /* End C linkage when compiling from C++ */
} // extern "C"
#endif

#endif // HEBI_IMU_FEEDBACK_H
