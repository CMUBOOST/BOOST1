/* This file has been automatically generated. Do not edit by hand. */

#ifndef HEBI_PID_FEEDBACK_H
#define HEBI_PID_FEEDBACK_H

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

typedef struct _HebiPidFeedback {
} HebiPidFeedback;

// Define all fields
float hebiPidFeedbackGetKp(const HebiPidFeedback*);
int hebiPidFeedbackHasKp(const HebiPidFeedback*);

float hebiPidFeedbackGetKi(const HebiPidFeedback*);
int hebiPidFeedbackHasKi(const HebiPidFeedback*);

float hebiPidFeedbackGetKd(const HebiPidFeedback*);
int hebiPidFeedbackHasKd(const HebiPidFeedback*);

float hebiPidFeedbackGetFeedForward(const HebiPidFeedback*);
int hebiPidFeedbackHasFeedForward(const HebiPidFeedback*);

float hebiPidFeedbackGetDeadZone(const HebiPidFeedback*);
int hebiPidFeedbackHasDeadZone(const HebiPidFeedback*);

float hebiPidFeedbackGetIClamp(const HebiPidFeedback*);
int hebiPidFeedbackHasIClamp(const HebiPidFeedback*);

float hebiPidFeedbackGetPunch(const HebiPidFeedback*);
int hebiPidFeedbackHasPunch(const HebiPidFeedback*);

float hebiPidFeedbackGetMinTarget(const HebiPidFeedback*);
int hebiPidFeedbackHasMinTarget(const HebiPidFeedback*);

float hebiPidFeedbackGetMaxTarget(const HebiPidFeedback*);
int hebiPidFeedbackHasMaxTarget(const HebiPidFeedback*);

float hebiPidFeedbackGetTargetLowpassGain(const HebiPidFeedback*);
int hebiPidFeedbackHasTargetLowpassGain(const HebiPidFeedback*);

float hebiPidFeedbackGetMinOutput(const HebiPidFeedback*);
int hebiPidFeedbackHasMinOutput(const HebiPidFeedback*);

float hebiPidFeedbackGetMaxOutput(const HebiPidFeedback*);
int hebiPidFeedbackHasMaxOutput(const HebiPidFeedback*);

float hebiPidFeedbackGetOutputLowpassGain(const HebiPidFeedback*);
int hebiPidFeedbackHasOutputLowpassGain(const HebiPidFeedback*);

int hebiPidFeedbackGetDOnError(const HebiPidFeedback*);
int hebiPidFeedbackHasDOnError(const HebiPidFeedback*);

#ifdef __cplusplus /* End C linkage when compiling from C++ */
} // extern "C"
#endif

#endif // HEBI_PID_FEEDBACK_H
