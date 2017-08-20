/* This file has been automatically generated. Do not edit by hand. */

#ifndef HEBI_PID_INFO_H
#define HEBI_PID_INFO_H

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

typedef struct _HebiPidInfo {
} HebiPidInfo;

// Define all fields
float hebiPidInfoGetKp(const HebiPidInfo*);
int hebiPidInfoHasKp(const HebiPidInfo*);

float hebiPidInfoGetKi(const HebiPidInfo*);
int hebiPidInfoHasKi(const HebiPidInfo*);

float hebiPidInfoGetKd(const HebiPidInfo*);
int hebiPidInfoHasKd(const HebiPidInfo*);

float hebiPidInfoGetFeedForward(const HebiPidInfo*);
int hebiPidInfoHasFeedForward(const HebiPidInfo*);

float hebiPidInfoGetDeadZone(const HebiPidInfo*);
int hebiPidInfoHasDeadZone(const HebiPidInfo*);

float hebiPidInfoGetIClamp(const HebiPidInfo*);
int hebiPidInfoHasIClamp(const HebiPidInfo*);

float hebiPidInfoGetPunch(const HebiPidInfo*);
int hebiPidInfoHasPunch(const HebiPidInfo*);

float hebiPidInfoGetMinTarget(const HebiPidInfo*);
int hebiPidInfoHasMinTarget(const HebiPidInfo*);

float hebiPidInfoGetMaxTarget(const HebiPidInfo*);
int hebiPidInfoHasMaxTarget(const HebiPidInfo*);

float hebiPidInfoGetTargetLowpassGain(const HebiPidInfo*);
int hebiPidInfoHasTargetLowpassGain(const HebiPidInfo*);

float hebiPidInfoGetMinOutput(const HebiPidInfo*);
int hebiPidInfoHasMinOutput(const HebiPidInfo*);

float hebiPidInfoGetMaxOutput(const HebiPidInfo*);
int hebiPidInfoHasMaxOutput(const HebiPidInfo*);

float hebiPidInfoGetOutputLowpassGain(const HebiPidInfo*);
int hebiPidInfoHasOutputLowpassGain(const HebiPidInfo*);

int hebiPidInfoGetDOnError(const HebiPidInfo*);
int hebiPidInfoHasDOnError(const HebiPidInfo*);

#ifdef __cplusplus /* End C linkage when compiling from C++ */
} // extern "C"
#endif

#endif // HEBI_PID_INFO_H
