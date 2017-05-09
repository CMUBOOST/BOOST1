/* This file has been automatically generated. Do not edit by hand. */

#ifndef HEBI_PID_SETTINGS_H
#define HEBI_PID_SETTINGS_H

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

typedef struct _HebiPidSettings {
} HebiPidSettings;

// Define all fields
void hebiPidSettingsSetKp(HebiPidSettings*, float);
float hebiPidSettingsGetKp(const HebiPidSettings*);
void hebiPidSettingsClearKp(HebiPidSettings*);
int hebiPidSettingsHasKp(const HebiPidSettings*);

void hebiPidSettingsSetKi(HebiPidSettings*, float);
float hebiPidSettingsGetKi(const HebiPidSettings*);
void hebiPidSettingsClearKi(HebiPidSettings*);
int hebiPidSettingsHasKi(const HebiPidSettings*);

void hebiPidSettingsSetKd(HebiPidSettings*, float);
float hebiPidSettingsGetKd(const HebiPidSettings*);
void hebiPidSettingsClearKd(HebiPidSettings*);
int hebiPidSettingsHasKd(const HebiPidSettings*);

void hebiPidSettingsSetFeedForward(HebiPidSettings*, float);
float hebiPidSettingsGetFeedForward(const HebiPidSettings*);
void hebiPidSettingsClearFeedForward(HebiPidSettings*);
int hebiPidSettingsHasFeedForward(const HebiPidSettings*);

void hebiPidSettingsSetDeadZone(HebiPidSettings*, float);
float hebiPidSettingsGetDeadZone(const HebiPidSettings*);
void hebiPidSettingsClearDeadZone(HebiPidSettings*);
int hebiPidSettingsHasDeadZone(const HebiPidSettings*);

void hebiPidSettingsSetIClamp(HebiPidSettings*, float);
float hebiPidSettingsGetIClamp(const HebiPidSettings*);
void hebiPidSettingsClearIClamp(HebiPidSettings*);
int hebiPidSettingsHasIClamp(const HebiPidSettings*);

void hebiPidSettingsSetPunch(HebiPidSettings*, float);
float hebiPidSettingsGetPunch(const HebiPidSettings*);
void hebiPidSettingsClearPunch(HebiPidSettings*);
int hebiPidSettingsHasPunch(const HebiPidSettings*);

void hebiPidSettingsSetMinTarget(HebiPidSettings*, float);
float hebiPidSettingsGetMinTarget(const HebiPidSettings*);
void hebiPidSettingsClearMinTarget(HebiPidSettings*);
int hebiPidSettingsHasMinTarget(const HebiPidSettings*);

void hebiPidSettingsSetMaxTarget(HebiPidSettings*, float);
float hebiPidSettingsGetMaxTarget(const HebiPidSettings*);
void hebiPidSettingsClearMaxTarget(HebiPidSettings*);
int hebiPidSettingsHasMaxTarget(const HebiPidSettings*);

void hebiPidSettingsSetTargetLowpassGain(HebiPidSettings*, float);
float hebiPidSettingsGetTargetLowpassGain(const HebiPidSettings*);
void hebiPidSettingsClearTargetLowpassGain(HebiPidSettings*);
int hebiPidSettingsHasTargetLowpassGain(const HebiPidSettings*);

void hebiPidSettingsSetMinOutput(HebiPidSettings*, float);
float hebiPidSettingsGetMinOutput(const HebiPidSettings*);
void hebiPidSettingsClearMinOutput(HebiPidSettings*);
int hebiPidSettingsHasMinOutput(const HebiPidSettings*);

void hebiPidSettingsSetMaxOutput(HebiPidSettings*, float);
float hebiPidSettingsGetMaxOutput(const HebiPidSettings*);
void hebiPidSettingsClearMaxOutput(HebiPidSettings*);
int hebiPidSettingsHasMaxOutput(const HebiPidSettings*);

void hebiPidSettingsSetOutputLowpassGain(HebiPidSettings*, float);
float hebiPidSettingsGetOutputLowpassGain(const HebiPidSettings*);
void hebiPidSettingsClearOutputLowpassGain(HebiPidSettings*);
int hebiPidSettingsHasOutputLowpassGain(const HebiPidSettings*);

void hebiPidSettingsSetDOnError(HebiPidSettings*, int);
int hebiPidSettingsGetDOnError(const HebiPidSettings*);
void hebiPidSettingsClearDOnError(HebiPidSettings*);
int hebiPidSettingsHasDOnError(const HebiPidSettings*);

#ifdef __cplusplus /* End C linkage when compiling from C++ */
} // extern "C"
#endif

#endif // HEBI_PID_SETTINGS_H
