/* This file has been automatically generated. Do not edit by hand. */

#ifndef HEBI_ACTUATOR_SETTINGS_H
#define HEBI_ACTUATOR_SETTINGS_H

#include "hebi_pid_settings.h"

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

typedef struct _HebiActuatorSettings {
} HebiActuatorSettings;

// Define all submessages
HebiPidSettings* hebiActuatorSettingsGetPositionPidSettings(const HebiActuatorSettings*);
HebiPidSettings* hebiActuatorSettingsGetVelocityPidSettings(const HebiActuatorSettings*);
HebiPidSettings* hebiActuatorSettingsGetTorquePidSettings(const HebiActuatorSettings*);

// Define all fields
void hebiActuatorSettingsSetControlStrategy(HebiActuatorSettings*, int);
int hebiActuatorSettingsGetControlStrategy(const HebiActuatorSettings*);
void hebiActuatorSettingsClearControlStrategy(HebiActuatorSettings*);
int hebiActuatorSettingsHasControlStrategy(const HebiActuatorSettings*);

#ifdef __cplusplus /* End C linkage when compiling from C++ */
} // extern "C"
#endif

#endif // HEBI_ACTUATOR_SETTINGS_H
