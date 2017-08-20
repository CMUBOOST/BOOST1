/* This file has been automatically generated. Do not edit by hand. */

#ifndef HEBI_ACTUATOR_INFO_H
#define HEBI_ACTUATOR_INFO_H

#include "hebi_pid_info.h"

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

typedef struct _HebiActuatorInfo {
} HebiActuatorInfo;

// Define all submessages
HebiPidInfo* hebiActuatorInfoGetPositionPidInfo(const HebiActuatorInfo*);
HebiPidInfo* hebiActuatorInfoGetVelocityPidInfo(const HebiActuatorInfo*);
HebiPidInfo* hebiActuatorInfoGetTorquePidInfo(const HebiActuatorInfo*);

// Define all fields
int hebiActuatorInfoGetControlStrategy(const HebiActuatorInfo*);
int hebiActuatorInfoHasControlStrategy(const HebiActuatorInfo*);

#ifdef __cplusplus /* End C linkage when compiling from C++ */
} // extern "C"
#endif

#endif // HEBI_ACTUATOR_INFO_H
