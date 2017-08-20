/* This file has been automatically generated. Do not edit by hand. */

#ifndef HEBI_INFO_H
#define HEBI_INFO_H

#include "hebi_actuator_info.h"

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

typedef struct _HebiInfo {
} HebiInfo;

HebiInfo* hebiInfoCreate();

// Define all submessages
HebiActuatorInfo* hebiInfoGetActuatorInfo(const HebiInfo*);

// Define all fields
const char* hebiInfoGetName(const HebiInfo*);
int hebiInfoHasName(const HebiInfo*);

const char* hebiInfoGetFamily(const HebiInfo*);
int hebiInfoHasFamily(const HebiInfo*);

void hebiInfoDestroy(const HebiInfo*);

#ifdef __cplusplus /* End C linkage when compiling from C++ */
} // extern "C"
#endif

#endif // HEBI_INFO_H
