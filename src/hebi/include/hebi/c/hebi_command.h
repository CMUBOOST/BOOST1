/* This file has been automatically generated. Do not edit by hand. */

#ifndef HEBI_COMMAND_H
#define HEBI_COMMAND_H

#include "hebi_settings.h"
#include "hebi_actuator_command.h"
#include "hebi_led_command.h"

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

typedef struct _HebiCommand {
} HebiCommand;

HebiCommand* hebiCommandCreate();

// Define all submessages
HebiSettings* hebiCommandGetSettings(const HebiCommand*);
HebiActuatorCommand* hebiCommandGetActuatorCommand(const HebiCommand*);
HebiLedCommand* hebiCommandGetLedCommand(const HebiCommand*);

// Define all fields
void hebiCommandDestroy(const HebiCommand*);

#ifdef __cplusplus /* End C linkage when compiling from C++ */
} // extern "C"
#endif

#endif // HEBI_COMMAND_H
