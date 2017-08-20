/* This file has been automatically generated. Do not edit by hand. */

#ifndef HEBI_ACTUATOR_COMMAND_H
#define HEBI_ACTUATOR_COMMAND_H

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

typedef struct _HebiActuatorCommand {
} HebiActuatorCommand;

// Define all fields
void hebiActuatorCommandSetPosition(HebiActuatorCommand*, float);
float hebiActuatorCommandGetPosition(const HebiActuatorCommand*);
void hebiActuatorCommandClearPosition(HebiActuatorCommand*);
int hebiActuatorCommandHasPosition(const HebiActuatorCommand*);

void hebiActuatorCommandSetVelocity(HebiActuatorCommand*, float);
float hebiActuatorCommandGetVelocity(const HebiActuatorCommand*);
void hebiActuatorCommandClearVelocity(HebiActuatorCommand*);
int hebiActuatorCommandHasVelocity(const HebiActuatorCommand*);

void hebiActuatorCommandSetTorque(HebiActuatorCommand*, float);
float hebiActuatorCommandGetTorque(const HebiActuatorCommand*);
void hebiActuatorCommandClearTorque(HebiActuatorCommand*);
int hebiActuatorCommandHasTorque(const HebiActuatorCommand*);

#ifdef __cplusplus /* End C linkage when compiling from C++ */
} // extern "C"
#endif

#endif // HEBI_ACTUATOR_COMMAND_H
