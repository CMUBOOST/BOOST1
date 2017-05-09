/* This file has been automatically generated. Do not edit by hand. */

#ifndef HEBI_SETTINGS_H
#define HEBI_SETTINGS_H

#include "hebi_actuator_settings.h"

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

typedef struct _HebiSettings {
} HebiSettings;

// Define all submessages
HebiActuatorSettings* hebiSettingsGetActuatorSettings(const HebiSettings*);

// Define all fields
/**
 * Sets the name for this settings object.
 *
 * \param name Null-terminated character string for the name; must be <= 20
 * characters.
 */
void hebiSettingsSetName(HebiSettings*, const char *);
const char* hebiSettingsGetName(const HebiSettings*);
void hebiSettingsClearName(HebiSettings*);
int hebiSettingsHasName(const HebiSettings*);

/**
 * Sets the family for this settings object.
 *
 * \param family Null-terminated character string for the family; must be <=
 * 20 characters.
 */
void hebiSettingsSetFamily(HebiSettings*, const char *);
const char* hebiSettingsGetFamily(const HebiSettings*);
void hebiSettingsClearFamily(HebiSettings*);
int hebiSettingsHasFamily(const HebiSettings*);

/**
 * Sets or clears a flag to indicate if the module should save the current
 * values of all of its settings.
 *
 * A value of '1' sets this flag and a value of '0' clears this flag.
 */
void hebiSettingsSetSaveCurrentSettingsFlag(HebiSettings*, int);
/**
 * Checks whether this flag is set.  Returns '1' for yes, '0' for no.
 */
int hebiSettingsHasSaveCurrentSettingsFlag(const HebiSettings*);

#ifdef __cplusplus /* End C linkage when compiling from C++ */
} // extern "C"
#endif

#endif // HEBI_SETTINGS_H
