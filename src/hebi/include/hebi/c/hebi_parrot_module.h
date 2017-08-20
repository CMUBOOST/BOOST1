#ifndef HEBI_PARROT_MODULE_H
#define HEBI_PARROT_MODULE_H

#include "hebi_command.h"
#include "hebi_settings.h"
#include "hebi_feedback.h"
#include "hebi_info.h"

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

/**
 * Represents a 'dummy' module which parrots back commands and settings given
 * to it. Can be created without a lookup object. Useful for testing and
 * debugging.
 *
 * Note: no attempt has been made to make this class reentrant!
 *
 * @author Matthew Tesch < matt @ hebirobotics.com >
 * @since 26 May 2015
 */
typedef struct _HebiParrotModule {} HebiParrotModule;

HebiParrotModule* hebiParrotModuleCreate();
void hebiParrotModuleDestroy(HebiParrotModule* module);

/**
 * Copies the data in the input command to the output command, going through the
 * full internal serialization stack.
 *
 * Warning: other data in the provided 'Command' output object is erased!
 */
void hebiParrotModuleParrotCommand(HebiParrotModule* module,
  HebiCommand* input, HebiCommand* output);

/**
 * Returns feedback based on the previous parrotted command.
 *
 * Warning: other data in the provided 'Feedback' object is erased!
 */
void hebiParrotModuleRequestFeedback(HebiParrotModule* const module,
  HebiFeedback* feedback);

/**
 * Returns info based on the previous parrotted settings.
 *
 * Warning: other data in the provided 'Info' object is erased!
 */
void hebiParrotModuleRequestInfo(HebiParrotModule* const module,
  HebiInfo* info);

#ifdef __cplusplus /* End C linkage when compiling from C++ */
}
#endif

#endif // HEBI_PARROT_MODULE_H
