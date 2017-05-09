/* This file has been automatically generated. Do not edit by hand. */

#ifndef HEBI_GROUP_COMMAND_H
#define HEBI_GROUP_COMMAND_H

#include "hebi_command.h"

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

typedef struct _HebiGroupCommand {
} HebiGroupCommand;

HebiGroupCommand* hebiGroupCommandCreate(int number_of_modules);
int hebiGroupCommandGetNumModules(const HebiGroupCommand*);
HebiCommand* hebiGroupCommandGetModuleCommand(const HebiGroupCommand*, int module_index);
void hebiGroupCommandDestroy(const HebiGroupCommand*);

#ifdef __cplusplus /* End C linkage when compiling from C++ */
}
#endif

#endif // HEBI_GROUP_COMMAND_H
