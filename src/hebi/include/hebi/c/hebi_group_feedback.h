/* This file has been automatically generated. Do not edit by hand. */

#ifndef HEBI_GROUP_FEEDBACK_H
#define HEBI_GROUP_FEEDBACK_H

#include "hebi_feedback.h"

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

typedef struct _HebiGroupFeedback {
} HebiGroupFeedback;

HebiGroupFeedback* hebiGroupFeedbackCreate(int number_of_modules);
int hebiGroupFeedbackGetNumModules(const HebiGroupFeedback*);
HebiFeedback* hebiGroupFeedbackGetModuleFeedback(const HebiGroupFeedback*, int module_index);
void hebiGroupFeedbackDestroy(const HebiGroupFeedback*);

#ifdef __cplusplus /* End C linkage when compiling from C++ */
}
#endif

#endif // HEBI_GROUP_FEEDBACK_H
