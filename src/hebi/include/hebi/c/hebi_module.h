#ifndef HEBI_MODULE_H
#define HEBI_MODULE_H

#include "hebi_command.h"
#include "hebi_feedback.h"
#include "hebi_info.h"
#include <stdint.h>

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

/**
 * The C-style's API representation of a module. Do not inherit from this; only
 * obtain pointers through the API!
 *
 * Represents a connection to a single module.  Sends commands to and receives
 * feedback from the module.
 *
 * @author Matthew Tesch < matt @ hebirobotics.com >
 * @since 11 Jan 2015
 */
typedef struct _HebiModule {} HebiModule;

/**
 * Define a type for a feedback handling function.
 */
typedef void (*FeedbackHandlerFunction)(const HebiFeedback* const, void* user_data);

// TODO: state all failure cases for set/get functions.
// TODO: trigger 'persist' on setting name and gains guaranteed? confusing b/c of side effects? worse not to?
// TODO: logging

/**
 * Sends a command to the given module, requesting an acknowledgement of
 * transmission to be sent back.
 *
 * \param module The module to send this command to.
 * \param command The HebiCommand object containing information to be sent to
 * the module.
 * \param timeout_ms Indicates how many milliseconds to wait for a response
 * after sending a packet.  For typical networks, '15' ms is a value that can be
 * reasonably expected to encompass the time required for a round-trip
 * transmission.
 *
 * \returns '0' if an acknowledgement was successfully received (guaranteeing
 * the module received this command), or a negative number for an error otherwise.
 *
 * Note: A non-zero return does not indicate a specific failure, and may
 * result from an error while sending or simply a timeout/dropped response
 * packet after a successful transmission.
 */
int hebiModuleSendCommandWithAcknowledgement(HebiModule* const module,
  HebiCommand* const command,
  int timeout_ms);

/**
 * Sends a command to the given module without requesting an acknowledgement.
 * Appropriate for high-frequency applications.
 *
 * \param module The module to send this command to.
 * \param command The HebiCommand object containing information to be sent to
 * the module.
 *
 * \returns '0' if the command was successfully sent, a negative number otherwise.
 */
int hebiModuleSendCommand(HebiModule* const module,
  HebiCommand* const command);

/**
 * Sets the feedback request loop frequency (in Hz). The module is queried for
 * feedback in a background thread at this frequency, and any added callbacks
 * are called from this background thread.
 *
 * \param module Which module this frequency set is for.
 * \param frequency The feedback request loop frequency (in Hz). A value of '0'
 * is the default, and disables the feedback request thread.
 *
 * \returns '0' if feedback frequency successfully set, or a negative integer if value was
 * outside of accepted range (less than zero or faster than supported maximum).
 */
int hebiModuleSetFeedbackFrequencyHz(HebiModule* const module, float frequency);

/**
 * Returns the current feedback request loop frequency (in Hz).
 *
 * \param module Which module is being queried.
 *
 * \returns The current feedback request loop frequency (in Hz).
 */
float hebiModuleGetFeedbackFrequencyHz(HebiModule* const module);

/**
 * Add a function that is called whenever feedback is returned from the module.
 *
 * \param module The module to attach this handler to.
 * \param handler A feedback handling function called whenever feedback is
 * received from the module.
 * \param user_data A pointer to user data which will be returned as the second
 * callback argument. This pointer can be NULL if desired.
 */
void hebiModuleRegisterFeedbackHandler(
  HebiModule* const module, FeedbackHandlerFunction handler, void* user_data);

/**
 * Removes all feedback handling functions from the queue to be called on
 * receipt of module feedback.
 *
 * \param module The module to which the handlers are attached.
 */
void hebiModuleClearFeedbackHandlers(
  HebiModule* const module);

/**
 * Requests feedback from the module, and writes it to the provided Feedback
 * object.
 * Warning: other data in the provided 'Feedback' object is erased!
 *
 * \param module The module to return feedback from.
 * \param feedback On success, the feedback read from the module are written
 * into this structure.
 * \param timeout_ms Indicates how many milliseconds to wait for a response
 * after sending a packet.  For typical networks, '15' ms is a value that can be
 * reasonably expected to encompass the time required for a round-trip
 * transmission.
 *
 * \returns '0' if feedback was returned, or a negative integer if not (i.e., connection
 * error or timeout waiting for response).
 */
int hebiModuleRequestFeedback(HebiModule* const module, HebiFeedback* feedback,
  int timeout_ms);

/**
 * Requests info from the module, and writes it to the provided info object.
 * This includes feedback, commands, settings, and other data such as module
 * type.
 * Warning: other data in the provided 'Info' object is erased!
 *
 * \param module The module to send this command to.
 * \param info On success, the info read from the module is written into this
 * structure.
 * \param timeout_ms Indicates how many milliseconds to wait for a response
 * after sending a packet.  For typical networks, '15' ms is a value that can be
 * reasonably expected to encompass the time required for a round-trip
 * transmission.
 *
 * \returns '0' if feedback was returned, or a negative integer if not (i.e., connection
 * error or timeout waiting for response).
 */
int hebiModuleRequestInfo(HebiModule* const module, HebiInfo* info,
  int timeout_ms);

#ifdef __cplusplus /* End C linkage when compiling from C++ */
}
#endif

#endif // HEBI_MODULE_H
