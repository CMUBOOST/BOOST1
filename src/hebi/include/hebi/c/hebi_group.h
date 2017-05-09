#ifndef HEBI_GROUP_H
#define HEBI_GROUP_H

#include "hebi_group_command.h"
#include "hebi_group_feedback.h"
#include "hebi_group_info.h"
#include <stdint.h>

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

/**
 * The C-style's API representation of a group. Do not inherit from this; only
 * obtain pointers through the API!
 *
 * Represents a connection to a group of modules. Sends commands to and receives
 * feedback from the group.
 *
 * @author Matthew Tesch < matt @ hebirobotics.com >
 * @since 25 Mar 2015
 */
typedef struct _HebiGroup {} HebiGroup;

/**
 * Define a type for a group feedback handling function.
 */
typedef void (*GroupFeedbackHandlerFunction)(const HebiGroupFeedback* const, void* user_data);

/**
 * Note: call prevents name mangling to improve library compatibility
 * across compilers.
 */

// TODO: state all failure cases for set/get functions.
// TODO: trigger 'persist' on setting name and gains guaranteed? confusing b/c of side effects? worse not to?
// TODO: logging

/**
 * Returns the number of modules in a group.
 *
 * \param group The group to send this command to.
 *
 * \returns the number of modules in 'group'.
 */
int hebiGroupGetNumberOfModules(HebiGroup* const group);

/**
 * Sends a command to the given group, requesting an acknowledgement of
 * transmission to be sent back.
 *
 * \param group The group to send this command to.
 * \param command The HebiGroupCommand object containing information to be sent to
 * the group.
 * \param timeout_ms Indicates how many milliseconds to wait for a response
 * after sending a packet.  For typical networks, '15' ms is a value that can be
 * reasonably expected to encompass the time required for a round-trip
 * transmission.
 *
 * \returns '0' if an acknowledgement was successfully received (guaranteeing
 * the group received this command), or a negative number for an error otherwise.
 *
 * Note: A non-zero return does not indicate a specific failure, and may
 * result from an error while sending or simply a timeout/dropped response
 * packet after a successful transmission.
 */
int hebiGroupSendCommandWithAcknowledgement(HebiGroup* const group,
  HebiGroupCommand* const command,
  int timeout_ms);

/**
 * Sends a command to the given group without requesting an acknowledgement.
 * Appropriate for high-frequency applications.
 *
 * \param group The group to send this command to.
 * \param command The HebiGroupCommand object containing information to be sent to
 * the group.
 *
 * \returns '0' if the command was successfully sent, a negative number otherwise.
 */
int hebiGroupSendCommand(HebiGroup* const group,
  HebiGroupCommand* const command);

/**
 * Sets the feedback request loop frequency (in Hz). The group is queried for
 * feedback in a background thread at this frequency, and any added callbacks
 * are called from this background thread.
 *
 * \param group Which group this frequency set is for.
 * \param frequency The feedback request loop frequency (in Hz). A value of '0'
 * is the default, and disables the feedback request thread.
 *
 * \returns '0' if feedback frequency successfully set, or a negative integer if value was
 * outside of accepted range (less than zero or faster than supported maximum).
 */
int hebiGroupSetFeedbackFrequencyHz(HebiGroup* const group, float frequency);

/**
 * Returns the current feedback request loop frequency (in Hz).
 *
 * \param group Which group is being queried.
 *
 * \returns The current feedback request loop frequency (in Hz).
 */
float hebiGroupGetFeedbackFrequencyHz(HebiGroup* const group);

/**
 * Add a function that is called whenever feedback is returned from the group.
 *
 * \param group The group to attach this handler to.
 * \param handler A feedback handling function called whenever feedback is
 * received from the group.
 * \param user_data A pointer to user data which will be returned as the second
 * callback argument. This pointer can be NULL if desired.
 */
void hebiGroupRegisterFeedbackHandler(
  HebiGroup* const group, GroupFeedbackHandlerFunction handler, void* user_data);

/**
 * Removes all feedback handling functions from the queue to be called on
 * receipt of group feedback.
 *
 * \param group The group to which the handlers are attached.
 */
void hebiGroupClearFeedbackHandlers(
  HebiGroup* const group);

/**
 * Requests feedback from the group, and writes it to the provided Feedback
 * object.
 * Warning: other data in the provided 'Feedback' object is erased!
 *
 * \param group The group to return feedback from.
 * \param feedback On success, the feedback read from the group are written
 * into this structure.
 * \param timeout_ms Indicates how many milliseconds to wait for a response
 * after sending a packet.  For typical networks, '15' ms is a value that can be
 * reasonably expected to encompass the time required for a round-trip
 * transmission.
 *
 * \returns '0' if feedback was returned, or a negative integer if not (i.e., connection
 * error or timeout waiting for response).
 */
int hebiGroupRequestFeedback(HebiGroup* const group, HebiGroupFeedback* feedback,
  int timeout_ms);

/**
 * Requests info from the group, and writes it to the provided info object.
 * This includes feedback, commands, settings, and other data such as group
 * type.
 * Warning: other data in the provided 'Info' object is erased!
 *
 * \param group The group to send this command to.
 * \param info On success, the info read from the group is written into this
 * structure.
 * \param timeout_ms Indicates how many milliseconds to wait for a response
 * after sending a packet.  For typical networks, '15' ms is a value that can be
 * reasonably expected to encompass the time required for a round-trip
 * transmission.
 *
 * \returns '0' if feedback was returned, or a negative integer if not (i.e., connection
 * error or timeout waiting for response).
 */
int hebiGroupRequestInfo(HebiGroup* const group, HebiGroupInfo* info,
  int timeout_ms);

#ifdef __cplusplus /* End C linkage when compiling from C++ */
}
#endif

#endif // HEBI_GROUP_H
