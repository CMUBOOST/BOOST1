#ifndef HEBI_LED_FEEDBACK_H
#define HEBI_LED_FEEDBACK_H

// TODO: document; see include/hebi_module!

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

/**
 * The current LED color.
 */
typedef struct _HebiLedFeedback {} HebiLedFeedback;

/**
 * Gets the current color, if there is one.  If no color stored, returns false
 * and does not modify r, g, and b.
 *
 * \param r If the command is an override command, this function will store the red LED command
 * value at this location.
 * \param g If the command is an override command, this function will store the green LED command
 * value at this location.
 * \param b If the command is an override command, this function will store the blue LED command
 * value at this location.
 *
 * \returns '1' for if color stored, '0' otherwise.
 */
int hebiLedFeedbackGetColor(HebiLedFeedback* fbk, float *r, float *g, float *b);
int hebiLedFeedbackHasColor(HebiLedFeedback* fbk);

#ifdef __cplusplus /* End C linkage when compiling from C++ */
}
#endif

#endif // HEBI_LED_FEEDBACK_H
