#ifndef HEBI_LED_COMMAND_H
#define HEBI_LED_COMMAND_H

// TODO: document; see include/hebi_module!

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

/**
 * Allows user override of module LED color.
 * This command can either be set to 'override' the current LED color, to turn
 * off the override and allow the module to control its own LED.  The absence of
 * a command leaves the module in the previous state.
 */
typedef struct _HebiLedCommand {} HebiLedCommand;

/**
 * \param r The commanded red channel value.  Must be between 0 and 255.
 * \param g The commanded green channel value.  Must be between 0 and 255.
 * \param b The commanded blue channel value.  Must be between 0 and 255.
 */
void hebiLedCommandSetEnableOverride(HebiLedCommand* cmd, float r, float g, float b);
void hebiLedCommandSetDisableOverride(HebiLedCommand* cmd);
/**
 * Gets the current command, if there is one.  If no command stored, results in undefined behavior.
 *
 * \param r If the command is an override command, this function will store the red LED command
 * value at this location.
 * \param g If the command is an override command, this function will store the green LED command
 * value at this location.
 * \param b If the command is an override command, this function will store the blue LED command
 * value at this location.
 *
 * \returns '1' for an override command, '0' for a disable override command.
 */
int hebiLedCommandGetCommand(HebiLedCommand* cmd, float *r, float *g, float *b);
void hebiLedCommandClearCommand(HebiLedCommand* cmd);
int hebiLedCommandHasCommand(HebiLedCommand* cmd);

#ifdef __cplusplus /* End C linkage when compiling from C++ */
}
#endif

#endif // HEBI_LED_COMMAND_H
