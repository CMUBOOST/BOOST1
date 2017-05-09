#ifndef MODULE_HPP
#define MODULE_HPP

#include "hebi_module.h"
#include "command.hpp"
#include "settings.hpp"
#include "feedback.hpp"
#include "info.hpp"

#include <vector>
#include <functional>
#include <mutex>

namespace hebi {

typedef std::function<void (Feedback* const)> ModuleFeedbackHandler;

class Module final
{
  private:
    static const long DEFAULT_TIMEOUT = 500;

  public:
    // Note: this constructor should only be called from the lookup class! (TODO: make it a friend? Constructor key?)
    Module(HebiModule* module);
    virtual ~Module();

    bool sendCommand(Command& command);
    bool sendCommandWithAcknowledgement(Command& command, long timeout_ms=DEFAULT_TIMEOUT);

    bool sendSettings(Settings& settings);
    bool sendSettingsWithAcknowledgement(Settings& settings, long timeout_ms=DEFAULT_TIMEOUT);
    bool saveSettings(long timeout_ms=DEFAULT_TIMEOUT);

    Feedback requestFeedback(long timeout_ms=DEFAULT_TIMEOUT);
    bool requestInfo(Info& info, long timeout_ms=DEFAULT_TIMEOUT);

    // TODO: write wrapper functions without timeouts?
    bool reset(long timeout_ms);
    bool boot(bool boot_command, long timeout_ms=DEFAULT_TIMEOUT);

    // Asyncronous Feedback Functions
    bool setFeedbackFrequencyHz(float frequency);
    float getFeedbackFrequencyHz();

    void addFeedbackHandler(ModuleFeedbackHandler handler);
    void clearFeedbackHandlers();

  private:
    std::mutex handler_lock_;
    std::vector<ModuleFeedbackHandler> handlers_;
    // TODO: can this still be a C style function while being a friend? If not, just make the relevant callAttachedHandlers function public...)
    friend void callbackWrapper(const HebiFeedback* const module_feedback, void* user_data);
    void callAttachedHandlers(const HebiFeedback* const module_feedback);
    // TODO: delete copy constructor...

    // C-style module object.
    HebiModule* module_;
};

} // namespace hebi

#endif // MODULE_HPP
