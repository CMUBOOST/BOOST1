#include "hebi_lookup.h"
#include "hebi_module.h"
#include "module.hpp"

namespace hebi {

Module::Module(HebiModule* module)
  : module_(module)
{
}

Module::~Module()
{
  if (module_ != NULL) // Should not need, but this prevents a crash
    hebiReleaseModule(module_);
}

bool Module::sendCommand(Command& command)
{
  return hebiModuleSendCommand(module_, command.command_);
}

bool Module::sendCommandWithAcknowledgement(Command& command, long timeout_ms)
{
  return hebiModuleSendCommandWithAcknowledgement(module_, command.command_, timeout_ms);
}

/*bool Module::saveSettings(long timeout_ms)
{
  Command persist_command;
  return hebiModuleSendCommandWithAcknowledgement(module_, command.command_, timeout_ms);
}*/

Feedback Module::requestFeedback(long timeout_ms)
{
  HebiFeedback* feedback = hebiFeedbackCreate();
  if (feedback == NULL)
    return Feedback(nullptr);
  bool success = hebiModuleRequestFeedback(module_, feedback, timeout_ms);
  if (success)
    return Feedback(feedback); // TODO: could trigger a 'managed' flag here, so we know we need to destroy this...
  else
    hebiFeedbackDestroy(feedback);
  return Feedback(nullptr);
}

bool Module::requestInfo(Info& info, long timeout_ms)
{
  return hebiModuleRequestInfo(module_, info.info_, timeout_ms);
}

// Asyncronous Feedback Functions
bool Module::setFeedbackFrequencyHz(float frequency)
{
  return hebiModuleSetFeedbackFrequencyHz(module_, frequency);
}

float Module::getFeedbackFrequencyHz()
{
  return hebiModuleGetFeedbackFrequencyHz(module_);
}

// Intermediary to convert C-style function callbacks to C++ style, and change
// callback parameter types.
void callbackWrapper(const HebiFeedback* const module_feedback, void* user_data)
{
  ((Module*)user_data)->callAttachedHandlers(module_feedback);
}

// TODO: think about this locking strategy! Fix it so there are no deadlocks or
// invalid states (e.g., the handlers_ list is > 0 iff the module has a registered
// callback.
// Maybe just always register handler, and just turn it on or off...
void Module::addFeedbackHandler(ModuleFeedbackHandler handler)
{
  std::lock_guard<std::mutex> lock_guard(handler_lock_);
  handlers_.push_back(handler); // Is the copy OK here? or are std::function classes move-only?
  if (handlers_.size() == 1) // (i.e., this was the first one)
    hebiModuleRegisterFeedbackHandler(module_, callbackWrapper, (void*)this);
}

void Module::clearFeedbackHandlers()
{
  hebiModuleClearFeedbackHandlers(module_);
  std::lock_guard<std::mutex> lock_guard(handler_lock_);
  handlers_.clear();
}

void Module::callAttachedHandlers(const HebiFeedback* const module_feedback)
{
  // Wrap this:
  Feedback wrapped_fbk(module_feedback);

  // Call handlers:
  std::lock_guard<std::mutex> lock_guard(handler_lock_);
  for (unsigned int i = 0; i < handlers_.size(); i++)
  {
    ModuleFeedbackHandler handler = handlers_[i];
    // TODO: be sure to catch exceptions!
    try
    {
      handler(&wrapped_fbk);
    }
    catch (...)
    {
      // TODO: print error or something?
    }
  }
}

} // namespace hebi
