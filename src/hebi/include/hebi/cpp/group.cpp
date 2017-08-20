#include "hebi_lookup.h"
#include "group.hpp"

namespace hebi {

Group::Group(HebiGroup* group)
  : group_(group)
{
}

Group::~Group()
{
  // TODO: why can't I use NULL here?  doesn't seem to exist!
//  if (group_ != NULL) // Should not need, but this prevents a crash
    hebiReleaseGroup(group_);
}

int Group::size()
{
  // TODO: cache this value! No point in calling to get it more than once...
  return hebiGroupGetNumberOfModules(group_);
}

// Asyncronous Feedback Functions
bool Group::setFeedbackFrequencyHz(float frequency)
{
  return hebiGroupSetFeedbackFrequencyHz(group_, frequency);
}

float Group::getFeedbackFrequencyHz()
{
  return hebiGroupGetFeedbackFrequencyHz(group_);
}


// Intermediary to convert C-style function callbacks to C++ style, and change
// callback parameter types.
void callbackWrapper(const HebiGroupFeedback* const group_feedback, void* user_data)
{
  ((Group*)user_data)->callAttachedHandlers(group_feedback);
}

// TODO: think about this locking strategy! Fix it so there are no deadlocks or
// invalid states (e.g., the handlers_ list is > 0 iff the group has a registered
// callback.
// Maybe just always register handler, and just turn it on or off...
void Group::addFeedbackHandler(GroupFeedbackHandler handler)
{
  std::lock_guard<std::mutex> lock_guard(handler_lock_);
  handlers_.push_back(handler); // Is the copy OK here? or are std::function classes move-only?
  if (handlers_.size() == 1) // (i.e., this was the first one)
    hebiGroupRegisterFeedbackHandler(group_, callbackWrapper, (void*)this);
}

bool Group::sendCommand(GroupCommand& command)
{
  return hebiGroupSendCommand(group_, command.group_command_);
}

void Group::clearFeedbackHandlers()
{
  hebiGroupClearFeedbackHandlers(group_);
  std::lock_guard<std::mutex> lock_guard(handler_lock_);
  handlers_.clear();
}

void Group::callAttachedHandlers(const HebiGroupFeedback* const group_feedback)
{
  // Wrap this:
  GroupFeedback wrapped_fbk(group_feedback);
  // Call handlers:
  std::lock_guard<std::mutex> lock_guard(handler_lock_);
  for (unsigned int i = 0; i < handlers_.size(); i++)
  {
    GroupFeedbackHandler handler = handlers_[i];
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
