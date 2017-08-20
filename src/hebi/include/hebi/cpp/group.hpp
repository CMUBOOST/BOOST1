#ifndef GROUP_HPP
#define GROUP_HPP

#include "hebi_group.h"
#include "group_command.hpp"
#include "group_feedback.hpp"

#include <functional>
#include <mutex>

namespace hebi {

typedef std::function<void (GroupFeedback* const)> GroupFeedbackHandler;

class Group final
{
  public:
    // Note: this constructor should only be called from the lookup class! (TODO: make it a friend? Constructor key?)
    Group(HebiGroup* group);
    virtual ~Group();

    // Returns the number of modules in the group
    int size();

    // TODO: add remaining functions!

    bool sendCommand(GroupCommand& command);

    // Asyncronous Feedback Functions
    bool setFeedbackFrequencyHz(float frequency);
    float getFeedbackFrequencyHz();

    void addFeedbackHandler(GroupFeedbackHandler handler);
    void clearFeedbackHandlers();

  private:
    std::mutex handler_lock_;
    std::vector<GroupFeedbackHandler> handlers_;
    // TODO: can this still be a C style function while being a friend? If not, just make the relevant callAttachedHandlers function public...)
    friend void callbackWrapper(const HebiGroupFeedback* const group_feedback, void* user_data);
    void callAttachedHandlers(const HebiGroupFeedback* const group_feedback);
    // TODO: delete copy constructor...

    // C-style group object.
    HebiGroup* group_;
};

} // namespace hebi

#endif // GROUP_HPP
