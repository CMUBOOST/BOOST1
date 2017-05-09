#include "group_feedback.hpp"

namespace hebi {

GroupFeedback::GroupFeedback(int number_of_modules)
  : number_of_modules_(number_of_modules)
{
  // TODO: check return value?
  group_feedback_ = hebiGroupFeedbackCreate(number_of_modules_);
  for (int i = 0; i < number_of_modules_; i++)
    subfeedbacks_.emplace_back(hebiGroupFeedbackGetModuleFeedback(group_feedback_, i));
  manage_pointer_lifetime_ = true;
}

GroupFeedback::GroupFeedback(const HebiGroupFeedback* const group_feedback)
  : number_of_modules_(hebiGroupFeedbackGetNumModules(group_feedback)),
    group_feedback_(group_feedback)
{
  // TODO: what happens on error during construction (invalid group_feedback)
  for (int i = 0; i < number_of_modules_; i++)
    subfeedbacks_.emplace_back(hebiGroupFeedbackGetModuleFeedback(group_feedback_, i));
  manage_pointer_lifetime_ = false;
}

GroupFeedback::~GroupFeedback()
{
  if (manage_pointer_lifetime_)
    hebiGroupFeedbackDestroy(group_feedback_);
}

Feedback& GroupFeedback::operator[](int index)
{
  return subfeedbacks_[index];
}

} // namespace hebi

