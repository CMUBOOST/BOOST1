#include "feedback.hpp"

namespace hebi {

Feedback::Feedback()
  : feedback_(hebiFeedbackCreate())
{
  // TODO: what about failures for creating feedbacks? This results in an invalid
  // object! Private constructor, and then creator method returning object?
  // Perhaps have a std::move, but not a copy constructor? Only one feedback
  // should have a reference to this child object!
  manage_pointer_lifetime_ = true;
}

Feedback::Feedback(const HebiFeedback* const feedback)
  : feedback_(feedback)
{
  manage_pointer_lifetime_ = false;
}

ActuatorFeedback Feedback::actuatorFeedback()
{
  return ActuatorFeedback(hebiFeedbackGetActuatorFeedback(feedback_));
}

LedFeedback Feedback::ledFeedback()
{
  return LedFeedback(feedback_);
}

IoFeedback Feedback::ioFeedback()
{
  return IoFeedback(hebiFeedbackGetIoFeedback(feedback_));
}

Feedback::~Feedback()
{
  if (manage_pointer_lifetime_)
    hebiFeedbackDestroy(feedback_);
}

} // namespace hebi
