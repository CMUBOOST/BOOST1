#ifndef GROUP_FEEDBACK_HPP
#define GROUP_FEEDBACK_HPP

#include "hebi_group_feedback.h"
#include "feedback.hpp"
#include <vector>

namespace hebi {

class GroupFeedback final
{
  friend class Group; // Allow group to access internal variable (feedback_)

  public:
    GroupFeedback(int number_of_modules);
    // Creates "Unmanaged" version, which is needed when objects are given in
    // a callback. TODO: somehow protect for normal operation?
    GroupFeedback(const HebiGroupFeedback* const group_feedback);
    virtual ~GroupFeedback();

    Feedback& operator[](int index);

  private:
    const int number_of_modules_;
    // List of Feedback subobjects:
    std::vector<Feedback> subfeedbacks_;
    // C-style group feedback object.
    const HebiGroupFeedback* group_feedback_;
    // True if this object manages the pointers lifetime, 
    bool manage_pointer_lifetime_;
};

} // namespace hebi

#endif // GROUP_FEEDBACK_HPP

