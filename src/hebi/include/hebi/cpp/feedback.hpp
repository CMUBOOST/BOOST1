#ifndef FEEDBACK_HPP
#define FEEDBACK_HPP

#include "hebi_led_feedback.h"
#include "hebi_actuator_feedback.h"
#include "hebi_feedback.h"

namespace hebi {

class ActuatorFeedback final
{
  public:
    // TODO: make this private with Feedback being a friend?
    ActuatorFeedback(HebiActuatorFeedback* actuator_feedback)
      : actuator_feedback_(actuator_feedback) {}
    virtual ~ActuatorFeedback() {}

    // TODO: should all these calls be inlined?
    float getPosition()
    {
      return hebiActuatorFeedbackGetPosition(actuator_feedback_);
    }
    bool hasPosition()
    {
      return hebiActuatorFeedbackHasPosition(actuator_feedback_);
    }
    float getVelocity()
    {
      return hebiActuatorFeedbackGetVelocity(actuator_feedback_);
    }
    bool hasVelocity()
    {
      return hebiActuatorFeedbackHasVelocity(actuator_feedback_);
    }
    float getTorque()
    {
      return hebiActuatorFeedbackGetTorque(actuator_feedback_);
    }
    bool hasTorque()
    {
      return hebiActuatorFeedbackHasTorque(actuator_feedback_);
    }

  private:
    // C-style actuator feedback object; managed by the HebiFeedback* parent so
    // no need to cleanup.
    HebiActuatorFeedback* actuator_feedback_;
};

class LedFeedback final
{
  public:
    LedFeedback(const HebiFeedback* feedback) // TODO: make this private with Feedback being a friend?
    {
      // C-style led feedback object; managed by the HebiFeedback* parent so
      // no need to cleanup.
      HebiLedFeedback* led_feedback = hebiFeedbackGetLedFeedback(feedback);
      has_feedback = hebiLedFeedbackHasColor(led_feedback);
      if (!has_feedback)
      {
        red = 0; green = 0; blue = 0;
      }
      else
      {
        hebiLedFeedbackGetColor(led_feedback, &red, &green, &blue);
      }
    }
    virtual ~LedFeedback() {};

    bool hasColor()
    {
      return has_feedback;
    }
    float getRed()
    {
      return red;
    }
    float getGreen()
    {
      return green;
    }
    float getBlue()
    {
      return blue;
    }

  private:
    bool has_feedback;
    float red, green, blue;
};

class IoFeedback final
{
  public:
    IoFeedback(HebiIoFeedback* io_feedback) // TODO: make this private with Feedback being a friend?
      : io_feedback_(io_feedback) {}
    virtual ~IoFeedback() {};

    bool hasPin(int pinIndex)
    {
      switch (pinIndex)
      {
        case 0: return (hebiIoFeedbackHasPin1(io_feedback_) == 1);
        case 1: return (hebiIoFeedbackHasPin2(io_feedback_) == 1);
        case 2: return (hebiIoFeedbackHasPin3(io_feedback_) == 1);
        case 3: return (hebiIoFeedbackHasPin4(io_feedback_) == 1);
        case 4: return (hebiIoFeedbackHasPin5(io_feedback_) == 1);
        case 5: return (hebiIoFeedbackHasPin6(io_feedback_) == 1);
        case 6: return (hebiIoFeedbackHasPin7(io_feedback_) == 1);
        case 7: return (hebiIoFeedbackHasPin8(io_feedback_) == 1);
      }
      return false;
    }

    float getPin(int pinIndex)
    {
      switch (pinIndex)
      {
        case 0: return hebiIoFeedbackGetPin1(io_feedback_);
        case 1: return hebiIoFeedbackGetPin2(io_feedback_);
        case 2: return hebiIoFeedbackGetPin3(io_feedback_);
        case 3: return hebiIoFeedbackGetPin4(io_feedback_);
        case 4: return hebiIoFeedbackGetPin5(io_feedback_);
        case 5: return hebiIoFeedbackGetPin6(io_feedback_);
        case 6: return hebiIoFeedbackGetPin7(io_feedback_);
        case 7: return hebiIoFeedbackGetPin8(io_feedback_);
      }
      return 0.0f; // TODO: throw exception here?
    }

  private:
    // C-style io feedback object; managed by the HebiFeedback* parent so
    // no need to cleanup.
    HebiIoFeedback* io_feedback_;
};

class Feedback final
{
  friend class Module; // Allow module to access internal variable (feedback_)

  public:
    Feedback();
    // Creates "Unmanaged" version, which is needed when objects are constructed for callback and
    // for GroupFeedback.  TODO: somehow protect for normal operation?
    Feedback(const HebiFeedback* const feedback);
    virtual ~Feedback();

    ActuatorFeedback actuatorFeedback();
    LedFeedback ledFeedback();
    IoFeedback ioFeedback();

  private:
    // C-style feedback object.
    // TODO: can we make this const HF* const fb_?
    const HebiFeedback* const feedback_;
    // True if this object manages the pointers lifetime, 
    bool manage_pointer_lifetime_;
};

} // namespace hebi

#endif // FEEDBACK_HPP
