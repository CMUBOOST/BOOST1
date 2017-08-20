#ifndef COMMAND_HPP
#define COMMAND_HPP

#include "hebi_led_command.h"
#include "hebi_actuator_command.h"
#include "hebi_command.h"

namespace hebi {

class ActuatorCommand final
{
  public:
    // TODO: make this private with Command being a friend?
    ActuatorCommand(HebiActuatorCommand* actuator_command)
      : actuator_command_(actuator_command) {}
    virtual ~ActuatorCommand() {}

    // TODO: should all these calls be inlined?
    void setPosition(float position)
    {
      hebiActuatorCommandSetPosition(actuator_command_, position);
    }
    float getPosition()
    {
      return hebiActuatorCommandGetPosition(actuator_command_);
    }
    void clearPosition()
    {
      hebiActuatorCommandClearPosition(actuator_command_);
    }
    bool hasPosition()
    {
      return hebiActuatorCommandHasPosition(actuator_command_);
    }
    void setVelocity(float velocity)
    {
      hebiActuatorCommandSetVelocity(actuator_command_, velocity);
    }
    float getVelocity()
    {
      return hebiActuatorCommandGetVelocity(actuator_command_);
    }
    void clearVelocity()
    {
      hebiActuatorCommandClearVelocity(actuator_command_);
    }
    bool hasVelocity()
    {
      return hebiActuatorCommandHasVelocity(actuator_command_);
    }
    void setTorque(float torque)
    {
      hebiActuatorCommandSetTorque(actuator_command_, torque);
    }
    float getTorque()
    {
      return hebiActuatorCommandGetTorque(actuator_command_);
    }
    void clearTorque()
    {
      hebiActuatorCommandClearTorque(actuator_command_);
    }
    bool hasTorque()
    {
      return hebiActuatorCommandHasTorque(actuator_command_);
    }

  private:
    // C-style actuator command object; managed by the HebiCommand* parent so
    // no need to cleanup.
    HebiActuatorCommand* actuator_command_;
};

class LedCommand final
{
  public:
    LedCommand(HebiLedCommand* led_command) // TODO: make this private with Command being a friend?
      : led_command_(led_command) {}
    virtual ~LedCommand() {};

    // TODO: finish!
    void setRed()
    {
      hebiLedCommandSetEnableOverride(led_command_, 255, 0, 0);
    }
    void setBlue()
    {
      hebiLedCommandSetEnableOverride(led_command_, 0, 255, 0);
    }
    void setGreen()
    {
      hebiLedCommandSetEnableOverride(led_command_, 0, 0, 255);
    }
    void clear()
    {
      hebiLedCommandSetDisableOverride(led_command_);
    }

  private:
    // C-style led command object; managed by the HebiCommand* parent so
    // no need to cleanup.
    HebiLedCommand* led_command_;
};

class Command final
{
  friend class Module; // Allow module to access internal variable (command_)

  public:
    Command();
    // Creates "Unmanaged" version (TODO: make this protected, and accessible from GroupCommand only?)
    Command(HebiCommand* command);
    virtual ~Command();

    ActuatorCommand actuatorCommand();
    LedCommand ledCommand();

  private:
    // C-style command object.
    HebiCommand* command_;
    // True if this object manages the pointers lifetime, 
    bool manage_pointer_lifetime_;
};

} // namespace hebi

#endif // COMMAND_HPP
