#include "command.hpp"

namespace hebi {

Command::Command()
  : command_(hebiCommandCreate())
{
  // TODO: what about failures for creating commands? This results in an invalid
  // object! Private constructor, and then creator method returning object?
  // Perhaps have a std::move, but not a copy constructor? Only one command
  // should have a reference to this child object!
  manage_pointer_lifetime_ = true;
}

Command::Command(HebiCommand* command)
  : command_(command)
{
  manage_pointer_lifetime_ = false;
}

ActuatorCommand Command::actuatorCommand()
{
  return ActuatorCommand(hebiCommandGetActuatorCommand(command_));
}

LedCommand Command::ledCommand()
{
  return LedCommand(hebiCommandGetLedCommand(command_));
}

Command::~Command()
{
  if (manage_pointer_lifetime_)
    hebiCommandDestroy(command_);
}

} // namespace hebi

