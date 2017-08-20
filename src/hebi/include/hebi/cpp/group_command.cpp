#include "group_command.hpp"

namespace hebi {

GroupCommand::GroupCommand(int number_of_modules)
  : number_of_modules_(number_of_modules)
{
  // TODO: check return value?
  group_command_ = hebiGroupCommandCreate(number_of_modules_);
  for (int i = 0; i < number_of_modules_; i++)
    subcommands_.emplace_back(hebiGroupCommandGetModuleCommand(group_command_, i));
}

GroupCommand::~GroupCommand()
{
  hebiGroupCommandDestroy(group_command_);
}

Command& GroupCommand::operator[](int index)
{
  return subcommands_[index];
}

} // namespace hebi
