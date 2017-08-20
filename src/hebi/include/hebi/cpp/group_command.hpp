#ifndef GROUP_COMMAND_HPP
#define GROUP_COMMAND_HPP

#include "hebi_group_command.h"
#include "command.hpp"
#include <vector>

namespace hebi {

class GroupCommand final
{
  friend class Group; // Allow group to access internal variable (command_)

  public:
    GroupCommand(int number_of_modules);
    virtual ~GroupCommand();

    Command& operator[](int index);

  private:
    const int number_of_modules_;
    // List of Command subobjects:
    std::vector<Command> subcommands_;
    // C-style group command object.
    HebiGroupCommand* group_command_;
};

} // namespace hebi

#endif // GROUP_COMMAND_HPP
