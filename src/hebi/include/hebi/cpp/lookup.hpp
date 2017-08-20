#ifndef LOOKUP_HPP
#define LOOKUP_HPP

#include "hebi_lookup.h"
#include "module.hpp"
#include "group.hpp"
#include <string>

namespace hebi {

class Lookup final
{
  private:
    static const long DEFAULT_TIMEOUT = 500;

  public:
    Lookup();
    virtual ~Lookup();

    // TODO: replace with something that spits out to an ostream?
    void printTable();

    // TODO: why not return 'unique_ptr'?
    // Note: returns 'nullptr' on failure. On success, returns allocated Module. Must be 'deleted'
    // afterwards.
    Module* getModuleFromName(const std::string& name, const std::string& family_name, long timeout_ms=DEFAULT_TIMEOUT);

    // TODO: why not return 'unique_ptr'?
    // Note: returns 'nullptr' on failure. On success, returns allocated Module. Must be 'deleted'
    // afterwards.
    Module* getModuleFromMac(const HebiMacAddress, long timeout_ms=DEFAULT_TIMEOUT);

    // Note: returns 'nullptr' on failure. On success, returns allocated Group. Must be 'deleted'
    // afterwards.
    Group* getGroupFromMacs(const std::vector<HebiMacAddress> addresses, long timeout_ms=DEFAULT_TIMEOUT);

    // Note: returns 'nullptr' on failure. On success, returns allocated Group. Must be 'deleted'
    // afterwards.
    Group* getGroupFromNames(const std::vector<std::string> names, std::vector<std::string> family_names, long timeout_ms=DEFAULT_TIMEOUT);

    // Note: returns 'nullptr' on failure. On success, returns allocated Group. Must be 'deleted'
    // afterwards.
    Group* getGroupFromFamily(const std::string& family, long timeout_ms=DEFAULT_TIMEOUT);

  private:
    // C-style lookup object.
    HebiLookup* lookup_;
};


}

#endif // LOOKUP_HPP
