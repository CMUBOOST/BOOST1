#include "lookup.hpp"
#include <algorithm>

namespace hebi {

Lookup::Lookup()
{
  lookup_ = hebiCreateLookup();
}

Lookup::~Lookup()
{
  hebiDeleteLookup(lookup_);
}

void Lookup::printTable()
{
  hebiPrintLookupTable(lookup_);
}

Module* Lookup::getModuleFromName(const std::string& name, const std::string& family_name, long timeout_ms)
{
  HebiModule* module = hebiGetModuleFromName(lookup_, name.c_str(), family_name.c_str(), timeout_ms);
  if (module == NULL)
    return nullptr;
  else
    return new Module(module);
}

Module* Lookup::getModuleFromMac(const HebiMacAddress address, long timeout_ms)
{
  HebiModule* module = hebiGetModuleFromMac(lookup_, &address, timeout_ms);
  if (module == NULL)
    return nullptr;
  else
    return new Module(module);
}

Group* Lookup::getGroupFromMacs(const std::vector<HebiMacAddress> addresses, long timeout_ms)
{
  HebiGroup* group = hebiGetGroupFromMacs(lookup_, addresses.data(), addresses.size(), timeout_ms);
  if (group == NULL)
    return nullptr;
  else
    return new Group(group);
}

Group* Lookup::getGroupFromNames(const std::vector<std::string> names, std::vector<std::string> family_names, long timeout_ms)
{
  std::vector<const char *> names_cstrs;
  std::vector<const char *> family_names_cstrs;
  names_cstrs.reserve(names.size());
  family_names_cstrs.reserve(family_names.size());
  std::transform(std::begin(names), std::end(names),
    std::back_inserter(names_cstrs), std::mem_fn(&std::string::c_str));
  std::transform(std::begin(family_names), std::end(family_names),
    std::back_inserter(family_names_cstrs), std::mem_fn(&std::string::c_str));
  HebiGroup* group = hebiGetGroupFromNames(lookup_, names_cstrs.data(), names_cstrs.size(), family_names_cstrs.data(), family_names_cstrs.size(), timeout_ms);
  if (group == NULL)
    return nullptr;
  else
    return new Group(group);
}

Group* Lookup::getGroupFromFamily(const std::string& family, long timeout_ms)
{
  HebiGroup* group = hebiGetGroupFromFamily(lookup_, family.c_str(), timeout_ms);
  if (group == NULL)
    return nullptr;
  else
    return new Group(group);
}

} // namespace hebi
