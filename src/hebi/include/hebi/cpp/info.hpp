#ifndef INFO_HPP
#define INFO_HPP

#include "hebi_info.h"

namespace hebi {

class Info final
{
  friend class Module; // Allow module to access internal variable (info_)

  public:
    Info(HebiInfo* info);
    virtual ~Info();

  private:
    // C-style info object.
    HebiInfo* info_;
};

} // namespace hebi

#endif // INFO_HPP
