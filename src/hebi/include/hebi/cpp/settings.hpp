#ifndef SETTINGS_HPP
#define SETTINGS_HPP

#include "hebi_settings.h"

namespace hebi {

class Settings final
{
  friend class Module; // Allow module to access internal variable (settings_)

  public:
    Settings(HebiSettings* settings);
    virtual ~Settings();

  private:
    // C-style settings object.
    HebiSettings* settings_;
};

} // namespace hebi

#endif // SETTINGS_HPP
