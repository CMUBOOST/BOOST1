#ifndef HEBI_LOOKUP_H
#define HEBI_LOOKUP_H

#include "hebi_module.h"
#include "hebi_group.h"
#include "hebi_mac_address.h"

// TODO: any known public fields in HebiLookup? Must be added now or forever hold your peace for binary compat!

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

/**
 * Maintains a list of connected modules and returns Module objects to the user.
 * Only one ModuleLookup object is needed per application.
 *
 * @author Matthew Tesch < matt @ hebirobotics.com >
 * @since 29 Oct 2014
 */
typedef struct _HebiLookup {} HebiLookup;

/*class ModuleLookup
{
  public:*/
    /**
     * Blocking call which waits to get a pointer to a Module object with the
     * given mac address.  Times out after timeout_msec milliseconds.
     * 
     * @param mac Physical mac address of the given module.  This is a unique
     * identifier for the hardware.
     * @param timeout_msec Timeout in milliseconds.  A value of -1 blocks until
     * a module is found, and a value of 0 returns immediately if no module with
     * that address is currently known by the ModuleLookup class.
     * @returns NULL if no module found in allotted time; pointer to module
     * object corresponding to mac otherwise.
     */
/*    virtual Module* getModule(const MacAddress* mac, long timeout_msec) = 0;

    virtual ~ModuleLookup() { };
};*/

/**
 * Get a pointer to a ModuleLookup instance. call prevents name
 * mangling to improve library compatibility across compilers.
 */
HebiLookup* hebiCreateLookup();
// TODO: document remaining!
// TODO: rename hebiLookup* to better match convention:
void hebiDeleteLookup(HebiLookup* lookup);
HebiModule* hebiGetModuleFromMac(HebiLookup* lookup, const HebiMacAddress* address, long timeout_ms);
HebiModule* hebiGetModuleFromName(HebiLookup* lookup, const char* name, const char* family, long timeout_ms);
HebiGroup* hebiGetGroupFromMacs(HebiLookup* lookup, const HebiMacAddress* addresses, int num_addresses, long timeout_ms);
HebiGroup* hebiGetGroupFromNames(HebiLookup* lookup, const char* const * names, int num_names, const char* const * families, int num_families, long timeout_ms);
HebiGroup* hebiGetGroupFromFamily(HebiLookup* lookup, const char* family, long timeout_ms);
HebiGroup* hebiGetConnectedGroupFromMac(HebiLookup* lookup, const HebiMacAddress* address, long timeout_ms);
HebiGroup* hebiGetConnectedGroupFromName(HebiLookup* lookup, const char* name, const char* family, long timeout_ms);
void hebiPrintLookupTable(HebiLookup* lookup);
void hebiReleaseModule(HebiModule* module);
void hebiReleaseGroup(HebiGroup* group);

#ifdef __cplusplus /* End C linkage when compiling from C++ */
}
#endif

#endif // HEBI_LOOKUP_H
