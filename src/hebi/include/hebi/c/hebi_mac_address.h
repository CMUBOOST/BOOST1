#ifndef HEBI_MAC_ADDRESS_H
#define HEBI_MAC_ADDRESS_H

#include <stdint.h>

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

#define MAC_BYTES 6
typedef struct _HebiMacAddress {
  uint8_t bytes_[MAC_BYTES];
} HebiMacAddress;
#undef MAC_BYTES

/*HebiMacAddress hebiMacAddressFromBytes(
  uint8_t a, uint8_t b, uint8_t c, uint8_t d, uint8_t e, uint8_t f)
{
  HebiMacAddress tmp;
  tmp.bytes_[0] = a;
  tmp.bytes_[1] = b;
  tmp.bytes_[2] = c;
  tmp.bytes_[3] = d;
  tmp.bytes_[4] = e;
  tmp.bytes_[5] = f;
  return tmp;
}*/

#ifdef __cplusplus /* End C linkage when compiling from C++ */
}
#endif

#endif // HEBI_MAC_ADDRESS_H
