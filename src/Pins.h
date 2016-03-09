#ifndef PINS_H__
#define PINS_H__

// Load Pins_<platform>.h

#if !defined(PLATFORM)
#define PLATFORM duet
#endif

#define P_EXPAND(x) x
#define P_CONCAT(x,y) P_EXPAND(x)y
#define P_STR(x) #x
#define P_XSTR(x) P_STR(x)
#define P_INCLUDE_FILE P_XSTR(P_CONCAT(Pins_,P_CONCAT(PLATFORM,.h)))
#include P_INCLUDE_FILE

#endif // PINS_H__
