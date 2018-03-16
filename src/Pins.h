#ifndef PINS_H__
#define PINS_H__

// Load Pins_<platform>.h

#if !defined(PLATFORM)
# if defined(__SAM3X8E__)
#  if defined(__RADDS__)
#   define PLATFORM RADDS
#  elif defined(__ALLIGATOR__)
#	define PLATFORM Alligator
#  else
#   define PLATFORM Duet
#  endif
# elif defined(__SAM4E8E__)
#  define PLATFORM DuetNG
# elif defined(__SAME70Q21__)
#  define PLATFORM SAME70_TEST
# elif defined(__SAM4S8C__)
#  define PLATFORM DuetM
# else
#  error Unknown platform
# endif
#endif

#if !defined(P_INCLUDE_FILE)
# define P_EXPAND(x) x
# define P_CONCAT(x,y) P_EXPAND(x)y
# define P_STR(x) #x
# define P_XSTR(x) P_STR(x)
# define P_INCLUDE_FILE P_XSTR(P_CONCAT(PLATFORM,P_CONCAT(/Pins_,P_CONCAT(PLATFORM,.h))))
#endif
#include P_INCLUDE_FILE

#ifndef SUPPORT_NONLINEAR_EXTRUSION
# define SUPPORT_NONLINEAR_EXTRUSION		1		// for now this is always enabled
#endif

#ifndef SUPPORT_WORKPLACE_COORDINATES
# define SUPPORT_WORKPLACE_COORDINATES		0
#endif

#ifndef SUPPORT_12864_LCD
# define SUPPORT_12864_LCD					0
#endif

#ifndef USE_CACHE
# define USE_CACHE							0
#endif

#endif // PINS_H__
