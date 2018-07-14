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
# elif defined(DUET_M)
#  define PLATFORM DuetM
# elif defined(PCCB)
#  define PLATFORM Pccb
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

// Apply default values to anything not configured
#ifndef SUPPORT_NONLINEAR_EXTRUSION
# define SUPPORT_NONLINEAR_EXTRUSION		1		// for now this is always enabled
#endif

#ifndef SUPPORT_WORKPLACE_COORDINATES
# define SUPPORT_WORKPLACE_COORDINATES		0
#endif

#ifndef SUPPORT_12864_LCD
# define SUPPORT_12864_LCD		0
#endif

#ifndef USE_CACHE
# define USE_CACHE				0
#endif

#ifndef SUPPORT_TMC2660
# define SUPPORT_TMC2660		0
#endif

#ifndef SUPPORT_TMC22xx
# define SUPPORT_TMC22xx		0
#endif

#define HAS_SMART_DRIVERS		(SUPPORT_TMC2660 || SUPPORT_TMC22xx)
#define HAS_STALL_DETECT		SUPPORT_TMC2660

#define HAS_UNIFIED_NETWORKING	(HAS_LWIP_NETWORKING || HAS_WIFI_NETWORKING || HAS_W5500_NETWORKING)

#if SUPPORT_DHT_SENSOR && !defined(RTOS)
# error DHT sensor support requires RTOS
#endif

#endif // PINS_H__
