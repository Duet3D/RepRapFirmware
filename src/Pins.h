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
# elif defined(__SAME70Q21__) || defined(__SAME70Q20B__) || defined(__SAME70Q21B__)
#  if defined(DUET3_V03)
#   define PLATFORM Duet3_V03
#  elif defined(DUET3_V05)
#   define PLATFORM Duet3_V05
#  elif defined(SAME70XPLD)
#   define PLATFORM SAME70xpld
#  else
#   error Unknown platform
#  endif
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

#ifndef SUPPORT_LASER
# define SUPPORT_LASER			0
#endif

#ifndef SUPPORT_IOBITS
# define SUPPORT_IOBITS			0
#endif

#ifndef SUPPORT_12864_LCD
# define SUPPORT_12864_LCD		0
#endif

#ifndef SUPPORT_DOTSTAR_LED
# define SUPPORT_DOTSTAR_LED	0
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

#ifndef SUPPORT_TMC51xx
# define SUPPORT_TMC51xx		0
#endif

#ifndef SUPPORT_CAN_EXPANSION
# define SUPPORT_CAN_EXPANSION	0
#endif

#ifndef SUPPORT_OBJECT_MODEL
# define SUPPORT_OBJECT_MODEL	0
#endif

#define HAS_SMART_DRIVERS		(SUPPORT_TMC2660 || SUPPORT_TMC22xx || SUPPORT_TMC51xx)
#define HAS_STALL_DETECT		(SUPPORT_TMC2660 || SUPPORT_TMC51xx)

// HAS_LWIP_NETWORKING refers to Lwip 2 support in the Networking folder, not legacy SAM3XA networking using Lwip 1
#ifndef HAS_LWIP_NETWORKING
# define HAS_LWIP_NETWORKING	0
#endif

#ifndef HAS_WIFI_NETWORKING
# define HAS_WIFI_NETWORKING	0
#endif

#ifndef HAS_W5500_NETWORKING
# define HAS_W5500_NETWORKING	0
#endif

// Boards that support legacy SAM3X Lwip 1 networking must define HAS_LEGACY_NETWORKING in their specific Pins_xxx.h file
#ifndef HAS_LEGACY_NETWORKING
# define HAS_LEGACY_NETWORKING	0
#endif

#ifndef HAS_RTOSPLUSTCP_NETWORKING
# define HAS_RTOSPLUSTCP_NETWORKING    0
#endif

#ifndef HAS_ESP32_NETWORKING
# define HAS_ESP32_NETWORKING    0
#endif

#define HAS_NETWORKING			(HAS_LWIP_NETWORKING || HAS_WIFI_NETWORKING || HAS_W5500_NETWORKING || HAS_LEGACY_NETWORKING || HAS_RTOSPLUSTCP_NETWORKING || HAS_ESP32_NETWORKING)

#ifndef SUPPORT_FTP
# define SUPPORT_FTP			HAS_NETWORKING
#endif

#ifndef SUPPORT_TELNET
# define SUPPORT_TELNET			HAS_NETWORKING
#endif

#if SUPPORT_DHT_SENSOR && !defined(RTOS)
# error DHT sensor support requires RTOS
#endif

#endif // PINS_H__
