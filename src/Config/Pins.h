#ifndef PINS_H__
#define PINS_H__

// Load Pins_<platform>.h

#if !defined(PLATFORM)
# if defined(__SAM4E8E__)
#  define PLATFORM DuetNG
# elif defined(__SAME70Q20B__) || defined(__SAME70Q21B__) || defined(__SAMV71Q20B__)
#  if defined(DUET3_MB6HC)
#   define PLATFORM Duet3_MB6HC
#   define DUET3		1
#  elif defined(DUET3_MB6XD)
#   define PLATFORM Duet3_MB6XD
#   define DUET3		1
#  else
#   error Unknown platform
#  endif
# elif defined(DUET_M)
#  define PLATFORM DuetM
# elif defined(PCCB)
#  define PLATFORM Pccb
# elif defined(DUET3MINI_V04)
#  define DUET3MINI		1
#  define PLATFORM Duet3Mini
# elif defined(FMDC_V02)
#  define DUET3MINI		1
#  define PLATFORM FMDC
# elif defined(__LPC17xx__)
#  define PLATFORM LPC
# else
#  error Unknown platform
# endif
#endif

#if !defined(P_INCLUDE_FILE)
# define P_EXPAND(x) x
# define P_CONCAT(x,y) P_EXPAND(x)y
# define P_STR(x) #x
# define P_XSTR(x) P_STR(x)
# define P_INCLUDE_FILE P_XSTR(P_CONCAT(Pins_,P_CONCAT(PLATFORM,.h)))
#endif

#include P_INCLUDE_FILE

// Apply default values to anything not configured
#ifndef SUPPORT_NONLINEAR_EXTRUSION
# define SUPPORT_NONLINEAR_EXTRUSION		1		// for now this is always enabled
#endif

#ifndef SUPPORT_WORKPLACE_COORDINATES
# define SUPPORT_WORKPLACE_COORDINATES		0
#endif

#ifndef SUPPORT_COORDINATE_ROTATION
# define SUPPORT_COORDINATE_ROTATION	SUPPORT_WORKPLACE_COORDINATES
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

#ifndef SUPPORT_LED_STRIPS
# define SUPPORT_LED_STRIPS		0
#endif

#ifndef SUPPORT_SPI_SENSORS
# define SUPPORT_SPI_SENSORS	1
#endif

#define HAS_AUX_DEVICES			(defined(SERIAL_AUX_DEVICE))		// if SERIAL_AUX_DEVICE is defined then we have one or more aux devices

#ifndef SUPPORT_PANELDUE_FLASH
# define SUPPORT_PANELDUE_FLASH	HAS_AUX_DEVICES
#endif

#ifndef ALLOW_ARBITRARY_PANELDUE_PORT
# define ALLOW_ARBITRARY_PANELDUE_PORT (0)
#endif

#ifndef USE_CACHE
# define USE_CACHE				0
#endif

#ifndef USE_MPU
# define USE_MPU				0
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

#ifndef VARIABLE_NUM_DRIVERS
# define VARIABLE_NUM_DRIVERS	0
#endif

#ifndef SUPPORT_CAN_EXPANSION
# define SUPPORT_CAN_EXPANSION	0
#endif

#ifndef DUAL_CAN
# define DUAL_CAN				0
#endif

#ifndef SUPPORT_OBJECT_MODEL
# define SUPPORT_OBJECT_MODEL	0
#endif

#ifndef TRACK_OBJECT_NAMES
# define TRACK_OBJECT_NAMES		0
#endif

#define HAS_SMART_DRIVERS		(SUPPORT_TMC2660 || SUPPORT_TMC22xx || SUPPORT_TMC51xx)
#ifndef HAS_STALL_DETECT
# define HAS_STALL_DETECT		(SUPPORT_TMC2660 || SUPPORT_TMC51xx)
#endif

#ifndef SUPPORT_SLOW_DRIVERS
# define SUPPORT_SLOW_DRIVERS	1
#endif

#ifndef HAS_12V_MONITOR
# define HAS_12V_MONITOR		0
# define ENFORCE_MIN_V12		0
#endif

#if !HAS_VOLTAGE_MONITOR
# define ENFORCE_MAX_VIN		0
#endif

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

#ifndef HAS_RTOSPLUSTCP_NETWORKING
# define HAS_RTOSPLUSTCP_NETWORKING    0
#endif

#ifndef HAS_ESP32_NETWORKING
# define HAS_ESP32_NETWORKING    0
#endif

#define HAS_NETWORKING			(HAS_LWIP_NETWORKING || HAS_WIFI_NETWORKING || HAS_W5500_NETWORKING || HAS_RTOSPLUSTCP_NETWORKING || HAS_ESP32_NETWORKING)

#ifndef SUPPORT_HTTP
# define SUPPORT_HTTP			HAS_NETWORKING
#endif

#ifndef SUPPORT_FTP
# define SUPPORT_FTP			HAS_NETWORKING
#endif

#ifndef SUPPORT_TELNET
# define SUPPORT_TELNET			HAS_NETWORKING
#endif

#ifndef HAS_SBC_INTERFACE
# define HAS_SBC_INTERFACE		0
#endif

#ifndef HAS_MASS_STORAGE
# define HAS_MASS_STORAGE		1
#endif

#ifndef HAS_EMBEDDED_FILES
# define HAS_EMBEDDED_FILES		0
#endif

#if !HAS_MASS_STORAGE && !HAS_SBC_INTERFACE
# if SUPPORT_12864_LCD
#  error "12864 LCD support requires mass storage or SBC interface"
# endif
#endif

#if !HAS_MASS_STORAGE
# if SUPPORT_FTP
#  error "FTP support requires mass storage"
# endif
# if SUPPORT_SCANNER
#  error "Scanner support requires mass storage"
# endif
#endif

#ifndef SUPPORT_ASYNC_MOVES
# define SUPPORT_ASYNC_MOVES	0
#endif

#ifndef ALLOCATE_DEFAULT_PORTS
# define ALLOCATE_DEFAULT_PORTS	0
#endif

#ifndef SUPPORT_ACCELEROMETERS
# define SUPPORT_ACCELEROMETERS	0
#endif

// Optional kinematics support, to allow us to reduce flash memory usage
#ifndef SUPPORT_LINEAR_DELTA
# define SUPPORT_LINEAR_DELTA	1
#endif

#ifndef SUPPORT_ROTARY_DELTA
# define SUPPORT_ROTARY_DELTA	1
#endif

#ifndef SUPPORT_POLAR
# define SUPPORT_POLAR			1
#endif

#ifndef SUPPORT_SCARA
# define SUPPORT_SCARA			1
#endif

#ifndef SUPPORT_FIVEBARSCARA
# define SUPPORT_FIVEBARSCARA	1
#endif

#ifndef SUPPORT_HANGPRINTER
# define SUPPORT_HANGPRINTER	1
#endif

// We must define MCU_HAS_UNIQUE_ID as either 0 or 1 so we can use it in maths
#if SAM4E || SAM4S || SAME70 || SAME5x
# define MCU_HAS_UNIQUE_ID		1
#else
# define MCU_HAS_UNIQUE_ID		0
#endif

#if SAME70 || SAME5x
# define MCU_HAS_TRUERANDOM	1
#else
# define MCU_HAS_TRUERANDOM	0
#endif

// Define SUPPORT_REMOTE_COMMANDS according to whether this hardware accepts commands over CAN
#define SUPPORT_REMOTE_COMMANDS		(SUPPORT_CAN_EXPANSION && !defined(DUET3_ATE))
#define USE_REMOTE_INPUT_SHAPING	(0)		//TODO temporary!!!

#endif // PINS_H__
