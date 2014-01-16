/****************************************************************************************************

RepRapFirmware - Configuration

This is where all machine-independent configuration and other definitions are set up.  Nothing that
depends on any particular RepRap, RepRap component, or RepRap controller  should go in here.  Define 
machine-dependent things in Platform.h

-----------------------------------------------------------------------------------------------------

Version 0.1

18 November 2012

Adrian Bowyer
RepRap Professional Ltd
http://reprappro.com

Licence: GPL

****************************************************************************************************/

#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#define NAME "RepRapFirmware"
#define VERSION "0.57a"
#define DATE "2014-01-16"
#define LAST_AUTHOR "reprappro.com"

// Other firmware that we might switch to be compatible with.

enum Compatibility
{
	me = 0,
	reprapFirmware = 1,
	marlin = 2,
	teacup = 3,
	sprinter = 4,
	repetier = 5
};

// Some numbers...

#define ABS_ZERO -273.15  // Celsius

#define INCH_TO_MM 25.4

#define HEAT_SAMPLE_TIME 0.5 // Seconds

#define TEMPERATURE_CLOSE_ENOUGH 5.0 // Celsius
#define TEMPERATURE_LOW_SO_DONT_CARE 40.0 // Celsius

// If temperatures fall outside this range, something
// nasty has happened.

#define BAD_LOW_TEMPERATURE -30.0
#define BAD_HIGH_TEMPERATURE 300.0
#define MAX_BAD_TEMPERATURE_COUNT 6

#define STANDBY_INTERRUPT_RATE 2.0e-4 // Seconds

#define NUMBER_OF_PROBE_POINTS 4
#define Z_DIVE 5.0  // Height from which to probe the bed (mm)

#define SILLY_Z_VALUE -9999.0

// Webserver stuff

//#define NETWORK true // Set true to turn the ethernet on

#define DEFAULT_PASSWORD "reprap"
#define DEFAULT_NAME "My RepRap 1"
#define INDEX_PAGE "reprap.htm"
#define MESSAGE_FILE "messages.txt"
#define FOUR04_FILE "html404.htm"
#define CONFIG_FILE "config.g" // The file that sets the machine's parameters
#define HOME_X_G "homex.g"
#define HOME_Y_G "homey.g"
#define HOME_Z_G "homez.g"
#define HOME_ALL_G "homeall.g"

#define LONG_TIME 300.0 // Seconds

#define EOF_STRING "<!-- **EoF** -->"


#endif
