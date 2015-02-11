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
#define VERSION "1.00h-dc42"
#define DATE "2015-02-11"
#define AUTHORS "reprappro, dc42, zpl"

#define FLASH_SAVE_ENABLED	(1)

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

#define AUX_BAUD_RATE	(57600)

const unsigned int GcodeLength = 100;		// Maximum length of a G Code string that we handle

#define ABS_ZERO (-273.15)  // Celsius

#define INCH_TO_MM (25.4)

#define HEAT_SAMPLE_TIME (0.5) // Seconds

#define TEMPERATURE_CLOSE_ENOUGH (2.5) 		// Celsius
#define TEMPERATURE_LOW_SO_DONT_CARE (40.0)	// Celsius
#define HOT_ENOUGH_TO_EXTRUDE (160.0)       // Celsius
#define HOT_ENOUGH_TO_RETRACT (90.0)		// Celsius
#define TIME_TO_HOT (150.0)					// Seconds

// If temperatures fall outside this range, something nasty has happened.

#define MAX_BAD_TEMPERATURE_COUNT 6
#define BAD_LOW_TEMPERATURE -10.0
#define BAD_HIGH_TEMPERATURE 300.0

#define NUMBER_OF_PROBE_POINTS 5	// Maximum number of probe points
#define Z_DIVE 5.0  				// Height from which to probe the bed (mm)
#define TRIANGLE_0 -0.001			// Slightly less than 0 for point-in-triangle tests

#define SILLY_Z_VALUE -9999.0

// String lengths

#define STRING_LENGTH 1024
#define SHORT_STRING_LENGTH 40

#define FILENAME_LENGTH 100
#define GCODE_REPLY_LENGTH 2048

// Print estimation defaults
#define NOZZLE_DIAMETER 0.5						// Thickness of the nozzle
#define MAX_LAYER_SAMPLES 5						// Number of layer samples (except for first layer)
#define ESTIMATION_MIN_FILAMENT_USAGE 0.025		// Minimum per cent for filament usage estimation
#define FIRST_LAYER_SPEED_FACTOR 0.25			// First layer speed compared to others (only for layer-based estimation)

// Webserver stuff

#define DEFAULT_PASSWORD "reprap"
#define DEFAULT_NAME "My RepRap 1"
#define INDEX_PAGE "reprap.htm"
//#define MESSAGE_FILE "messages.txt"	// currently unused
#define FOUR04_FILE "html404.htm"
#define CONFIG_FILE "config.g" 			// The file that sets the machine's parameters
#define DEFAULT_FILE "default.g"				// If the config file isn't found
#define HOME_X_G "homex.g"
#define HOME_Y_G "homey.g"
#define HOME_Z_G "homez.g"
#define HOME_ALL_G "homeall.g"
#define HOME_DELTA_G "homedelta.g"
#define SET_BED_EQUATION "bed.g"
#define PAUSE_G "pause.g"
#define RESUME_G "resume.g"

#define LIST_SEPARATOR ':'						// Lists in G Codes
#define FILE_LIST_SEPARATOR ','					// Put this between file names when listing them
#define FILE_LIST_BRACKET '"'					// Put these round file names when listing them

#define LONG_TIME 300.0 // Seconds

#define EOF_STRING "<!-- **EoF** -->"           // For HTML uploads

#define FLASH_LED 'F' 							// Type byte of a message that is to flash an LED; the next two bytes define
                      	  	  	  	  	  	  	// the frequency and M/S ratio.
#define DISPLAY_MESSAGE 'L'  					// Type byte of a message that is to appear on a local display; the L is
                             	 	 	 	 	// not displayed; \f and \n should be supported.
#define HOST_MESSAGE 'H' 						// Type byte of a message that is to be sent to the host via USB; the H is not sent.
#define WEB_MESSAGE 'W'							// Type byte of message that is to be sent to the web
#define WEB_ERROR_MESSAGE 'E'					// Type byte of message that is to be sent to the web - flags an error
#define BOTH_MESSAGE 'B'						// Type byte of message that is to be sent to the web & host
#define BOTH_ERROR_MESSAGE 'A'					// Type byte of message that is to be sent to the web & host - flags an error
#define DEBUG_MESSAGE 'D'						// Type byte of debug message to send in blocking mode to USB

#endif
