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
#define VERSION "0.1"
#define DATE "2012-11-18"
#define LAST_AUTHOR "reprappro.com"

#define ABS_ZERO -273.15

#define INCH_TO_MM 25.4

#define FLASH_LED 'F' // Type byte of a message that is to flash an LED; the next two bytes define 
                      // the frequency and M/S ratio.
#define DISPLAY_MESSAGE 'L'  // Type byte of a message that is to appear on a local display; the L is 
                             // not displayed; \f and \n should be supported.
#define HOST_MESSAGE 'H' // Type byte of a message that is to be sent to the host; the H is not sent.


// Webserver stuff

#define DEFAULT_PASSWORD "reprap"

#define DEFAULT_NAME "My RepRap 1"

#define CLIENT_CLOSE_DELAY 1000 // Microseconds to wait after serving a page

#define INDEX_PAGE "reprap.htm"
#define PRINT_PAGE "print.php"
#define MESSAGE_FILE "messages.txt"
#define FOUR04_FILE "html404.htm"
#define KO_START "rr_"
#define KO_FIRST 3
#define STRING_LENGTH 1000
#define POST_LENGTH 200
#define START_FEED_RATE 200

// The axes etc in a GCode

#define GCODE_LETTERS {'X', 'Y', 'Z', 'E', 'F' }

#endif
