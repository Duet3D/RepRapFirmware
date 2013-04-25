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

class RepRap;
class Platform;
class Move;
class Heat;
class GCodes;
class Webserver;

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

#define PASSWORD_PAGE "passwd.php"
#define INDEX_PAGE "reprap.htm"
#define PRINT_PAGE "print.php"
#define MESSAGE_FILE "messages.php"
#define MESSAGE_TEMPLATE "messages.txt"
#define FOUR04_FILE "html404.htm"
#define KO_START "rr_"
#define KO_FIRST 3
#define STRING_LENGTH 1000
#define PHP_TAG_LENGTH 200
#define POST_LENGTH 200
#define PHP_IF 1
#define PHP_ECHO 2
#define PHP_PRINT 3
#define NO_PHP 99
#define START_FEED_RATE 200

#endif
