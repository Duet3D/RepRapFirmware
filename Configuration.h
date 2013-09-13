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

#define ABS_ZERO -273.15  // Celsius

#define INCH_TO_MM 25.4

#define HEAT_SAMPLE_TIME 0.5 // Seconds

#define STANDBY_INTERRUPT_RATE 2.0e-4 // Seconds

#define NUMBER_OF_PROBE_POINTS 3
#define Z_DIVE 5.0  // Height from which to probe the bed (mm)

// Webserver stuff

#define DEFAULT_PASSWORD "reprap"
#define DEFAULT_NAME "My RepRap 1"

#endif
