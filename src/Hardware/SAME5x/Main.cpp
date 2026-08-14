#include <CoreIO.h>
#include <hri_oscctrl_e54.h>
#include <hri_gclk_e54.h>

void AppInit() noexcept
{
}

// Return the XOSC frequency in MHz
unsigned int AppGetXoscFrequency() noexcept
{
	return 25;
}

// Return the XOSC number
unsigned int AppGetXoscNumber() noexcept
{
#ifdef INDX
	return 0;
#else
	return 1;
#endif
}

// End
