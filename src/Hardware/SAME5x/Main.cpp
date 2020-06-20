#include <AtmelStart_SAME5x/atmel_start.h>
#include <AtmelStart_SAME5x/Config/peripheral_clk_config.h>

#include "RepRapFirmware.h"
#include "Tasks.h"

int main(void)
{
	atmel_start_init();								// Initialize MCU, drivers and middleware
	AppMain();
}

// End
