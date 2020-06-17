#include <AtmelStart_SAME5x/atmel_start.h>
#include <AtmelStart_SAME5x/Config/peripheral_clk_config.h>

#include "RepRapFirmware.h"
#include "Tasks.h"

extern "C" [[noreturn]] void __cxa_pure_virtual() noexcept { while (1); }

int main(void)
{
	atmel_start_init();								// Initialize MCU, drivers and middleware

	SystemCoreClock = CONF_CPU_FREQUENCY;			// FreeRTOS needs this to be set correctly because it uses it to set the systick reload value
	SystemPeripheralClock = CONF_CPU_FREQUENCY/2;

	AppMain();
}

// End
