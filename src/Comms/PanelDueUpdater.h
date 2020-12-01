/*
 * PanelDueUpdater.h
 *
 *  Created on: 25 Nov 2020
 *      Author: manuel
 */

#ifndef SRC_COMMS_PANELDUEUPDATER_H_
#define SRC_COMMS_PANELDUEUPDATER_H_

#include <RepRap.h>

#ifdef DUET3MINI			// if using CoreN2G
# include <SAME5x_C21/Uart.h>
# define UARTClass	Uart
#else						// using CoreNG
# include <UARTClass.h>
#endif

#define DEBUG_BOSSA (0)
#define ALLOW_OTHER_AUX (0) // If we ever decide to allow PanelDue on something else than AUX reenable this

#include <bossa/Samba.h>
#include <bossa/Device.h>
#include <bossa/Flasher.h>
#include <bossa/SerialPort.h>

constexpr uint32_t RequiredBaudRate = 115200;

constexpr const char * panelDueCommandEraseAndReset	= "{\"controlCommand\":\"eraseAndReset\"}\n";
constexpr const char * panelDueCommandReset 		= "{\"controlCommand\":\"reset\"}\n";

constexpr uint32_t WaitMsAfterEraseAndReset = 1000;			// How long to wait in ms after eraseAndReset

class PanelDueUpdater {
public:
	PanelDueUpdater() noexcept;
	virtual ~PanelDueUpdater() noexcept;
	void Spin() noexcept;
	void Start(const uint32_t serialChan = 1) noexcept;
	bool Idle() const noexcept { return state == FlashState::idle; }

    // For now fix the filename here
    constexpr static const char* const firmwareFilename = DEFAULT_SYS_DIR PANEL_DUE_FIRMWARE_FILE;

private:
#if ALLOW_OTHER_AUX
	size_t serialChannel;
#endif
	size_t currentBaudRate;
	Samba* samba;
	SerialPort* serialPort;
	Device* device;
	FlasherObserver* flasherObserver;
	Flasher* flasher;
	UARTClass::InterruptCallbackFn currentInterruptCallbackFn;
	uint32_t offset;
	uint32_t erasedAndResetAt;

	enum class FlashState
	{
		idle,
		eraseAndReset,
		waitAfterEraseAndReset,
		setup,
		unlock,
		bossaErase,
		write,
		verify,
		writeOptions,
		bossaReset,
		done
	};
	FlashState state;

	UARTClass* GetAuxPort() noexcept;
};

#endif /* SRC_COMMS_PANELDUEUPDATER_H_ */
