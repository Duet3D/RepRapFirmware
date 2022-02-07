/*
 * PanelDueUpdater.h
 *
 *  Created on: 25 Nov 2020
 *      Author: manuel
 */

#ifndef SRC_COMMS_PANELDUEUPDATER_H_
#define SRC_COMMS_PANELDUEUPDATER_H_

#include <RepRapFirmware.h>

#if SUPPORT_PANELDUE_FLASH

#include <General/NamedEnum.h>

#include <AsyncSerial.h>

#include <bossa/Samba.h>
#include <bossa/Device.h>
#include <bossa/Flasher.h>
#include <bossa/SerialPort.h>

constexpr uint32_t RequiredBaudRate = 115200;

constexpr const char *_ecv_array panelDueCommandEraseAndReset	= "{\"controlCommand\":\"eraseAndReset\"}\n";
constexpr const char *_ecv_array panelDueCommandReset 		= "{\"controlCommand\":\"reset\"}\n";

constexpr uint16_t WaitMsAfterEraseAndReset = 2000;			// How long to wait in ms after eraseAndReset

class PanelDueUpdater {
public:
	PanelDueUpdater() noexcept;
	virtual ~PanelDueUpdater() noexcept;
	void Spin() noexcept;
	void Start(const StringRef& filenameRef, const uint32_t serialChan = 1) noexcept;
	bool Idle() const noexcept { return state.RawValue() == FlashState::idle; }

private:
	NamedEnum(FlashState, uint8_t,
		idle,
		eraseAndReset,
		waitAfterEraseAndReset,
		setup,
		bossaUnlock,
		bossaErase,
		bossaWrite,
		bossaVerify,
		bossaWriteOptions,
		bossaReset,
		done
	);

	uint8_t serialChannel;
	size_t currentBaudRate;
	Samba* samba;
	SerialPort *_ecv_from serialPort;
	Device* device;
	FlasherObserver *_ecv_from flasherObserver;
	Flasher* flasher;
	AsyncSerial::InterruptCallbackFn currentInterruptCallbackFn;
	uint32_t offset;
	uint32_t erasedAndResetAt;
	FlashState state;
	FileStore *firmwareFile;

	AsyncSerial* GetAuxPort() noexcept;
};

#endif	// HAS_AUX_DEVICES

#endif /* SRC_COMMS_PANELDUEUPDATER_H_ */
