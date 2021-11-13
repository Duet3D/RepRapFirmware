/*
 * PanelDueUpdater.cpp
 *
 *  Created on: 25 Nov 2020
 *      Author: manuel
 */

#include "PanelDueUpdater.h"

#if HAS_AUX_DEVICES

#include <Platform/Platform.h>
#include <Platform/RepRap.h>

class AuxSerialPort : public SerialPort
{
public:
	AuxSerialPort(AsyncSerial& uartClass) noexcept : uart(uartClass), _timeout(0) {}
	~AuxSerialPort() {}

	bool open(int baud = 115200,
			  int data = 8,
			  SerialPort::Parity parity = SerialPort::ParityNone,
			  SerialPort::StopBit stop = SerialPort::StopBitOne) noexcept override { return true; }
	void close() noexcept override {}

	bool isUsb() noexcept override { return false; }

	int read(uint8_t* data, int size) noexcept override;
	int write(const uint8_t* data, int size) noexcept override { return this->uart.write(data, size); }
	int get() noexcept override;
	int put(int c) noexcept override { return this->uart.write(c); }

	bool timeout(int millisecs) noexcept override { _timeout = millisecs; return true; }
	void flush() noexcept override { this->uart.flush(); }
	void setDTR(bool dtr) noexcept override {}
	void setRTS(bool rts) noexcept override {}

private:
	AsyncSerial& uart;
	int _timeout;
};

int	AuxSerialPort::get() noexcept
{
	uint8_t byte;

	if (read(&byte, 1) != 1)
		return -1;

	return byte;
}

int AuxSerialPort::read(uint8_t* data, int size) noexcept
{
	const uint32_t start = millis();
	int read = 0;
	do
	{
		const int readNow = (int) this->uart.readBytes((uint8_t*)data+read, size-read);
		if (readNow >= 0)
		{
			read += readNow;
		}
	} while (read < size && (int) (millis() - start) < _timeout);
	return read;
}

class DebugObserver : public FlasherObserver
{
public:
	DebugObserver() noexcept : lastPercentage(0) {}
	virtual ~DebugObserver() {}

	void onStatus(const char *message, ...) noexcept override;
	void onProgress(int num, int div) noexcept override;
    void Reset() noexcept override { lastPercentage = 0; };
private:
	uint8_t lastPercentage;
};

void DebugObserver::onStatus(const char *message, ...) noexcept
{
	va_list ap;

	va_start(ap, message);
	reprap.GetPlatform().MessageV(GenericMessage, message, ap);
	va_end(ap);
}

void DebugObserver::onProgress(int num, int div) noexcept
{
	uint8_t percentage = (uint8_t)(num * 100 / div);
	if (percentage == lastPercentage + 20) {
		lastPercentage = percentage;
		reprap.GetPlatform().MessageF(GenericMessage, "Progress: %d%%\n", percentage);
	}
}

PanelDueUpdater::PanelDueUpdater() noexcept
	:
		serialChannel(NumSerialChannels+1)
		, currentBaudRate(0)
		, samba(nullptr)
		, serialPort(nullptr)
		, device(nullptr)
		, flasherObserver(nullptr)
		, flasher(nullptr)
		, offset(0)
		, erasedAndResetAt(0)
		, state(FlashState::idle)
		, firmwareFile(nullptr)
{
}

PanelDueUpdater::~PanelDueUpdater() noexcept
{
	DeleteObject(samba);
	DeleteObject(serialPort);
	DeleteObject(device);
	DeleteObject(flasherObserver);
	DeleteObject(flasher);
}

void PanelDueUpdater::Start(const StringRef& filenameRef, const uint32_t serialChan) noexcept
{
	if (state == FlashState::idle)
	{
		serialChannel = serialChan;
		const char * const filename = filenameRef.IsEmpty() ? PANEL_DUE_FIRMWARE_FILE : filenameRef.c_str();
		firmwareFile = reprap.GetPlatform().OpenFile(FIRMWARE_DIRECTORY, filename, OpenMode::read);
		if (firmwareFile == nullptr)
		{
			reprap.GetPlatform().MessageF(ErrorMessage, "Can't open file %s\n", filename);
			state = FlashState::done;
		}
		else if (firmwareFile->Length() > 256 * 1024)
		{
			reprap.GetPlatform().MessageF(ErrorMessage, "Firmware file %s is too large\n", filename);
			state = FlashState::done;
		}
		else
		{
			state = FlashState::eraseAndReset;
		}
	}
}

void PanelDueUpdater::Spin() noexcept
{
	try
	{
		switch (state.RawValue())
		{
		case FlashState::eraseAndReset:
			{
				reprap.GetPlatform().Message(GenericMessage, "Sending Erase-and-Reset command to PanelDue\n");

				// Since writing messages via AppendAuxReply is disabled while flashing we need to send it directly
				auto auxPort = GetAuxPort();
				auxPort->write('\n');			// Make sure the previous message is regarded as terminated by PanelDue
				auxPort->print(panelDueCommandEraseAndReset);
				auxPort->flush();
				state = FlashState::waitAfterEraseAndReset;
				erasedAndResetAt = millis();
			}
			break;

		case FlashState::waitAfterEraseAndReset:
			if (millis() - erasedAndResetAt > WaitMsAfterEraseAndReset)
			{
				state = FlashState::setup;
			}
			break;

		case FlashState::setup:
			{
				reprap.GetPlatform().Message(GenericMessage, "Establishing connection to PanelDue bootloader\n");

				// Verifying creates the Emergency Stop sequence so disable it here
				auto auxPort = GetAuxPort();
				currentInterruptCallbackFn = auxPort->SetInterruptCallback(nullptr);
				Platform& platform = reprap.GetPlatform();
				uint32_t baudRate = platform.GetBaudRate(serialChannel);

				// Make sure baud rate is set correctly
				if (baudRate != RequiredBaudRate)
				{
					currentBaudRate = baudRate;
					platform.SetBaudRate(serialChannel, RequiredBaudRate);
					platform.ResetChannel(serialChannel);
				}

				samba = new Samba();

				serialPort = new AuxSerialPort(*auxPort);
				samba->connect(serialPort);

				device = new Device(*samba);
				device->create();				// If this throws the bootloader did not answer

				flasherObserver = new DebugObserver();

				flasher = new Flasher(*samba, *device, *flasherObserver);
				state = FlashState::bossaUnlock;
			}
			break;

		case FlashState::bossaUnlock:
			reprap.GetPlatform().Message(GenericMessage, "Unlocking PanelDue flash memory\n");
			flasher->lock(false);
			state = FlashState::bossaErase;
			break;

		case FlashState::bossaErase:
			reprap.GetPlatform().Message(GenericMessage, "Erasing PanelDue flash memory\n");
			flasher->erase(0);
			state = FlashState::bossaWrite;
			break;

		case FlashState::bossaWrite:
			{
				bool done = flasher->write(firmwareFile, offset);
				if (done)
				{
					offset = 0;						// Reset it for verification
					firmwareFile->Seek(offset);
					state = FlashState::bossaVerify;
					flasherObserver->Reset();
				}
			}
			break;

		case FlashState::bossaVerify:
			{
				uint32_t pageErrors;
				uint32_t totalErrors;
				bool done = flasher->verify(firmwareFile, pageErrors, totalErrors, offset);
				if (done && pageErrors == 0)
				{
					state = FlashState::bossaWriteOptions;
				}
				else if (pageErrors > 0)
				{
					reprap.GetPlatform().MessageF(ErrorMessage, "Verify failed: Page errors: %" PRIu32 " - Byte errors: %" PRIu32 "\n", pageErrors, totalErrors);
					state = FlashState::done;
				}
			}
			break;

		case FlashState::bossaWriteOptions:
			{
				reprap.GetPlatform().Message(GenericMessage, "Writing PanelDue flash options\n");
				BossaFlash* flash = device->getFlash();
				flash->setBootFlash(true);
				flash->writeOptions();
				state = FlashState::bossaReset;
			}
			break;

		case FlashState::bossaReset:
			reprap.GetPlatform().Message(GenericMessage, "Restarting PanelDue\n");
			device->reset();
			GetAuxPort()->flush();
			state = FlashState::done;
			break;

		case FlashState::done:
			{
				// Restore previous baud rate
				if (currentBaudRate != 0)
				{
					Platform& platform = reprap.GetPlatform();
					platform.SetBaudRate(serialChannel, currentBaudRate);
					platform.ResetChannel(serialChannel);
				}

				// Restore the callback for the Emergency Stop sequence
				auto auxPort = GetAuxPort();
				auxPort->SetInterruptCallback(currentInterruptCallbackFn);
				currentInterruptCallbackFn = nullptr;
				serialChannel = NumSerialChannels+1;
				currentBaudRate = 0;

				// Delete all objects we new'd
				DeleteObject(samba);
				DeleteObject(serialPort);
				DeleteObject(device);
				DeleteObject(flasherObserver);
				DeleteObject(flasher);

				offset = 0;
				erasedAndResetAt = 0;

				if (firmwareFile != nullptr)
				{
					firmwareFile->Close();
					firmwareFile = nullptr;
				}

				state = FlashState::idle;
			}
			break;

		default:
			break;
		}
	}
	catch (GCodeException& ex)
	{
#if DEBUG_BOSSA
		String<StringLength100> errorMessage;
		ex.GetMessage(errorMessage.GetRef(), nullptr);
		reprap.GetPlatform().MessageF(ErrorMessage, "%s", errorMessage.c_str());
#endif
		if (state == FlashState::setup)
		{
			reprap.GetPlatform().Message(ErrorMessage, "Failed to communicate with PanelDue bootloader (no START signal received). Please try again or press the Erase and Reset switches on PanelDue.");
		}
		else
		{
			reprap.GetPlatform().MessageF(ErrorMessage, "Flashing PanelDue failed in step %s. Please try again.", state.ToString());
		}

		// Delete all objects we new'd
		DeleteObject(samba);
		DeleteObject(serialPort);
		DeleteObject(device);
		DeleteObject(flasherObserver);
		DeleteObject(flasher);

		state = FlashState::done;
	}
}

AsyncSerial* PanelDueUpdater::GetAuxPort() noexcept
{
	return
			serialChannel == 0 || serialChannel > NumSerialChannels ? nullptr :
#ifdef SERIAL_AUX2_DEVICE
			serialChannel == 2 ? &SERIAL_AUX2_DEVICE :
#endif
			&SERIAL_AUX_DEVICE;	// Channel 1
}

#endif

// End
