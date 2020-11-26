/*
 * PanelDueUpdater.cpp
 *
 *  Created on: 25 Nov 2020
 *      Author: manuel
 */

#include <Comms/PanelDueUpdater.h>
#include "Platform.h"

constexpr MessageType statusMessages = (MessageType) (UsbMessage | HttpMessage | TelnetMessage);
#if !ALLOW_OTHER_AUX
 constexpr uint32_t serialChannel = 1;
#endif

class AuxSerialPort : public SerialPort
{
public:
	AuxSerialPort(UARTClass& uartClass) : uart(uartClass), _timeout(0) {}
	~AuxSerialPort() {}

	bool open(int baud = 115200,
			  int data = 8,
			  SerialPort::Parity parity = SerialPort::ParityNone,
			  SerialPort::StopBit stop = SerialPort::StopBitOne) noexcept { return true; }
	void close() noexcept {}

	bool isUsb() noexcept { return false; }

	int read(uint8_t* data, int size) noexcept;
	int write(const uint8_t* data, int size) noexcept { return this->uart.write(data, size); }
	int get() noexcept;
	int put(int c) noexcept { return this->uart.write(c); }

	bool timeout(int millisecs) noexcept { _timeout = millisecs; return true; }
	void flush() noexcept { this->uart.flush(); }
	void setDTR(bool dtr) noexcept {}
	void setRTS(bool rts) noexcept {}
private:
	UARTClass& uart;
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
	DebugObserver() {}
	virtual ~DebugObserver() {}

	virtual void onStatus(const char *message, ...) noexcept;
	virtual void onProgress(int num, int div) noexcept;
};

void DebugObserver::onStatus(const char *message, ...) noexcept
{
	va_list ap;

	va_start(ap, message);
	reprap.GetPlatform().MessageF(statusMessages, message, ap);
	va_end(ap);
}

void DebugObserver::onProgress(int num, int div) noexcept
{
#if DEBUG_BOSSA
	debugPrintf("%d%% (%d/%d pages)\n", num * 100 / div, num, div);
#endif
}

PanelDueUpdater::PanelDueUpdater() noexcept
	:
#if ALLOW_OTHER_AUX
		serialChannel(NumSerialChannels+1) ,
#endif
		currentBaudRate(0)
		, samba(nullptr)
		, serialPort(nullptr)
		, device(nullptr)
		, flasherObserver(nullptr)
		, flasher(nullptr)
		, offset(0)
		, erasedAndResetAt(0)
		, state(FlashState::idle)
{
}

PanelDueUpdater::~PanelDueUpdater() noexcept
{
	delete samba;
	delete serialPort;
	delete device;
	delete flasherObserver;
	delete flasher;
}

void PanelDueUpdater::Start(const uint32_t serialChan) noexcept
{
	if (state != FlashState::idle)
	{
		return;
	}
#if ALLOW_OTHER_AUX
	serialChannel = serialChan;
#endif
	state = FlashState::eraseAndReset;
}

void PanelDueUpdater::Spin() noexcept
{
	switch (state)
	{
	case FlashState::eraseAndReset:
		reprap.GetPlatform().Message(statusMessages, "Erase and reset PanelDue\n");
		reprap.GetPlatform().AppendAuxReply(serialChannel-1, panelDueCommandEraseAndReset, true);
		state = FlashState::waitAfterEraseAndReset;
		erasedAndResetAt = millis();
		break;

	case FlashState::waitAfterEraseAndReset:
		if (millis() - erasedAndResetAt > WaitMsAfterEraseAndReset)
		{
			state = FlashState::setup;
		}
		break;

	case FlashState::setup:
		try
		{
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
			samba->setDebug(DEBUG_BOSSA);

			serialPort = new AuxSerialPort(*auxPort);
			samba->connect(serialPort);

			device = new Device(*samba);
			device->create();

			flasherObserver = new DebugObserver();

			flasher = new Flasher(*samba, *device, *flasherObserver);
			state = FlashState::unlock;
		}
		catch (GCodeException& ex)
		{
			HandleException(ex);
		}
		break;

	case FlashState::unlock:
		try
		{
			flasher->lock(false);
			state = FlashState::bossaErase;
		}
		catch (GCodeException& ex)
		{
			HandleException(ex);
		}
		break;

	case FlashState::bossaErase:
		try
		{
			flasher->erase(0);
			state = FlashState::write;
		}
		catch (GCodeException& ex)
		{
			HandleException(ex);
		}
		break;

	case FlashState::write:
		try
		{
			bool done = flasher->write(nullptr /* file is hard-coded for now */, offset);
			if (done)
			{
				offset = 0;						// Reset it for verification
				state = FlashState::verify;
			}
		}
		catch (GCodeException& ex)
		{
			HandleException(ex);
		}
		break;

	case FlashState::verify:
		try
		{
			uint32_t pageErrors;
			uint32_t totalErrors;
			bool done = flasher->verify(nullptr /* file is hard-coded for now */, pageErrors, totalErrors, offset);
			if (done && pageErrors == 0)
			{
				state = FlashState::writeOptions;
			}
			else if (pageErrors > 0)
			{
				reprap.GetPlatform().MessageF(ErrorMessage, "Verify failed: Page errors: %" PRIu32 " - Byte errors: %" PRIu32 "\n", pageErrors, totalErrors);
				state = FlashState::done;
			}
		}
		catch (GCodeException& ex)
		{
			HandleException(ex);
		}
		break;

	case FlashState::writeOptions:
		try
		{
			Flash* flash = device->getFlash();
            flash->setBootFlash(true);
			flash->writeOptions();
			state = FlashState::bossaReset;
		}
		catch (GCodeException& ex)
		{
			HandleException(ex);
		}
		break;

	case FlashState::bossaReset:
		try
		{
			reprap.GetPlatform().Message(statusMessages, "Restarting PanelDue\n");
			device->reset();
			state = FlashState::done;
		}
		catch (GCodeException& ex)
		{
			HandleException(ex);
		}
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
#if ALLOW_OTHER_AUX
			serialChannel = NumSerialChannels+1;
#endif
			currentBaudRate = 0;

			// Delete all objects we new'd
			delete samba;
			samba = nullptr;

			delete serialPort;
			serialPort = nullptr;

			delete device;
			device = nullptr;

			delete flasherObserver;
			flasherObserver = nullptr;

			delete flasher;
			flasher = nullptr;

			offset = 0;
			erasedAndResetAt = 0;
			state = FlashState::idle;
		}
		break;

	default:
		break;

	}
}

void PanelDueUpdater::HandleException(GCodeException& ex) noexcept
{
	String<StringLength100> errorMessage;
	ex.GetMessage(errorMessage.GetRef(), nullptr);
	reprap.GetPlatform().MessageF(ErrorMessage, "%s", errorMessage.c_str());
	state = FlashState::done;
}

UARTClass* PanelDueUpdater::GetAuxPort() noexcept
{
	return
#if ALLOW_OTHER_AUX
			serialChannel == 0 || serialChannel > NumSerialChannels ? nullptr :
# ifdef SERIAL_AUX2_DEVICE
			serialChannel == 2 ? &SERIAL_AUX2_DEVICE :
# endif
#endif
			&SERIAL_AUX_DEVICE;
}
