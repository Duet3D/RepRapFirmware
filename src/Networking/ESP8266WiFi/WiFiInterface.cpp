/*
 * WiFiInterface.cpp
 *
 *  Created on: 27 Nov 2017
 *      Authors: Christian and David
 */


#include "WiFiInterface.h"

#if HAS_WIFI_NETWORKING

#include <Platform/Platform.h>
#include <Platform/RepRap.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Networking/HttpResponder.h>
#include <Networking/FtpResponder.h>
#include <Networking/TelnetResponder.h>
#include "WifiFirmwareUploader.h"
#include <General/IP4String.h>
#include "WiFiSocket.h"
#include <Cache.h>

static_assert(SsidLength == SsidBufferLength, "SSID lengths in NetworkDefs.h and MessageFormats.h don't match");

// Define exactly one of the following as 1, the other as zero

#if defined(DUET_NG)

# include <pmc/pmc.h>
# include <spi/spi.h>

// The PDC seems to be too slow to work reliably without getting transmit underruns, so we use the DMAC now.
# define USE_PDC			0		// use SAM4 peripheral DMA controller
# define USE_DMAC			1		// use SAM4 general DMA controller
# define USE_DMAC_MANAGER	0		// use SAMD/SAME DMA controller via DmacManager module
# define USE_XDMAC			0		// use SAME7 XDMA controller

#elif defined(DUET3_V03) || defined(SAME70XPLD)

# define USE_PDC			0		// use SAM4 peripheral DMA controller
# define USE_DMAC			0		// use SAM4 general DMA controller
# define USE_DMAC_MANAGER	0		// use SAMD/SAME DMA controller via DmacManager module
# define USE_XDMAC			1		// use SAME7 XDMA controller

#elif SAME5x

# include <DmacManager.h>
# include <Interrupts.h>
# include <Serial.h>
# include <AsyncSerial.h>

# define USE_PDC            0		// use SAM4 peripheral DMA controller
# define USE_DMAC           0 		// use SAM4 general DMA controller
# define USE_DMAC_MANAGER	1		// use SAMD/SAME DMA controller via DmacManager module
# define USE_XDMAC          0		// use SAME7 XDMA controller

// Compatibility with existing RRF Code
constexpr Pin APIN_ESP_SPI_MISO = EspMisoPin;
constexpr Pin APIN_ESP_SPI_SCK = EspSclkPin;
constexpr IRQn ESP_SPI_IRQn = WiFiSpiSercomIRQn;

#elif defined(__LPC17xx__)

# define USE_PDC            0		// use SAM4 peripheral DMA controller
# define USE_DMAC           0		// use SAM4 general DMA controller
# define USE_DMAC_MANAGER	0		// use SAMD/SAME DMA controller via DmacManager module
# define USE_XDMAC          0		// use SAME7 XDMA controller

// Compatibility with existing RRF Code
constexpr Pin APIN_ESP_SPI_MISO = SPI0_MOSI;
constexpr Pin APIN_ESP_SPI_SCK = SPI0_SCK;
constexpr SSPChannel ESP_SPI = SSP0;

#else
# error Unknown board
#endif

#if USE_PDC
# include "pdc/pdc.h"
#endif

#if USE_DMAC
# include "dmac/dmac.h"
#endif

#if USE_XDMAC
# include "xdmac/xdmac.h"
#endif

#if SAME5x
#elif !defined(__LPC17xx__)
# include "matrix/matrix.h"
#endif

const uint32_t WiFiResponseTimeoutMillis = 200;					// SPI timeout when when the ESP does not have to write to flash memory
const uint32_t WiFiTransferTimeoutMillis = 60;					// Christian measured this at 29 to 31ms when the ESP has to write to flash memory
const uint32_t WiFiWaitReadyMillis = 100;
const uint32_t WiFiStartupMillis = 300;
const uint32_t WiFiStableMillis = 100;

const unsigned int MaxHttpConnections = 4;

#if SAME5x

void SerialWiFiPortInit(AsyncSerial*) noexcept
{
	for (Pin p : WiFiUartSercomPins)
	{
		SetPinFunction(p, WiFiUartSercomPinsMode);
	}
}

void SerialWiFiPortDeinit(AsyncSerial*) noexcept
{
	for (Pin p : WiFiUartSercomPins)
	{
		pinMode(p, INPUT_PULLUP);								// just enable pullups on TxD and RxD pins
	}
}

#endif

// Static functions
static inline void DisableSpi() noexcept
{
#if SAME5x
	WiFiSpiSercom->SPI.CTRLA.reg &= ~SERCOM_SPI_CTRLA_ENABLE;
	while (WiFiSpiSercom->SPI.SYNCBUSY.reg & (SERCOM_SPI_SYNCBUSY_SWRST | SERCOM_SPI_SYNCBUSY_ENABLE)) { };
#else
	spi_disable(ESP_SPI);
#endif
}

static inline void EnableSpi()
{
#if SAME5x
	WiFiSpiSercom->SPI.CTRLA.reg |= SERCOM_SPI_CTRLA_ENABLE;
	while (WiFiSpiSercom->SPI.SYNCBUSY.reg & (SERCOM_SPI_SYNCBUSY_SWRST | SERCOM_SPI_SYNCBUSY_ENABLE)) { };
#else
	spi_enable(ESP_SPI);
#endif
}

// Clear the transmit and receive registers and put the SPI into slave mode, SPI mode 1
static inline void ResetSpi()
{
#if SAME5x
	WiFiSpiSercom->SPI.CTRLA.reg |= SERCOM_SPI_CTRLA_SWRST;
	while (WiFiSpiSercom->SPI.SYNCBUSY.reg & SERCOM_SPI_SYNCBUSY_SWRST) { };
	WiFiSpiSercom->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_CPHA | SERCOM_SPI_CTRLA_DIPO(3) | SERCOM_SPI_CTRLA_DOPO(0) | SERCOM_SPI_CTRLA_MODE(2);
	WiFiSpiSercom->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_RXEN | SERCOM_SPI_CTRLB_SSDE | SERCOM_SPI_CTRLB_PLOADEN;
	while (WiFiSpiSercom->SPI.SYNCBUSY.reg & SERCOM_SPI_SYNCBUSY_MASK) { };
	WiFiSpiSercom->SPI.CTRLC.reg = SERCOM_SPI_CTRLC_DATA32B;
#else
	spi_reset(ESP_SPI);				// this clears the transmit and receive registers and puts the SPI into slave mode
#endif
}

static void spi_dma_disable() noexcept;

#if !SAME5x
static bool spi_dma_check_rx_complete() noexcept;
#endif

#ifdef DUET3MINI

AsyncSerial *SerialWiFiDevice;
# define SERIAL_WIFI_DEVICE	(*SerialWiFiDevice)

# if !defined(SERIAL_WIFI_ISR0) || !defined(SERIAL_WIFI_ISR2) || !defined(SERIAL_WIFI_ISR3)
#  error SERIAL_WIFI_ISRn not defined
# endif

void SERIAL_WIFI_ISR0() noexcept
{
	SerialWiFiDevice->Interrupt0();
}

void SERIAL_WIFI_ISR2() noexcept
{
	SerialWiFiDevice->Interrupt2();
}

void SERIAL_WIFI_ISR3() noexcept
{
	SerialWiFiDevice->Interrupt3();
}

#endif

static volatile bool transferPending = false;
static WiFiInterface *wifiInterface;

#if 0
static void debugPrintBuffer(const char *msg, void *buf, size_t dataLength) noexcept
{
	const size_t MaxDataToPrint = 20;
	const uint8_t * const data = reinterpret_cast<const uint8_t *>(buf);
	debugPrintf("%s %02x %02x %02x %02x %04x %04x %08x",
		msg,
		data[0], data[1], data[2], data[3],
		*reinterpret_cast<const uint16_t*>(data + 4), *reinterpret_cast<const uint16_t*>(data + 6),
		*reinterpret_cast<const uint32_t*>(data + 8));
	if (dataLength != 0)
	{
		debugPrintf(" ");
		for (size_t i = 0; i < min<size_t>(dataLength, MaxDataToPrint); ++i)
		{
			debugPrintf("%02x", data[i + 12]);
		}
		if (dataLength > MaxDataToPrint)
		{
			debugPrintf("...");
		}
	}
	debugPrintf("\n");
}
#endif

#ifdef __LPC17xx__
# include "WiFiInterface_LPC.hpp"
#endif

static void EspTransferRequestIsr(CallbackParameter) noexcept
{
	wifiInterface->EspRequestsTransfer();
}

static inline void EnableEspInterrupt() noexcept
{
	attachInterrupt(EspDataReadyPin, EspTransferRequestIsr, InterruptMode::rising, CallbackParameter(nullptr));
}

static inline void DisableEspInterrupt() noexcept
{
	detachInterrupt(EspDataReadyPin);
}

/*-----------------------------------------------------------------------------------*/
// WiFi interface class

WiFiInterface::WiFiInterface(Platform& p) noexcept
	: platform(p), bufferOut(nullptr), bufferIn(nullptr), uploader(nullptr), espWaitingTask(nullptr),
	  ftpDataPort(0), closeDataPort(false),
	  requestedMode(WiFiState::disabled), currentMode(WiFiState::disabled), activated(false),
	  espStatusChanged(false), spiTxUnderruns(0), spiRxOverruns(0), serialRunning(false), debugMessageChars(0)
{
	wifiInterface = this;

	// Create the sockets
	for (WiFiSocket*& skt : sockets)
	{
		skt = new WiFiSocket(this);
	}

	for (size_t i = 0; i < NumProtocols; ++i)
	{
		portNumbers[i] = DefaultPortNumbers[i];
		protocolEnabled[i] = (i == HttpProtocol);
	}

	strcpy(actualSsid, "(unknown)");
	strcpy(wiFiServerVersion, "(unknown)");

#ifdef DUET3MINI
	SerialWiFiDevice = new AsyncSerial(WiFiUartSercomNumber, WiFiUartRxPad, 512, 512, SerialWiFiPortInit, SerialWiFiPortDeinit);
	SerialWiFiDevice->setInterruptPriority(NvicPriorityWiFiUartRx, NvicPriorityWiFiUartTx);
#else
	SERIAL_WIFI_DEVICE.setInterruptPriority(NvicPriorityWiFiUart);
#endif
}

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(_ret) OBJECT_MODEL_FUNC_BODY(WiFiInterface, _ret)

constexpr ObjectModelTableEntry WiFiInterface::objectModelTable[] =
{
	// These entries must be in alphabetical order
	{ "actualIP",			OBJECT_MODEL_FUNC(self->ipAddress),				ObjectModelEntryFlags::none },
	{ "firmwareVersion",	OBJECT_MODEL_FUNC(self->wiFiServerVersion),		ObjectModelEntryFlags::none },
	{ "gateway",			OBJECT_MODEL_FUNC(self->gateway),				ObjectModelEntryFlags::none },
	{ "mac",				OBJECT_MODEL_FUNC(self->macAddress),			ObjectModelEntryFlags::none },
	{ "state",				OBJECT_MODEL_FUNC(self->GetStateName()),		ObjectModelEntryFlags::none },
	{ "subnet",				OBJECT_MODEL_FUNC(self->netmask),				ObjectModelEntryFlags::none },
	{ "type",				OBJECT_MODEL_FUNC_NOSELF("wifi"),				ObjectModelEntryFlags::none },
};

constexpr uint8_t WiFiInterface::objectModelTableDescriptor[] = { 1, 7 };

DEFINE_GET_OBJECT_MODEL_TABLE(WiFiInterface)

#endif

void WiFiInterface::Init() noexcept
{
	interfaceMutex.Create("WiFi");

	// Make sure the ESP8266 is held in the reset state
	ResetWiFi();
	lastTickMillis = millis();

	SetIPAddress(DefaultIpAddress, DefaultNetMask, DefaultGateway);

	for (size_t i = 0; i < NumWiFiTcpSockets; ++i)
	{
		sockets[i]->Init(i);
	}

	currentSocket = 0;
}

GCodeResult WiFiInterface::EnableProtocol(NetworkProtocol protocol, int port, int secure, const StringRef& reply) noexcept
{
	if (secure != 0 && secure != -1)
	{
		reply.copy("Error: this firmware does not support TLS");
	}
	else if (protocol < NumProtocols)
	{
		const TcpPort portToUse = (port < 0) ? DefaultPortNumbers[protocol] : port;
		MutexLocker lock(interfaceMutex);

		if (portToUse != portNumbers[protocol] && GetState() == NetworkState::active)
		{
			// We need to shut down and restart the protocol if it is active because the port number has changed
			ShutdownProtocol(protocol);
			protocolEnabled[protocol] = false;
		}
		portNumbers[protocol] = portToUse;
		if (!protocolEnabled[protocol])
		{
			protocolEnabled[protocol] = true;
			if (GetState() == NetworkState::active)
			{
				StartProtocol(protocol);
				// mDNS announcement is done by the WiFi Server firmware
			}
		}
		ReportOneProtocol(protocol, reply);
		return GCodeResult::ok;
	}

	reply.copy("Invalid protocol parameter");
	return GCodeResult::error;
}

GCodeResult WiFiInterface::DisableProtocol(NetworkProtocol protocol, const StringRef& reply) noexcept
{
	if (protocol < NumProtocols)
	{
		MutexLocker lock(interfaceMutex);

		if (GetState() == NetworkState::active)
		{
			ShutdownProtocol(protocol);
		}
		protocolEnabled[protocol] = false;
		ReportOneProtocol(protocol, reply);
		return GCodeResult::ok;
	}

	reply.copy("Invalid protocol parameter");
	return GCodeResult::error;
}

void WiFiInterface::StartProtocol(NetworkProtocol protocol) noexcept
{
	MutexLocker lock(interfaceMutex);

	switch(protocol)
	{
	case HttpProtocol:
		SendListenCommand(portNumbers[protocol], protocol, MaxHttpConnections);
		break;

	case FtpProtocol:
		SendListenCommand(portNumbers[protocol], protocol, 1);
		break;

	case TelnetProtocol:
		SendListenCommand(portNumbers[protocol], protocol, 1);
		break;

	default:
		break;
	}
}

void WiFiInterface::ShutdownProtocol(NetworkProtocol protocol) noexcept
{
	MutexLocker lock(interfaceMutex);

	switch(protocol)
	{
	case HttpProtocol:
		StopListening(portNumbers[protocol]);
		TerminateSockets(portNumbers[protocol]);
		break;

	case FtpProtocol:
		StopListening(portNumbers[protocol]);
		TerminateSockets(portNumbers[protocol]);
		if (ftpDataPort != 0)
		{
			StopListening(ftpDataPort);
			TerminateSockets(ftpDataPort);
		}
		break;

	case TelnetProtocol:
		StopListening(portNumbers[protocol]);
		TerminateSockets(portNumbers[protocol]);
		break;

	default:
		break;
	}
}

// Report the protocols and ports in use
GCodeResult WiFiInterface::ReportProtocols(const StringRef& reply) const noexcept
{
	for (size_t i = 0; i < NumProtocols; ++i)
	{
		ReportOneProtocol(i, reply);
	}
	return GCodeResult::ok;
}

void WiFiInterface::ReportOneProtocol(NetworkProtocol protocol, const StringRef& reply) const noexcept
{
	if (protocolEnabled[protocol])
	{
		reply.lcatf("%s is enabled on port %u", ProtocolNames[protocol], portNumbers[protocol]);
	}
	else
	{
		reply.lcatf("%s is disabled", ProtocolNames[protocol]);
	}
}

NetworkProtocol WiFiInterface::GetProtocolByLocalPort(TcpPort port) const noexcept
{
	if (port == ftpDataPort)
	{
		return FtpDataProtocol;
	}

	for (NetworkProtocol p : ARRAY_INDICES(portNumbers))
	{
		if (portNumbers[p] == port)
		{
			return p;
		}
	}

	return AnyProtocol;
}

// This is called at the end of config.g processing.
// Start the network if it was enabled
void WiFiInterface::Activate() noexcept
{
	if (!activated)
	{
		activated = true;

		bufferOut = new MessageBufferOut;
		bufferIn = new MessageBufferIn;
#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
		uploader = new WifiFirmwareUploader(SERIAL_WIFI_DEVICE, *this);
#endif
		if (requestedMode != WiFiState::disabled)
		{
			Start();
		}
		else
		{
			platform.Message(UsbMessage, "WiFi is disabled.\n");
		}
	}
}

void WiFiInterface::Exit() noexcept
{
	Stop();
}

// Get the network state into the reply buffer, returning true if there is some sort of error
GCodeResult WiFiInterface::GetNetworkState(const StringRef& reply) noexcept
{
	switch (GetState())
	{
	case NetworkState::disabled:
		reply.copy("WiFi module is disabled");
		break;
	case NetworkState::starting1:
	case NetworkState::starting2:
		reply.copy("WiFi module is being started");
		break;
	case NetworkState::changingMode:
		reply.copy("WiFi module is changing mode");
		break;
	case NetworkState::active:
		reply.copy("WiFi module is ");
		reply.cat(TranslateWiFiState(currentMode));
		if (currentMode == WiFiState::connected || currentMode == WiFiState::runningAsAccessPoint)
		{
			reply.catf("%s, IP address %s", actualSsid, IP4String(ipAddress).c_str());
		}
		break;
	default:
		reply.copy("Unknown network state");
		return GCodeResult::error;
		break;
	}
	return GCodeResult::ok;
}

// Start up the ESP. We assume it is not already started.
// ESP8266 boot modes:
// GPIO0	GPIO2	GPIO15
// 0		1		0		Firmware download from UART
// 1		1		0		Normal boot from flash memory
// 0		0		1		SD card boot (not used on Duet)
void WiFiInterface::Start() noexcept
{
	// The ESP8266 is held in a reset state by a pulldown resistor until we enable it.
	// Make sure the ESP8266 is in the reset state
	pinMode(EspResetPin, OUTPUT_LOW);

#if defined(DUET_NG) || defined(DUET3MINI)
	pinMode(EspEnablePin, OUTPUT_LOW);
#endif

	// Set up our transfer request pin (GPIO4) as an output and set it low
	pinMode(SamTfrReadyPin, OUTPUT_LOW);

	// Set up our data ready pin (ESP GPIO0) as an output and set it high ready to boot the ESP from flash
	pinMode(EspDataReadyPin, OUTPUT_HIGH);

	// GPIO2 also needs to be high to boot. It's connected to MISO on the SAM, so set the pullup resistor on that pin
	pinMode(APIN_ESP_SPI_MISO, INPUT_PULLUP);

	// Set our CS input (ESP GPIO15) low ready for booting the ESP. This also clears the transfer ready latch.
	pinMode(SamCsPin, OUTPUT_LOW);

	// Make sure it has time to reset - no idea how long it needs, but 20ms should be plenty
	delay(50);

	// Release the reset on the ESP8266
	StartWiFi();

	// Give it time to sample GPIO0 and GPIO15
	// GPIO0 has to be held high for sufficient time:
	// - 10ms is not enough
	// - 18ms after reset is released, an oscillating signal appears on GPIO0 for 55ms
	// - so 18ms is probably long enough. Use 50ms for safety.
	delay(50);

	// Relinquish control of our CS pin so that the ESP can take it over
	pinMode(SamCsPin, INPUT);

	// Set the data request pin to be an input
	pinMode(EspDataReadyPin, INPUT_PULLUP);

	// The ESP takes about 300ms before it starts talking to us, so don't wait for it here, do that in Spin()
	spiTxUnderruns = spiRxOverruns = 0;
	reconnectCount = 0;
	transferAlreadyPendingCount = readyTimeoutCount = responseTimeoutCount = 0;

	lastTickMillis = millis();
	SetState(NetworkState::starting1);
}

// Stop the ESP
void WiFiInterface::Stop() noexcept
{
	if (GetState() != NetworkState::disabled)
	{
		MutexLocker lock(interfaceMutex);

		digitalWrite(SamTfrReadyPin, false);		// tell the ESP we can't receive
		digitalWrite(EspResetPin, false);			// put the ESP back into reset
#if defined(DUET_NG) || defined(DUET3MINI)
		digitalWrite(EspEnablePin, false);
#endif
		DisableEspInterrupt();						// ignore IRQs from the transfer request pin

		NVIC_DisableIRQ(ESP_SPI_IRQn);
		DisableSpi();
#if !SAME5x
		spi_dma_check_rx_complete();
#endif
		spi_dma_disable();

		SetState(NetworkState::disabled);
		currentMode = WiFiState::disabled;
	}
}

void WiFiInterface::Spin() noexcept
{
	// Main state machine.
	switch (GetState())
	{
	case NetworkState::starting1:
		{
			// The ESP toggles CS before it has finished starting up, so don't look at the CS signal too soon
			const uint32_t now = millis();
			if (now - lastTickMillis >= WiFiStartupMillis)
			{
				lastTickMillis = now;
				SetState(NetworkState::starting2);
			}
		}
		break;

	case NetworkState::starting2:
		{
			// See if the ESP8266 has kept its pins at their stable values for long enough
			const uint32_t now = millis();
			if (digitalRead(SamCsPin) && digitalRead(EspDataReadyPin) && !digitalRead(APIN_ESP_SPI_SCK))
			{
				if (now - lastTickMillis >= WiFiStableMillis)
				{
					// Setup the SPI controller in slave mode and assign the CS pin to it
					platform.Message(NetworkInfoMessage, "WiFi module started\n");
					SetupSpi();									// set up the SPI subsystem

					// Read the status to get the WiFi server version and MAC address
					Receiver<NetworkStatusResponse> status;
					int32_t rc = SendCommand(NetworkCommand::networkGetStatus, 0, 0, nullptr, 0, status);
					if (rc > 0)
					{
						SafeStrncpy(wiFiServerVersion, status.Value().versionText, ARRAY_SIZE(wiFiServerVersion));
						macAddress.SetFromBytes(status.Value().macAddress);

						// Set the hostname before anything else is done
						rc = SendCommand(NetworkCommand::networkSetHostName, 0, 0, 0, reprap.GetNetwork().GetHostname(), HostNameLength, nullptr, 0);
						if (rc != ResponseEmpty)
						{
							reprap.GetPlatform().MessageF(NetworkErrorMessage, "failed to set WiFi hostname: %s\n", TranslateWiFiResponse(rc));
						}
#if SAME5x
						// If running the RTOS-based WiFi module code, tell the module to increase SPI clock speed to 40MHz.
						// This is safe on SAME5x processors but not on SAM4 processors.
						if (isdigit(wiFiServerVersion[0]) && wiFiServerVersion[0] >= '2')
						{
							rc = SendCommand(NetworkCommand::networkSetClockControl, 0, 0, 0x2001, nullptr, 0, nullptr, 0);
							if (rc != ResponseEmpty)
							{
								reprap.GetPlatform().MessageF(NetworkErrorMessage, "failed to set WiFi SPI speed: %s\n", TranslateWiFiResponse(rc));
							}
						}
#endif
						SetState(NetworkState::active);
						espStatusChanged = true;				// make sure we fetch the current state and enable the ESP interrupt
					}
					else
					{
						// Something went wrong, maybe a bad firmware image was flashed
						// Disable the WiFi chip again in this case
						platform.MessageF(NetworkErrorMessage, "failed to initialise WiFi module: %s\n", TranslateWiFiResponse(rc));
						Stop();
					}
				}
			}
			else
			{
				lastTickMillis = now;
			}
		}
		break;

	case NetworkState::disabled:
#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
		if (uploader != nullptr)
		{
			uploader->Spin();
		}
#endif
		break;

	case NetworkState::active:
		if (espStatusChanged && digitalRead(EspDataReadyPin))
		{
			if (reprap.Debug(moduleNetwork))
			{
				debugPrintf("ESP reported status change\n");
			}
			GetNewStatus();
		}
		else if (   currentMode != requestedMode
				 && currentMode != WiFiState::connecting
				 && currentMode != WiFiState::reconnecting
				 && currentMode != WiFiState::autoReconnecting
				)
		{
			// Tell the wifi module to change mode
			int32_t rslt = ResponseUnknownError;
			if (currentMode != WiFiState::idle)
			{
				// We must set WiFi module back to idle before changing to the new state
				rslt = SendCommand(NetworkCommand::networkStop, 0, 0, 0, nullptr, 0, nullptr, 0);
			}
			else if (requestedMode == WiFiState::connected)
			{
				rslt = SendCommand(NetworkCommand::networkStartClient, 0, 0, 0, requestedSsid, SsidLength, nullptr, 0);
			}
			else if (requestedMode == WiFiState::runningAsAccessPoint)
			{
				rslt = SendCommand(NetworkCommand::networkStartAccessPoint, 0, 0, 0, nullptr, 0, nullptr, 0);
			}

			if (rslt >= 0)
			{
				SetState(NetworkState::changingMode);
			}
			else
			{
				Stop();
				platform.MessageF(NetworkErrorMessage, "failed to change WiFi mode: %s\n", TranslateWiFiResponse(rslt));
			}
		}
		else if (currentMode == WiFiState::connected || currentMode == WiFiState::runningAsAccessPoint)
		{
			// Find the next socket to poll
			const size_t startingSocket = currentSocket;
			do
			{
				if (sockets[currentSocket]->NeedsPolling())
				{
					break;
				}
				++currentSocket;
				if (currentSocket == NumWiFiTcpSockets)
				{
					currentSocket = 0;
				}
			} while (currentSocket != startingSocket);

			// Either the current socket needs polling, or no sockets do but we must still poll one of them to get notified of any new connections
			sockets[currentSocket]->Poll();
			++currentSocket;
			if (currentSocket == NumWiFiTcpSockets)
			{
				currentSocket = 0;
			}

			// Check if the data port needs to be closed
			if (closeDataPort)
			{
				for (WiFiSocket *s : sockets)
				{
					if (s->GetProtocol() == FtpDataProtocol)
					{
						if (!s->IsClosing())
						{
							TerminateDataPort();
						}
						break;
					}
				}
			}
		}
		break;

	case NetworkState::changingMode:
		// Here when we have asked the ESP to change mode. Don't leave this state until we have a new status report from the ESP.
		if (espStatusChanged && digitalRead(EspDataReadyPin))
		{
			GetNewStatus();
			switch (currentMode)
			{
			case WiFiState::connecting:
			case WiFiState::reconnecting:
			case WiFiState::autoReconnecting:
				break;											// let the connect attempt continue

			case WiFiState::connected:
			case WiFiState::runningAsAccessPoint:
				SetState(NetworkState::active);
				{
					// Get our IP address, this needs to be correct for FTP to work
					Receiver<NetworkStatusResponse> status;
					if (SendCommand(NetworkCommand::networkGetStatus, 0, 0, nullptr, 0, status) > 0)
					{
						ipAddress.SetV4LittleEndian(status.Value().ipAddress);
						SafeStrncpy(actualSsid, status.Value().ssid, SsidLength);
					}
					InitSockets();
					reconnectCount = 0;
					platform.MessageF(NetworkInfoMessage, "WiFi module is %s%s, IP address %s\n",
						TranslateWiFiState(currentMode),
						actualSsid,
						IP4String(ipAddress).c_str());
				}
				break;

			default:
				if (requestedMode != WiFiState::connected)
				{
					requestedMode = currentMode;				// don't keep repeating the request if it failed and it wasn't a connect request
				}
				SetState(NetworkState::active);
				platform.MessageF(NetworkInfoMessage, "WiFi module is %s\n", TranslateWiFiState(currentMode));
				break;
			}
		}
		break;

	default:
		break;
	}

	// Check for debug info received from the WiFi module
	if (serialRunning)
	{
		while (!debugPrintPending && SERIAL_WIFI_DEVICE.available() != 0)
		{
			const char c = (char)SERIAL_WIFI_DEVICE.read();
			if (c == '\n')
			{
				debugPrintPending = true;
			}
			else if (c != '\r')
			{
				debugMessageBuffer[debugMessageChars++] = c;
				if (debugMessageChars == ARRAY_SIZE(debugMessageBuffer) - 1)
				{
					debugPrintPending = true;
				}
			}
		}
	}

	// Check for debug info received from the WiFi module
	if (debugPrintPending)
	{
		if (reprap.Debug(moduleWiFi))
		{
			debugMessageBuffer[debugMessageChars] = 0;
			debugPrintf("WiFi: %s\n", debugMessageBuffer);
		}
		debugMessageChars = 0;
		debugPrintPending = false;
	}
}

// Translate a ESP8266 reset reason to text. Keep this in step with the codes used in file MessageFormats.h in the WiFi server project.
const char* WiFiInterface::TranslateEspResetReason(uint32_t reason) noexcept
{
	// Mapping from known ESP reset codes to reasons
	static const char * const resetReasonTexts[] =
	{
		"Power up",
		"Hardware watchdog",
		"Exception",
		"Software watchdog",
		"Software restart",
		"Deep-sleep wakeup",
		"Turned on by main processor",
		"Brownout",
		"SDIO reset",
		"Unknown"
	};

	return (reason < sizeof(resetReasonTexts)/sizeof(resetReasonTexts[0]))
			? resetReasonTexts[reason]
			: "Unrecognised";
}

void WiFiInterface::Diagnostics(MessageType mtype) noexcept
{
	platform.MessageF(mtype, "- WiFi -\nNetwork state is %s\n", GetStateName());
	platform.MessageF(mtype, "WiFi module is %s\n", TranslateWiFiState(currentMode));
	platform.MessageF(mtype, "Failed messages: pending %u, notready %u, noresp %u\n", transferAlreadyPendingCount, readyTimeoutCount, responseTimeoutCount);

#if 0
	// The underrun/overrun counters don't work at present
	platform.MessageF(mtype, "SPI underruns %u, overruns %u\n", spiTxUnderruns, spiRxOverruns);
#endif

	if (GetState() != NetworkState::disabled && GetState() != NetworkState::starting1 && GetState() != NetworkState::starting2)
	{
		Receiver<NetworkStatusResponse> status;
		status.Value().clockReg = 0xFFFFFFFF;				// older WiFi firmware doesn't return this value, so preset it
		if (SendCommand(NetworkCommand::networkGetStatus, 0, 0, nullptr, 0, status) > 0)
		{
			NetworkStatusResponse& r = status.Value();
			r.versionText[ARRAY_UPB(r.versionText)] = 0;
			platform.MessageF(mtype, "WiFi firmware version %s\n", r.versionText);
			platform.MessageF(mtype, "WiFi MAC address %02x:%02x:%02x:%02x:%02x:%02x\n",
								r.macAddress[0], r.macAddress[1], r.macAddress[2], r.macAddress[3], r.macAddress[4], r.macAddress[5]);
			platform.MessageF(mtype, "WiFi Vcc %.2f, reset reason %s\n", (double)((float)r.vcc/1024), TranslateEspResetReason(r.resetReason));
			platform.MessageF(mtype, "WiFi flash size %" PRIu32 ", free heap %" PRIu32 "\n", r.flashSize, r.freeHeap);

			if (currentMode == WiFiState::connected || currentMode == WiFiState::runningAsAccessPoint)
			{
				platform.MessageF(mtype, "WiFi IP address %s\n", IP4String(r.ipAddress).c_str());
			}

			if (currentMode == WiFiState::connected)
			{
				constexpr const char* SleepModes[4] = { "unknown", "none", "light", "modem" };
				constexpr const char* ConnectionModes[4] =  { "none", "802.11b", "802.11g", "802.11n" };
				platform.MessageF(mtype, "WiFi signal strength %ddBm, mode %s, reconnections %u, sleep mode %s\n", (int)r.rssi, ConnectionModes[r.phyMode], reconnectCount, SleepModes[r.sleepMode]);
			}
			else if (currentMode == WiFiState::runningAsAccessPoint)
			{
				platform.MessageF(mtype, "Connected clients %u\n", (unsigned int)r.numClients);
			}
			// status, ssid and hostName not displayed
			platform.MessageF(mtype, "Clock register %08" PRIx32 "\n", r.clockReg);

			// Print LwIP stats and other values over the ESP's UART line
			if (SendCommand(NetworkCommand::diagnostics, 0, 0, 0, nullptr, 0, nullptr, 0) != ResponseEmpty)
			{
				platform.Message(mtype, "Failed to request ESP stats\n");
			}
		}
		else
		{
			platform.Message(mtype, "Failed to get WiFi status\n");
		}
	}
	platform.Message(mtype, "Socket states:");
	for (size_t i = 0; i < NumWiFiTcpSockets; i++)
	{
		platform.MessageF(mtype, " %d", sockets[i]->State());
	}
	platform.Message(mtype, "\n");
}

// Enable or disable the network
GCodeResult WiFiInterface::EnableInterface(int mode, const StringRef& ssid, const StringRef& reply) noexcept
{
	// Translate enable mode to desired WiFi mode
	const WiFiState modeRequested = (mode == 0) ? WiFiState::idle
									: (mode == 1) ? WiFiState::connected
										: (mode == 2) ? WiFiState::runningAsAccessPoint
											: WiFiState::disabled;
	if (modeRequested == WiFiState::connected)
	{
		memset(requestedSsid, 0, sizeof(requestedSsid));
		SafeStrncpy(requestedSsid, ssid.c_str(), ARRAY_SIZE(requestedSsid));
	}

	if (activated)
	{
		if (modeRequested == WiFiState::disabled)
		{
			// Shut down WiFi module completely
			requestedMode = modeRequested;
			if (GetState() != NetworkState::disabled)
			{
				Stop();
				platform.Message(GenericMessage, "WiFi module stopped\n");
			}
		}
		else
		{
			if (GetState() == NetworkState::disabled)
			{
				requestedMode = modeRequested;
				Start();
			}
			else if (modeRequested == currentMode || (modeRequested == WiFiState::connected && currentMode == WiFiState::connecting))
			{
				// Nothing to do, but make sure the requested mode is correct
				requestedMode = modeRequested;
			}
			else if (currentMode != WiFiState::idle && modeRequested != WiFiState::idle)
			{
				reply.copy("Turn off the current WiFi mode before selecting a new one");
			}
			else
			{
				requestedMode = modeRequested;
			}
		}
	}
	else
	{
		requestedMode = modeRequested;
	}
	return GCodeResult::ok;
}

int WiFiInterface::EnableState() const noexcept
{
	return (requestedMode == WiFiState::idle) ? 0
			: (requestedMode == WiFiState::connected) ? 1
				: (requestedMode == WiFiState::runningAsAccessPoint) ? 2
						: -1;
}

// Translate the wifi state to text.
// The 'connected' and 'runningAsAccessPoint' states include a space at the end because the caller is expected to append the access point name.
/*static*/ const char* WiFiInterface::TranslateWiFiState(WiFiState w) noexcept
{
	switch (w)
	{
	case WiFiState::disabled:				return "disabled";
	case WiFiState::idle:					return "idle";
	case WiFiState::connecting:				return "trying to connect";
	case WiFiState::connected:				return "connected to access point ";
	case WiFiState::runningAsAccessPoint:	return "providing access point ";
	case WiFiState::autoReconnecting:		return "auto-reconnecting";
	case WiFiState::reconnecting:			return "reconnecting";
	default:								return "in an unknown state";
	}
}

void WiFiInterface::EspRequestsTransfer() noexcept
{
	espStatusChanged = true;
	DisableEspInterrupt();				// don't allow more interrupts until we have acknowledged this one
}

void WiFiInterface::SetIPAddress(IPAddress p_ip, IPAddress p_netmask, IPAddress p_gateway) noexcept
{
	ipAddress = p_ip;
	netmask = p_netmask;
	gateway = p_gateway;
}

GCodeResult WiFiInterface::HandleWiFiCode(int mcode, GCodeBuffer &gb, const StringRef& reply, OutputBuffer*& longReply) THROWS(GCodeException)
{
	switch (mcode)
	{
	case 587:	// Add WiFi network or list remembered networks
		if (gb.Seen('S'))
		{
			WirelessConfigurationData config;
			memset(&config, 0, sizeof(config));
			String<ARRAY_SIZE(config.ssid)> ssid;
			gb.GetQuotedString(ssid.GetRef());
			SafeStrncpy(config.ssid, ssid.c_str(), ARRAY_SIZE(config.ssid));

			// Get the password
			gb.MustSee('P');
			{
				String<ARRAY_SIZE(config.password)> password;
				gb.GetQuotedString(password.GetRef());
				if (password.strlen() < 8 && password.strlen() != 0)			// WPA2 passwords must be at least 8 characters
				{
					reply.copy("WiFi password must be at least 8 characters");
					return GCodeResult::error;
				}
				SafeStrncpy(config.password, password.c_str(), ARRAY_SIZE(config.password));
			}

			if (gb.Seen('I'))
			{
				IPAddress temp;
				gb.GetIPAddress(temp);
				config.ip = temp.GetV4LittleEndian();
			}
			if (gb.Seen('J'))
			{
				IPAddress temp;
				gb.GetIPAddress(temp);
				config.gateway = temp.GetV4LittleEndian();
			}
			if (gb.Seen('K'))
			{
				IPAddress temp;
				gb.GetIPAddress(temp);
				config.netmask = temp.GetV4LittleEndian();
			}

			const int32_t rslt = SendCommand(NetworkCommand::networkAddSsid, 0, 0, 0, &config, sizeof(config), nullptr, 0);
			if (rslt == ResponseEmpty)
			{
				return GCodeResult::ok;
			}
			else
			{
				reply.printf("Failed to add SSID to remembered list: %s", TranslateWiFiResponse(rslt));
			}
		}
		else
		{
			// List remembered networks
			if (longReply == nullptr && !OutputBuffer::Allocate(longReply))
			{
				return GCodeResult::notFinished;			// try again later
			}

			const bool jsonFormat = gb.Seen('F') && gb.GetUIValue() == 1;

			const size_t declaredBufferLength = (MaxRememberedNetworks + 1) * ReducedWirelessConfigurationDataSize;		// enough for all the remembered SSID data
			uint32_t buffer[NumDwords(declaredBufferLength)];
			const int32_t rslt = SendCommand(NetworkCommand::networkRetrieveSsidData, 0, 0, 0, nullptr, 0, buffer, declaredBufferLength);
			if (rslt >= 0)
			{
				longReply->copy((jsonFormat) ? "{\"rememberedNetworks\":[" : "Remembered networks:");
				size_t offset = (jsonFormat) ? 0 : ReducedWirelessConfigurationDataSize;		// skip own SSID details unless reporting in JSON format
				bool found = false;
				while (offset + ReducedWirelessConfigurationDataSize <= (size_t)rslt)
				{
					WirelessConfigurationData* const wp = reinterpret_cast<WirelessConfigurationData *>(reinterpret_cast<char*>(buffer) + offset);
					if (wp->ssid[0] != 0 || (offset == 0 && jsonFormat))
					{
						wp->ssid[ARRAY_UPB(wp->ssid)] = 0;
						if (jsonFormat && found)
						{
							longReply->cat(',');
						}
						longReply->catf((jsonFormat)
										? "{\"ssid\":\"%.s\",\"ip\":\"%s\",\"gw\":\"%s\",\"mask\":\"%s\"}"
											: "\n%s IP=%s GW=%s NM=%s",
											  	 wp->ssid, IP4String(wp->ip).c_str(), IP4String(wp->gateway).c_str(), IP4String(wp->netmask).c_str());
						found = true;
					}
					offset += ReducedWirelessConfigurationDataSize;
				}

				if (jsonFormat)
				{
					longReply->cat("],\"err\":0}\n");
				}
				else if (!found)
				{
					longReply->cat(" none");
				}
				return GCodeResult::ok;
			}

			longReply->printf((jsonFormat) ? "{\"rememberedNetworks\":[],\"err\":1,\"errText\":\"%.s\"}" : "Failed to retrieve network list: %s", TranslateWiFiResponse(rslt));
		}
		return GCodeResult::error;

	case 588:	// Forget WiFi network
		{
			gb.MustSee('S');
			String<SsidLength> ssidText;
			gb.GetQuotedString(ssidText.GetRef());
			if (strcmp(ssidText.c_str(), "*") == 0)
			{
				const int32_t rslt = SendCommand(NetworkCommand::networkFactoryReset, 0, 0, 0, nullptr, 0, nullptr, 0);
				if (rslt == ResponseEmpty)
				{
					return GCodeResult::ok;
				}

				reply.printf("Failed to reset the WiFi module to factory settings: %s", TranslateWiFiResponse(rslt));
				return GCodeResult::error;
			}

			uint32_t ssid32[NumDwords(SsidLength)];				// need a dword-aligned buffer for SendCommand
			memcpy(ssid32, ssidText.c_str(), SsidLength);
			const int32_t rslt = SendCommand(NetworkCommand::networkDeleteSsid, 0, 0, 0, ssid32, SsidLength, nullptr, 0);
			if (rslt == ResponseEmpty)
			{
				return GCodeResult::ok;
			}

			reply.printf("Failed to remove SSID from remembered list: %s", TranslateWiFiResponse(rslt));
			return GCodeResult::error;
		}

	case 589:	// Configure access point
		if (gb.Seen('S'))
		{
			// Configure access point parameters
			WirelessConfigurationData config;
			memset(&config, 0, sizeof(config));
			String<SsidLength> ssid;
			gb.GetQuotedString(ssid.GetRef());
			if (strcmp(ssid.c_str(), "*") == 0)
			{
				// Delete the access point details
				memset(&config, 0xFF, sizeof(config));
			}
			else
			{
				SafeStrncpy(config.ssid, ssid.c_str(), ARRAY_SIZE(config.ssid));
				String<ARRAY_SIZE(config.password)> password;
				gb.MustSee('P');
				gb.GetQuotedString(password.GetRef());
				SafeStrncpy(config.password, password.c_str(), ARRAY_SIZE(config.password));
				if (password.strlen() < 8 && password.strlen() != 0)			// WPA2 passwords must be at least 8 characters
				{
					reply.copy("WiFi password must be at least 8 characters");
					return GCodeResult::error;
				}
				SafeStrncpy(config.password, password.c_str(), ARRAY_SIZE(config.password));
				gb.MustSee('I');
				IPAddress temp;
				gb.GetIPAddress(temp);
				config.ip = temp.GetV4LittleEndian();
				config.channel = (gb.Seen('C')) ? gb.GetIValue() : 0;
			}

			const int32_t rslt = SendCommand(NetworkCommand::networkConfigureAccessPoint, 0, 0, 0, &config, sizeof(config), nullptr, 0);
			if (rslt == ResponseEmpty)
			{
				return GCodeResult::ok;
			}

			reply.printf("Failed to configure access point parameters: %s", TranslateWiFiResponse(rslt));
		}
		else if (gb.Seen('T'))
		{
			// Special code to set max transmitter power, 0 to 20.5dBm
			const float powerTimes4 = gb.GetFValue() * 4;
			if (powerTimes4 < 0.0 || powerTimes4 > 82.0)
			{
				reply.copy("Power setting out of range");
			}
			else
			{
				const int32_t rslt = SendCommand(NetworkCommand::networkSetTxPower, 0, (uint8_t)powerTimes4, 0, nullptr, 0, nullptr, 0);
				if (rslt == ResponseEmpty)
				{
					return GCodeResult::ok;
				}
				reply.printf("Failed to set maximum transmit power: %s", TranslateWiFiResponse(rslt));
			}
		}
		else if (gb.Seen('L'))
		{
			// Special code to configure SPI clock speed
			const uint32_t clockVal = gb.GetUIValue();
			const int32_t rslt = SendCommand(NetworkCommand::networkSetClockControl, 0, 0, clockVal, nullptr, 0, nullptr, 0);
			if (rslt == ResponseEmpty)
			{
				return GCodeResult::ok;
			}
			reply.printf("Failed to set clock: %s", TranslateWiFiResponse(rslt));
		}
		else
		{
			// Report access point parameters
			uint32_t buffer[NumDwords(ReducedWirelessConfigurationDataSize)];
			const int32_t rslt = SendCommand(NetworkCommand::networkRetrieveSsidData, 0, 0, 0, nullptr, 0, buffer, ReducedWirelessConfigurationDataSize);
			if (rslt >= 0)
			{
				WirelessConfigurationData* const wp = reinterpret_cast<WirelessConfigurationData *>(buffer);
				if (wp->ssid[0] == 0)
				{
					reply.copy("Own SSID not configured");
				}
				else
				{
					wp->ssid[ARRAY_UPB(wp->ssid)] = 0;
					reply.printf("Own SSID: %s IP=%s GW=%s NM=%s", wp->ssid, IP4String(wp->ip).c_str(), IP4String(wp->gateway).c_str(), IP4String(wp->netmask).c_str());
					return GCodeResult::ok;
				}
			}
			else
			{
				reply.printf("Failed to retrieve own SSID data: %s", TranslateWiFiResponse(rslt));
			}
		}
		return GCodeResult::error;

	default:	// should not happen
		return GCodeResult::error;
	}
}

// Set the DHCP hostname
void WiFiInterface::UpdateHostname(const char *hostname) noexcept
{
	// Update the hostname if possible
	if (GetState() == NetworkState::active)
	{
		const int32_t rslt = SendCommand(NetworkCommand::networkSetHostName, 0, 0, 0, hostname, HostNameLength, nullptr, 0);
		if (rslt != ResponseEmpty)
		{
			platform.MessageF(GenericMessage, "Error: Could not set WiFi hostname: %s\n", TranslateWiFiResponse(rslt));
		}
	}
}

GCodeResult WiFiInterface::SetMacAddress(const MacAddress& mac, const StringRef& reply) noexcept
{
	reply.copy("Not supported on this interface");
	return GCodeResult::warningNotSupported;
}

void WiFiInterface::InitSockets() noexcept
{
	for (size_t i = 0; i < NumProtocols; ++i)
	{
		if (protocolEnabled[i])
		{
			StartProtocol(i);
		}
	}
	currentSocket = 0;
}

void WiFiInterface::TerminateSockets() noexcept
{
	for (SocketNumber skt = 0; skt < NumWiFiTcpSockets; ++skt)
	{
		sockets[skt]->Terminate();
	}
}

void WiFiInterface::TerminateSockets(TcpPort port) noexcept
{
	for (WiFiSocket *socket : sockets)
	{
		if (socket->GetLocalPort() == port)
		{
			socket->Terminate();
		}
	}
}

// This is called to tell the network which sockets are active
void WiFiInterface::UpdateSocketStatus(uint16_t connectedSockets, uint16_t otherEndClosedSockets) noexcept
{
	for (size_t i = 0; i < NumWiFiTcpSockets; ++i)
	{
		if (((connectedSockets | otherEndClosedSockets) & (1u << i)) != 0)
		{
			sockets[i]->SetNeedsPolling();
		}
	}
}

// Open the FTP data port
void WiFiInterface::OpenDataPort(TcpPort port) noexcept
{
	for (WiFiSocket *s : sockets)
	{
		if (s->GetProtocol() == FtpDataProtocol)
		{
			closeDataPort = true;
			TerminateDataPort();
			break;
		}
	}

	ftpDataPort = port;
	SendListenCommand(ftpDataPort, FtpDataProtocol, 1);
}

// Close FTP data port and purge associated resources
void WiFiInterface::TerminateDataPort() noexcept
{
	WiFiSocket *ftpDataSocket = nullptr;
	for (WiFiSocket *s : sockets)
	{
		if (s->GetLocalPort() == ftpDataPort)
		{
			ftpDataSocket = s;
			break;
		}
	}

	if (ftpDataSocket == nullptr)
	{
		StopListening(ftpDataPort);
		ftpDataPort = 0;
		return;
	}

	if (closeDataPort || !ftpDataSocket->IsClosing())
	{
		StopListening(ftpDataPort);
		ftpDataSocket->TerminateAndDisable();
		ftpDataPort = 0;
		closeDataPort = false;
	}
	else
	{
		// The socket may be waiting for a ACKs and a graceful disconnect.
		// Give it some more time
		closeDataPort = true;
	}
}

#ifndef __LPC17xx__

#if USE_PDC
static Pdc *spi_pdc;
#endif

#if USE_XDMAC

// XDMAC hardware
const uint32_t SPI0_XDMAC_TX_CH_NUM = 1;
const uint32_t SPI0_XDMAC_RX_CH_NUM = 2;

static xdmac_channel_config_t xdmac_tx_cfg, xdmac_rx_cfg;

#endif

#if !USE_PDC

static inline void spi_rx_dma_enable() noexcept
{
#if USE_DMAC
	dmac_channel_enable(DMAC, DmacChanWiFiRx);
#endif

#if USE_XDMAC
	xdmac_channel_enable(XDMAC, DmacChanWiFiRx);
#endif

#if USE_DMAC_MANAGER
	DmacManager::EnableChannel(DmacChanWiFiRx, DmacPrioWiFi);
#endif
}

static inline void spi_tx_dma_enable() noexcept
{
#if USE_DMAC
	dmac_channel_enable(DMAC, DmacChanWiFiTx);
#endif

#if USE_XDMAC
	xdmac_channel_enable(XDMAC, DmacChanWiFiTx);
#endif

#if USE_DMAC_MANAGER
	DmacManager::EnableChannel(DmacChanWiFiTx, DmacPrioWiFi);
#endif
}

static inline void spi_rx_dma_disable() noexcept
{
#if USE_DMAC
	dmac_channel_disable(DMAC, DmacChanWiFiRx);
#endif

#if USE_XDMAC
	xdmac_channel_disable(XDMAC, DmacChanWiFiRx);
#endif

#if USE_DMAC_MANAGER
	DmacManager::DisableChannel(DmacChanWiFiRx);
#endif
}

static inline void spi_tx_dma_disable() noexcept
{
#if USE_DMAC
	dmac_channel_disable(DMAC, DmacChanWiFiTx);
#endif

#if USE_XDMAC
	xdmac_channel_disable(XDMAC, DmacChanWiFiTx);
#endif

#if USE_DMAC_MANAGER
	DmacManager::DisableChannel(DmacChanWiFiTx);
#endif
}

#endif

static void spi_dma_disable() noexcept
{
#if USE_PDC
	pdc_disable_transfer(spi_pdc, PERIPH_PTCR_TXTDIS | PERIPH_PTCR_RXTDIS);
#else
	spi_tx_dma_disable();
	spi_rx_dma_disable();
#endif
}

static inline void spi_dma_enable() noexcept
{
#if USE_PDC
	pdc_enable_transfer(spi_pdc, PERIPH_PTCR_TXTEN | PERIPH_PTCR_RXTEN);
#else
	spi_rx_dma_enable();
	spi_tx_dma_enable();
#endif
}

#if !SAME5x

static bool spi_dma_check_rx_complete() noexcept
{
#if USE_PDC
	return true;
#endif

#if USE_DMAC
	const uint32_t status = DMAC->DMAC_CHSR;
	if (   ((status & (DMAC_CHSR_ENA0 << DmacChanWiFiRx)) == 0)		// controller is not enabled, perhaps because it finished a full buffer transfer
		|| ((status & (DMAC_CHSR_EMPT0 << DmacChanWiFiRx)) != 0)	// controller is enabled, probably suspended, and the FIFO is empty
	   )
	{
		// Disable the channel.
		// We also need to set the resume bit, otherwise it remains suspended when we re-enable it.
		DMAC->DMAC_CHDR = (DMAC_CHDR_DIS0 << DmacChanWiFiRx) | (DMAC_CHDR_RES0 << DmacChanWiFiRx);
		return true;
	}
#endif

#if USE_XDMAC
	const uint32_t status = xdmac_channel_get_status(XDMAC);
	const uint32_t channelStatus = XDMAC->XDMAC_CHID[DmacChanWiFiRx].XDMAC_CC;
	if (   ((status & (1 << DmacChanWiFiRx)) == 0)						// channel is not enabled
		|| (((channelStatus & XDMAC_CC_RDIP) == XDMAC_CC_RDIP_DONE) && ((channelStatus & XDMAC_CC_WRIP) == XDMAC_CC_WRIP_DONE))	// controller is neither reading nor writing via this channel
	)
	{
		// Disable the channel.
		// We also need to set the resume bit, otherwise it remains suspended when we re-enable it.
		xdmac_channel_disable(XDMAC, DmacChanWiFiRx);
		xdmac_channel_readwrite_resume(XDMAC, DmacChanWiFiRx);
		return true;
	}
#endif

	return false;
}

#endif

static void spi_tx_dma_setup(const void *buf, uint32_t transferLength) noexcept
{
#if USE_PDC
	pdc_packet_t pdc_spi_packet;
	pdc_spi_packet.ul_addr = reinterpret_cast<uint32_t>(buf);
	pdc_spi_packet.ul_size = transferLength;
	pdc_tx_init(spi_pdc, &pdc_spi_packet, NULL);
#endif

#if USE_DMAC
	DMAC->DMAC_EBCISR;		// clear any pending interrupts

	dmac_channel_set_source_addr(DMAC, DmacChanWiFiTx, reinterpret_cast<uint32_t>(buf));
	dmac_channel_set_destination_addr(DMAC, DmacChanWiFiTx, reinterpret_cast<uint32_t>(&(ESP_SPI->SPI_TDR)));
	dmac_channel_set_descriptor_addr(DMAC, DmacChanWiFiTx, 0);
	dmac_channel_set_ctrlA(DMAC, DmacChanWiFiTx, transferLength | DMAC_CTRLA_SRC_WIDTH_WORD | DMAC_CTRLA_DST_WIDTH_BYTE);
	dmac_channel_set_ctrlB(DMAC, DmacChanWiFiTx,
		DMAC_CTRLB_SRC_DSCR | DMAC_CTRLB_DST_DSCR | DMAC_CTRLB_FC_MEM2PER_DMA_FC | DMAC_CTRLB_SRC_INCR_INCREMENTING | DMAC_CTRLB_DST_INCR_FIXED);
#endif

#if USE_XDMAC
	xdmac_disable_interrupt(XDMAC, DmacChanWiFiTx);
	const uint32_t xdmaint = (XDMAC_CIE_BIE |
			XDMAC_CIE_DIE   |
			XDMAC_CIE_FIE   |
			XDMAC_CIE_RBIE  |
			XDMAC_CIE_WBIE  |
			XDMAC_CIE_ROIE);

	xdmac_tx_cfg.mbr_ubc = transferLength;
	xdmac_tx_cfg.mbr_sa = reinterpret_cast<uint32_t>(buf);
	xdmac_tx_cfg.mbr_da = reinterpret_cast<uint32_t>(&(ESP_SPI->SPI_TDR));
	xdmac_tx_cfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
			XDMAC_CC_MBSIZE_SINGLE |
			XDMAC_CC_DSYNC_MEM2PER |
			XDMAC_CC_CSIZE_CHK_1 |
			XDMAC_CC_DWIDTH_BYTE |
			XDMAC_CC_SIF_AHB_IF0 |
			XDMAC_CC_DIF_AHB_IF1 |
			XDMAC_CC_SAM_INCREMENTED_AM |
			XDMAC_CC_DAM_FIXED_AM |
			XDMAC_CC_PERID(SPI0_XDMAC_TX_CH_NUM);
	xdmac_tx_cfg.mbr_bc = 0;
	xdmac_tx_cfg.mbr_ds = 0;
	xdmac_tx_cfg.mbr_sus = 0;
	xdmac_tx_cfg.mbr_dus = 0;
	xdmac_configure_transfer(XDMAC, DmacChanWiFiTx, &xdmac_tx_cfg);

	xdmac_channel_set_descriptor_control(XDMAC, DmacChanWiFiTx, 0);
	xdmac_channel_disable_interrupt(XDMAC, DmacChanWiFiTx, xdmaint);
#endif

#if USE_DMAC_MANAGER
	DmacManager::SetSourceAddress(DmacChanWiFiTx, buf);
	DmacManager::SetDestinationAddress(DmacChanWiFiTx, &(WiFiSpiSercom->SPI.DATA.reg));
	DmacManager::SetBtctrl(DmacChanWiFiTx, DMAC_BTCTRL_STEPSIZE_X1 | DMAC_BTCTRL_STEPSEL_SRC | DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_BEATSIZE_WORD | DMAC_BTCTRL_BLOCKACT_NOACT);
	DmacManager::SetDataLength(DmacChanWiFiTx, (transferLength + 3) >> 2);			// must do this one last
	DmacManager::SetTriggerSourceSercomTx(DmacChanWiFiTx, WiFiSpiSercomNumber);
#endif
}

static void spi_rx_dma_setup(void *buf, uint32_t transferLength) noexcept
{
#if USE_PDC
	pdc_packet_t pdc_spi_packet;
	pdc_spi_packet.ul_addr = reinterpret_cast<uint32_t>(buf);
	pdc_spi_packet.ul_size = transferLength;
	pdc_rx_init(spi_pdc, &pdc_spi_packet, NULL);
#endif

#if USE_DMAC
	DMAC->DMAC_EBCISR;		// clear any pending interrupts

	dmac_channel_set_source_addr(DMAC, DmacChanWiFiRx, reinterpret_cast<uint32_t>(&(ESP_SPI->SPI_RDR)));
	dmac_channel_set_destination_addr(DMAC, DmacChanWiFiRx, reinterpret_cast<uint32_t>(buf));
	dmac_channel_set_descriptor_addr(DMAC, DmacChanWiFiRx, 0);
	dmac_channel_set_ctrlA(DMAC, DmacChanWiFiRx, transferLength | DMAC_CTRLA_SRC_WIDTH_BYTE | DMAC_CTRLA_DST_WIDTH_WORD);
	dmac_channel_set_ctrlB(DMAC, DmacChanWiFiRx,
		DMAC_CTRLB_SRC_DSCR | DMAC_CTRLB_DST_DSCR | DMAC_CTRLB_FC_PER2MEM_DMA_FC | DMAC_CTRLB_SRC_INCR_FIXED | DMAC_CTRLB_DST_INCR_INCREMENTING);
#endif

#if USE_XDMAC
	xdmac_disable_interrupt(XDMAC, DmacChanWiFiRx);
	const uint32_t xdmaint = (XDMAC_CIE_BIE |
			XDMAC_CIE_DIE   |
			XDMAC_CIE_FIE   |
			XDMAC_CIE_RBIE  |
			XDMAC_CIE_WBIE  |
			XDMAC_CIE_ROIE);

	xdmac_rx_cfg.mbr_ubc = transferLength;
	xdmac_rx_cfg.mbr_da = reinterpret_cast<uint32_t>(buf);
	xdmac_rx_cfg.mbr_sa = reinterpret_cast<uint32_t>(&(ESP_SPI->SPI_RDR));
	xdmac_rx_cfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
			XDMAC_CC_MBSIZE_SINGLE |
			XDMAC_CC_DSYNC_PER2MEM |
			XDMAC_CC_CSIZE_CHK_1 |
			XDMAC_CC_DWIDTH_BYTE|
			XDMAC_CC_SIF_AHB_IF1 |
			XDMAC_CC_DIF_AHB_IF0 |
			XDMAC_CC_SAM_FIXED_AM |
			XDMAC_CC_DAM_INCREMENTED_AM |
			XDMAC_CC_PERID(SPI0_XDMAC_RX_CH_NUM);
	xdmac_rx_cfg.mbr_bc = 0;
	xdmac_tx_cfg.mbr_ds = 0;
	xdmac_rx_cfg.mbr_sus = 0;
	xdmac_rx_cfg.mbr_dus = 0;
	xdmac_configure_transfer(XDMAC, DmacChanWiFiRx, &xdmac_rx_cfg);

	xdmac_channel_set_descriptor_control(XDMAC, DmacChanWiFiRx, 0);
	xdmac_channel_disable_interrupt(XDMAC, DmacChanWiFiRx, xdmaint);
#endif

#if USE_DMAC_MANAGER
	DmacManager::SetSourceAddress(DmacChanWiFiRx, &(WiFiSpiSercom->SPI.DATA.reg));
	DmacManager::SetDestinationAddress(DmacChanWiFiRx, buf);
	DmacManager::SetBtctrl(DmacChanWiFiRx, DMAC_BTCTRL_STEPSIZE_X1 | DMAC_BTCTRL_STEPSEL_DST | DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_BEATSIZE_WORD | DMAC_BTCTRL_BLOCKACT_INT);
	DmacManager::SetDataLength(DmacChanWiFiRx, (transferLength + 3) >> 2);			// must do this one last
	DmacManager::SetTriggerSourceSercomRx(DmacChanWiFiRx, WiFiSpiSercomNumber);
#endif
}

/**
 * \brief Set SPI slave transfer.
 */
void WiFiInterface::spi_slave_dma_setup(uint32_t dataOutSize, uint32_t dataInSize) noexcept
{
	spi_dma_disable();					// if we don't do this we get strange crashes on the Duet 3 Mini
	DisableSpi();
	spi_rx_dma_setup(bufferIn, dataInSize + sizeof(MessageHeaderEspToSam));
	spi_tx_dma_setup(bufferOut, dataOutSize + sizeof(MessageHeaderSamToEsp));
	spi_dma_enable();
}

// Set up the SPI system
void WiFiInterface::SetupSpi() noexcept
{
	// Initialise the DMAC
#if USE_PDC
	spi_pdc = spi_get_pdc_base(ESP_SPI);
	// The PDCs are masters 2 and 3 and the SRAM is slave 0. Give the PDCs the highest priority.
	matrix_set_master_burst_type(0, MATRIX_ULBT_8_BEAT_BURST);
	matrix_set_slave_default_master_type(0, MATRIX_DEFMSTR_LAST_DEFAULT_MASTER);
	matrix_set_slave_priority(0, (3 << MATRIX_PRAS0_M2PR_Pos) | (3 << MATRIX_PRAS0_M3PR_Pos));
	matrix_set_slave_slot_cycle(0, 8);
#endif

#if USE_DMAC
	pmc_enable_periph_clk(ID_DMAC);
	dmac_init(DMAC);
	dmac_set_priority_mode(DMAC, DMAC_PRIORITY_ROUND_ROBIN);
	dmac_enable(DMAC);

	// The DMAC is master 4 and the SRAM is slave 0. Give the DMAC the highest priority.
	matrix_set_slave_default_master_type(0, MATRIX_DEFMSTR_LAST_DEFAULT_MASTER);
	matrix_set_slave_priority(0, (3 << MATRIX_PRAS0_M4PR_Pos));
	// Set the slave slot cycle limit.
	// If we leave it at the default value of 511 clock cycles, we get transmit underruns due to the HSMCI using the bus for too long.
	// A value of 8 seems to work. I haven't tried other values yet.
	matrix_set_slave_slot_cycle(0, 8);
#endif

#if USE_XDMAC
	pmc_enable_periph_clk(ID_XDMAC);
#endif

#if USE_DMAC_MANAGER
	// Nothing to do here
#endif

	// Set up the SPI pins
#if SAME5x
	for (Pin p : WiFiSpiSercomPins)
	{
		SetPinFunction(p, WiFiSpiSercomPinsMode);
	}

	Serial::EnableSercomClock(WiFiSpiSercomNumber);
#else
	SetPinFunction(APIN_ESP_SPI_SCK, SPIPeriphMode);
	SetPinFunction(APIN_ESP_SPI_MOSI, SPIPeriphMode);
	SetPinFunction(APIN_ESP_SPI_MISO, SPIPeriphMode);
	SetPinFunction(APIN_ESP_SPI_SS0, SPIPeriphMode);

	pmc_enable_periph_clk(ESP_SPI_INTERFACE_ID);
#endif

	spi_dma_disable();
	ResetSpi();									// on the SAM4E this clears the transmit and receive registers and put the SPI into slave mode

#if USE_DMAC
	// Configure DMA RX channel
	dmac_channel_set_configuration(DMAC, DmacChanWiFiRx,
			DMAC_CFG_SRC_PER(DMA_HW_ID_SPI_RX) | DMAC_CFG_SRC_H2SEL | DMAC_CFG_SOD | DMAC_CFG_FIFOCFG_ASAP_CFG);

	// Configure DMA TX channel
	dmac_channel_set_configuration(DMAC, DmacChanWiFiTx,
			DMAC_CFG_DST_PER(DMA_HW_ID_SPI_TX) | DMAC_CFG_DST_H2SEL | DMAC_CFG_SOD | DMAC_CFG_FIFOCFG_ASAP_CFG);
#endif

#if SAME5x
	WiFiSpiSercom->SPI.INTENCLR.reg = 0xFF;		// disable all interrupts
	WiFiSpiSercom->SPI.INTFLAG.reg = 0xFF;		// clear any pending interrupts
#else
	(void)ESP_SPI->SPI_SR;						// clear any pending interrupt
	ESP_SPI->SPI_IDR = SPI_IER_NSSR;			// disable the interrupt
#endif

	NVIC_SetPriority(ESP_SPI_IRQn, NvicPrioritySpi);
	NVIC_EnableIRQ(ESP_SPI_IRQn);
}

#endif //end ifndef __LPC17xx__

// Send a command to the ESP and get the result
int32_t WiFiInterface::SendCommand(NetworkCommand cmd, SocketNumber socketNum, uint8_t flags, uint32_t param32, const void *dataOut, size_t dataOutLength, void* dataIn, size_t dataInLength) noexcept
{
	if (GetState() == NetworkState::disabled)
	{
		if (reprap.Debug(moduleNetwork))
		{
			debugPrintf("ResponseNetworkDisabled\n");
		}
		return ResponseNetworkDisabled;
	}

	MutexLocker lock(interfaceMutex);

	if (transferPending)
	{
		if (reprap.Debug(moduleNetwork))
		{
			debugPrintf("ResponseBusy\n");
		}
		++transferAlreadyPendingCount;
		return ResponseBusy;
	}

	// Wait for the ESP to be ready, with timeout
	{
		const uint32_t now = millis();
		while (!digitalRead(EspDataReadyPin) || !digitalRead(SamCsPin))
		{
			if (millis() - now > WiFiWaitReadyMillis)
			{
				if (reprap.Debug(moduleNetwork))
				{
					debugPrintf("ResponseBusy\n");
				}
				++readyTimeoutCount;
				return ResponseBusy;
			}
		}
	}

	bufferOut->hdr.formatVersion = MyFormatVersion;
	bufferOut->hdr.command = cmd;
	bufferOut->hdr.socketNumber = socketNum;
	bufferOut->hdr.flags = flags;
	bufferOut->hdr.param32 = param32;
	bufferOut->hdr.dataLength = (uint16_t)dataOutLength;
	bufferOut->hdr.dataBufferAvailable = (uint16_t)dataInLength;
	if (dataOut != nullptr)
	{
		memcpy(bufferOut->data, dataOut, dataOutLength);
	}
	bufferIn->hdr.formatVersion = InvalidFormatVersion;
	espWaitingTask = TaskBase::GetCallerTaskHandle();
	transferPending = true;

	Cache::FlushBeforeDMASend(bufferOut, (dataOut != nullptr) ? sizeof(bufferOut->hdr) + dataOutLength : sizeof(bufferOut->hdr));

#if SAME5x
    spi_slave_dma_setup(dataOutLength, dataInLength);
	WiFiSpiSercom->SPI.INTFLAG.reg = 0xFF;		// clear any pending interrupts
	WiFiSpiSercom->SPI.INTENSET.reg = SERCOM_SPI_INTENSET_SSL;	// enable the start of transfer (SS low) interrupt
	EnableSpi();
#elif defined(__LPC17xx__)
    spi_slave_dma_setup(dataOutLength, dataInLength);
#else
    // DMA may have transferred an extra word to the SPI transmit data register. We need to clear this.
	// The only way I can find to do this is to issue a software reset to the SPI system.
	// Fortunately, this leaves the SPI system in slave mode.
    ResetSpi();
	spi_set_bits_per_transfer(ESP_SPI, 0, SPI_CSR_BITS_8_BIT);

	// Set up the DMA controller
	spi_slave_dma_setup(dataOutLength, dataInLength);
	EnableSpi();

	// Enable the end-of transfer interrupt
	(void)ESP_SPI->SPI_SR;						// clear any pending interrupt
	ESP_SPI->SPI_IER = SPI_IER_NSSR;			// enable the NSS rising interrupt
#endif

	// Tell the ESP that we are ready to accept data
	digitalWrite(SamTfrReadyPin, true);

	// Wait until the DMA transfer is complete, with timeout
	do
	{
		if (!TaskBase::Take(WiFiResponseTimeoutMillis))
		{
			if (reprap.Debug(moduleNetwork))
			{
				debugPrintf("ResponseTimeout, pending=%d\n", (int)transferPending);
			}
			transferPending = false;
			spi_dma_disable();
			++responseTimeoutCount;
			return ResponseTimeout;
		}
	} while (transferPending);

	espWaitingTask = nullptr;

#if SAME5x
	{
		// We don't get an end-of-transfer interrupt, just a start-of-transfer one. So wait until SS is high, then disable the SPI.
		// The normal maximum block time is about 2K * 8/spi_clock_speed plus any pauses that the ESP takes, which at 26.7MHz clock rate is 620us plus pause time
		// However, when we send a command that involves writing to flash memory, then the flash write occurs between sending the header and the body, so it takes much longer
		const uint32_t startedWaitingAt = millis();
		const bool writingFlash = (   cmd == NetworkCommand::networkAddSsid || cmd == NetworkCommand::networkConfigureAccessPoint
								   || cmd == NetworkCommand::networkDeleteSsid || cmd == NetworkCommand::networkFactoryReset);
		while (!digitalRead(EspSSPin))
		{
			const uint32_t millisWaiting = millis() - startedWaitingAt;
			if (millisWaiting >= WiFiTransferTimeoutMillis)
			{
				return ResponseTimeout;
			}

			// The new RTOS SDK for the ESP8266 often interrupts out transfer task for long periods of time. So if the transfer is taking a while to complete, give up the CPU.
			// Also give up the CPU if we are writing to flash memory, because we know that takes a long time.
			if (writingFlash || millisWaiting >= 2)
			{
				delay(2);
			}
		}
		if (WiFiSpiSercom->SPI.STATUS.bit.BUFOVF)
		{
			++spiRxOverruns;
		}
		DisableSpi();
		spi_dma_disable();
	}
#else
	while (!spi_dma_check_rx_complete()) { }	// Wait for DMA to complete
#endif

	// Look at the response
	Cache::InvalidateAfterDMAReceive(&bufferIn->hdr, sizeof(bufferIn->hdr));
	if (bufferIn->hdr.formatVersion != MyFormatVersion)
	{
		if (reprap.Debug(moduleNetwork))
		{
			debugPrintf("bad format version %02x\n", bufferIn->hdr.formatVersion);
		}
		return ResponseBadReplyFormatVersion;
	}

	if (   (bufferIn->hdr.state == WiFiState::autoReconnecting || bufferIn->hdr.state == WiFiState::reconnecting)
		&& currentMode != WiFiState::autoReconnecting && currentMode != WiFiState::reconnecting
	   )
	{
		++reconnectCount;
	}

	currentMode = bufferIn->hdr.state;
	const int32_t response = bufferIn->hdr.response;
	if (response > 0 && dataIn != nullptr)
	{
		const size_t sizeToCopy = min<size_t>(dataInLength, (size_t)response);
		Cache::InvalidateAfterDMAReceive(bufferIn->data, sizeToCopy);
		memcpy(dataIn, bufferIn->data, sizeToCopy);
	}

	if (response < 0 && reprap.Debug(moduleNetwork))
	{
		debugPrintf("Network command %d socket %u returned error: %s\n", (int)cmd, socketNum, TranslateWiFiResponse(response));
	}

	return response;
}

void WiFiInterface::SendListenCommand(TcpPort port, NetworkProtocol protocol, unsigned int maxConnections) noexcept
{
	ListenOrConnectData lcb;
	lcb.port = port;
	lcb.protocol = protocol;
	lcb.remoteIp = AnyIp;
	lcb.maxConnections = maxConnections;
	SendCommand(NetworkCommand::networkListen, 0, 0, 0, &lcb, sizeof(lcb), nullptr, 0);
}

// Stop listening on a port
void WiFiInterface::StopListening(TcpPort port) noexcept
{
	SendListenCommand(port, AnyProtocol, 0);
}

// This is called when ESP is signalling to us that an error occurred or there was a state change
void WiFiInterface::GetNewStatus() noexcept
{
	struct MessageResponse
	{
		char messageBuffer[100];
	};
	Receiver<MessageResponse> rcvr;

	espStatusChanged = false;
	EnableEspInterrupt();

	const int32_t rslt = SendCommand(NetworkCommand::networkGetLastError, 0, 0, nullptr, 0, rcvr);
	rcvr.Value().messageBuffer[ARRAY_UPB(rcvr.Value().messageBuffer)] = 0;
	if (rslt < 0)
	{
		platform.MessageF(NetworkErrorMessage, "failed to retrieve WiFi status message: %s\n", TranslateWiFiResponse(rslt));
	}
	else if (rslt > 0 && rcvr.Value().messageBuffer[0] != 0)
	{
		platform.MessageF(NetworkErrorMessage, "WiFi module reported: %s\n", rcvr.Value().messageBuffer);
	}
}

/*static*/ const char* WiFiInterface::TranslateWiFiResponse(int32_t response) noexcept
{
	switch (response)
	{
	case ResponseUnknownCommand:			return "unknown command";
	case ResponseBadRequestFormatVersion:	return "bad request format version";
	case ResponseTooManySsids:				return "too many stored SSIDs";
	case ResponseWrongState:				return "wrong WiFi module state";
	case ResponseBadDataLength:				return "bad data length";
	case ResponseNetworkDisabled:			return "WiFi module is disabled";
	case ResponseTimeout:					return "SPI timeout";
	case ResponseBusy:						return "another SPI transfer is pending";
	case ResponseBufferTooSmall:			return "response buffer too small";
	case ResponseBadReplyFormatVersion:		return "bad reply format version";
	case ResponseBadParameter:				return "bad parameter in request";
	case ResponseUnknownError:				return "unknown error";
	default:								return "unknown response code";
	}
}

#if !defined(__LPC17xx__)

# ifndef ESP_SPI_HANDLER
#  error ESP_SPI_HANDLER not defined
# endif

// SPI interrupt handler, called when NSS goes high (SAM4E, SAME70) or low (SAME5x)
void ESP_SPI_HANDLER() noexcept
{
	wifiInterface->SpiInterrupt();
}

void WiFiInterface::SpiInterrupt() noexcept
{
#if SAME5x
	// On the SAM5x we can't get an end-of-transfer interrupt, only a start-of-transfer interrupt.
	// So we can't disable SPI or DMA in this ISR.
	const uint8_t status = WiFiSpiSercom->SPI.INTFLAG.reg;
	if ((status & SERCOM_SPI_INTENSET_SSL) != 0)
	{
		WiFiSpiSercom->SPI.INTENCLR.reg = SERCOM_SPI_INTENSET_SSL;		// disable the interrupt
		WiFiSpiSercom->SPI.INTFLAG.reg = SERCOM_SPI_INTENSET_SSL;		// clear the status
#else
	const uint32_t status = ESP_SPI->SPI_SR;							// read status and clear interrupt
	ESP_SPI->SPI_IDR = SPI_IER_NSSR;									// disable the interrupt
	if ((status & SPI_SR_NSSR) != 0)
	{

# if USE_PDC
		pdc_disable_transfer(spi_pdc, PERIPH_PTCR_TXTDIS | PERIPH_PTCR_RXTDIS);
# endif

# if USE_DMAC
		spi_tx_dma_disable();
		dmac_channel_suspend(DMAC, DmacChanWiFiRx);						// suspend the receive channel, don't disable it because the FIFO needs to empty first
# endif

# if USE_XDMAC
		spi_tx_dma_disable();
		xdmac_channel_readwrite_suspend(XDMAC, DmacChanWiFiRx);			// suspend the receive channel
# endif

		DisableSpi();
		if ((status & SPI_SR_OVRES) != 0)
		{
			++spiRxOverruns;
		}
		if ((status & SPI_SR_UNDES) != 0)
		{
			++spiTxUnderruns;
		}
#endif
		if (transferPending)
		{
			digitalWrite(SamTfrReadyPin, false);							// stop signalling that we are ready for another transfer
			transferPending = false;
			TaskBase::GiveFromISR(espWaitingTask);
		}
	}
}

#endif //ifndef __LPC17xx__

// Start the ESP
void WiFiInterface::StartWiFi() noexcept
{
	digitalWrite(EspResetPin, true);

#if defined(DUET_NG) || defined(DUET3MINI)
	delayMicroseconds(150);										// ESP8266 datasheet specifies minimum 100us from releasing reset to power up
	digitalWrite(EspEnablePin, true);
#endif

#if !SAME5x && !defined(__LPC17xx__)
	SetPinFunction(APIN_Serial1_TXD, Serial1PeriphMode);				// connect the pins to the UART
	SetPinFunction(APIN_Serial1_RXD, Serial1PeriphMode);				// connect the pins to the UART
#endif
	SERIAL_WIFI_DEVICE.begin(WiFiBaudRate);						// initialise the UART, to receive debug info
	debugMessageChars = 0;
	serialRunning = true;
	debugPrintPending = false;
}

// Reset the ESP8266 and leave held in reset
void WiFiInterface::ResetWiFi() noexcept
{
	pinMode(EspResetPin, OUTPUT_LOW);							// assert ESP8266 /RESET

#if defined(DUET_NG) || defined(DUET3MINI)
	pinMode(EspEnablePin, OUTPUT_LOW);
#endif

#if !defined(SAME5x)
	pinMode(APIN_Serial1_TXD, INPUT_PULLUP);					// just enable pullups on TxD and RxD pins
	pinMode(APIN_Serial1_RXD, INPUT_PULLUP);
#endif
	currentMode = WiFiState::disabled;

	if (serialRunning)
	{
		SERIAL_WIFI_DEVICE.end();
		serialRunning = false;
	}
}

// Reset the ESP8266 to take commands from the UART or from external input. The caller must wait for the reset to complete after calling this.
// ESP8266 boot modes:
// GPIO0	GPIO2	GPIO15
// 0		1		0		Firmware download from UART
// 1		1		0		Normal boot from flash memory
// 0		0		1		SD card boot (not used in on Duet)
void WiFiInterface::ResetWiFiForUpload(bool external) noexcept
{
	if (serialRunning)
	{
		SERIAL_WIFI_DEVICE.end();
		serialRunning = false;
	}

	// Make sure the ESP8266 is in the reset state
	pinMode(EspResetPin, OUTPUT_LOW);

#if defined(DUET_NG) || defined(DUET3MINI)
	// Power down the ESP8266
	pinMode(EspEnablePin, OUTPUT_LOW);
#endif

	// Set up our transfer request pin (GPIO4) as an output and set it low
	pinMode(SamTfrReadyPin, OUTPUT_LOW);

	// Set up our data ready pin (ESP GPIO0) as an output and set it low ready to boot the ESP from UART
	pinMode(EspDataReadyPin, OUTPUT_LOW);

	// GPIO2 also needs to be high to boot up. It's connected to MISO on the SAM, so set the pullup resistor on that pin
	pinMode(APIN_ESP_SPI_MISO, INPUT_PULLUP);

	// Set our CS input (ESP GPIO15) low ready for booting the ESP. This also clears the transfer ready latch.
	pinMode(SamCsPin, OUTPUT_LOW);

	// Make sure it has time to reset - no idea how long it needs, but 50ms should be plenty
	delay(50);

	if (external)
	{
#if !defined(DUET3MINI)
		pinMode(APIN_Serial1_TXD, INPUT_PULLUP);					// just enable pullups on TxD and RxD pins
		pinMode(APIN_Serial1_RXD, INPUT_PULLUP);
#endif
	}
	else
	{
#if !SAME5x && !defined(__LPC17xx__)
		SetPinFunction(APIN_Serial1_TXD, Serial1PeriphMode);				// connect the pins to the UART
		SetPinFunction(APIN_Serial1_RXD, Serial1PeriphMode);				// connect the pins to the UART
#endif
	}

	// Release the reset on the ESP8266
	digitalWrite(EspResetPin, true);

#if defined(DUET_NG) || defined(DUET3MINI)
	// Take the ESP8266 out of power down
	delayMicroseconds(150);											// ESP8266 datasheet specifies minimum 100us from releasing reset to power up
	digitalWrite(EspEnablePin, true);
#endif
}

#endif	// HAS_WIFI_NETWORKING

// End
