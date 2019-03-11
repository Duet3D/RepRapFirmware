/*
 * WiFiInterface.cpp
 *
 *  Created on: 27 Nov 2017
 *      Authors: Christian and David
 */


#include "WiFiInterface.h"
#include "Platform.h"
#include "RepRap.h"
#include "GCodes/GCodeBuffer.h"
#include "HttpResponder.h"
#include "FtpResponder.h"
#include "TelnetResponder.h"
#include "WifiFirmwareUploader.h"
#include "General/IP4String.h"
#include "WiFiSocket.h"

static_assert(SsidLength == SsidBufferLength, "SSID lengths in NetworkDefs.h and MessageFormats.h don't match");

// Define exactly one of the following as 1, the other as zero

#if defined(DUET_NG)

// The PDC seems to be too slow to work reliably without getting transmit underruns, so we use the DMAC now.
# define USE_PDC			0		// use peripheral DMA controller
# define USE_DMAC			1		// use general DMA controller
# define USE_XDMAC			0		// use XDMA controller

# define ESP_SPI			SPI
# define ESP_SPI_ID			ID_SPI
# define ESP_SPI_IRQn		SPI_IRQn
# define ESP_SPI_HANDLER	SPI_Handler

#elif defined(DUET3_V03) || defined(SAME70XPLD)

# define USE_PDC			0		// use peripheral DMA controller
# define USE_DMAC			0		// use general DMA controller
# define USE_XDMAC			1		// use XDMA controller

# define ESP_SPI			SPI0
# define ESP_SPI_ID			ID_SPI0
# define ESP_SPI_IRQn		SPI0_IRQn
# define ESP_SPI_HANDLER	SPI0_Handler

#else
# error Unknown board
#endif

#if USE_PDC
#include "pdc/pdc.h"
#endif

#if USE_DMAC
#include "dmac/dmac.h"
#endif

#if USE_XDMAC
#include "xdmac/xdmac.h"
#endif

#include "matrix/matrix.h"

const uint32_t WifiResponseTimeoutMillis = 200;
const uint32_t WiFiWaitReadyMillis = 100;
const uint32_t WiFiStartupMillis = 300;
const uint32_t WiFiStableMillis = 100;

const unsigned int MaxHttpConnections = 4;

// Forward declarations of static functions
static void spi_dma_disable();
static bool spi_dma_check_rx_complete();

struct MessageBufferOut
{
	MessageHeaderSamToEsp hdr;
	uint8_t data[MaxDataLength];	// data to send
};

struct MessageBufferIn
{
	MessageHeaderEspToSam hdr;
	uint8_t data[MaxDataLength];	// data to send
};

static MessageBufferOut bufferOut;
static MessageBufferIn bufferIn;
static volatile bool transferPending = false;
static WiFiInterface *wifiInterface;

#if 0
static void debugPrintBuffer(const char *msg, void *buf, size_t dataLength)
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

static void EspTransferRequestIsr(CallbackParameter)
{
	wifiInterface->EspRequestsTransfer();
}

static inline void EnableEspInterrupt()
{
	attachInterrupt(EspDataReadyPin, EspTransferRequestIsr, INTERRUPT_MODE_RISING, nullptr);
}

static inline void DisableEspInterrupt()
{
	detachInterrupt(EspDataReadyPin);
}

/*-----------------------------------------------------------------------------------*/
// WiFi interface class

WiFiInterface::WiFiInterface(Platform& p) : platform(p), uploader(nullptr), ftpDataPort(0), closeDataPort(false),
		state(NetworkState::disabled), requestedMode(WiFiState::disabled), currentMode(WiFiState::disabled), activated(false),
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
}

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(_ret) OBJECT_MODEL_FUNC_BODY(WiFiInterface, _ret)

const ObjectModelTableEntry WiFiInterface::objectModelTable[] =
{
	// These entries must be in alphabetical order
	{ "gateway", OBJECT_MODEL_FUNC(&(self->gateway)), TYPE_OF(IPAddress), ObjectModelTableEntry::none },
	{ "ip", OBJECT_MODEL_FUNC(&(self->ipAddress)), TYPE_OF(IPAddress), ObjectModelTableEntry::none },
	{ "name", OBJECT_MODEL_FUNC_NOSELF("wifi"), TYPE_OF(const char *), ObjectModelTableEntry::none },
	{ "netmask", OBJECT_MODEL_FUNC(&(self->netmask)), TYPE_OF(IPAddress), ObjectModelTableEntry::none },
};

DEFINE_GET_OBJECT_MODEL_TABLE(WiFiInterface)

#endif

void WiFiInterface::Init()
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

	uploader = new WifiFirmwareUploader(SERIAL_WIFI_DEVICE, *this);
	currentSocket = 0;
}

GCodeResult WiFiInterface::EnableProtocol(NetworkProtocol protocol, int port, int secure, const StringRef& reply)
{
	if (secure != 0 && secure != -1)
	{
		reply.copy("Error: this firmware does not support TLS");
	}
	else if (protocol < NumProtocols)
	{
		const Port portToUse = (port < 0) ? DefaultPortNumbers[protocol] : port;
		MutexLocker lock(interfaceMutex);

		if (portToUse != portNumbers[protocol] && state == NetworkState::active)
		{
			// We need to shut down and restart the protocol if it is active because the port number has changed
			ShutdownProtocol(protocol);
			protocolEnabled[protocol] = false;
		}
		portNumbers[protocol] = portToUse;
		if (!protocolEnabled[protocol])
		{
			protocolEnabled[protocol] = true;
			if (state == NetworkState::active)
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

GCodeResult WiFiInterface::DisableProtocol(NetworkProtocol protocol, const StringRef& reply)
{
	if (protocol < NumProtocols)
	{
		MutexLocker lock(interfaceMutex);

		if (state == NetworkState::active)
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

void WiFiInterface::StartProtocol(NetworkProtocol protocol)
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

void WiFiInterface::ShutdownProtocol(NetworkProtocol protocol)
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
GCodeResult WiFiInterface::ReportProtocols(const StringRef& reply) const
{
	reply.Clear();
	for (size_t i = 0; i < NumProtocols; ++i)
	{
		if (i != 0)
		{
			reply.cat('\n');
		}
		ReportOneProtocol(i, reply);
	}
	return GCodeResult::ok;
}

void WiFiInterface::ReportOneProtocol(NetworkProtocol protocol, const StringRef& reply) const
{
	if (protocolEnabled[protocol])
	{
		reply.catf("%s is enabled on port %u", ProtocolNames[protocol], portNumbers[protocol]);
	}
	else
	{
		reply.catf("%s is disabled", ProtocolNames[protocol]);
	}
}

NetworkProtocol WiFiInterface::GetProtocolByLocalPort(Port port) const
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
void WiFiInterface::Activate()
{
	if (!activated)
	{
		activated = true;
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

void WiFiInterface::Exit()
{
	Stop();
}

// Get the network state into the reply buffer, returning true if there is some sort of error
GCodeResult WiFiInterface::GetNetworkState(const StringRef& reply)
{
	switch (state)
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
void WiFiInterface::Start()
{
	// The ESP8266 is held in a reset state by a pulldown resistor until we enable it.
	// Make sure the ESP8266 is in the reset state
	pinMode(EspResetPin, OUTPUT_LOW);

#ifdef DUET_NG
	// Take the ESP8266 out of power down
	pinMode(EspEnablePin, OUTPUT_HIGH);
#endif

	// Set up our transfer request pin (GPIO4) as an output and set it low
	pinMode(SamTfrReadyPin, OUTPUT_LOW);

	// Set up our data ready pin (ESP GPIO0) as an output and set it high ready to boot the ESP from flash
	pinMode(EspDataReadyPin, OUTPUT_HIGH);

	// GPIO2 also needs to be high to boot. It's connected to MISO on the SAM, so set the pullup resistor on that pin
	pinMode(APIN_SPI_MISO, INPUT_PULLUP);

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
	state = NetworkState::starting1;
}

// Stop the ESP
void WiFiInterface::Stop()
{
	if (state != NetworkState::disabled)
	{
		MutexLocker lock(interfaceMutex);

		digitalWrite(SamTfrReadyPin, LOW);			// tell the ESP we can't receive
		digitalWrite(EspResetPin, LOW);				// put the ESP back into reset
		DisableEspInterrupt();						// ignore IRQs from the transfer request pin

		NVIC_DisableIRQ(ESP_SPI_IRQn);
		spi_disable(ESP_SPI);

		spi_dma_check_rx_complete();
		spi_dma_disable();

		state = NetworkState::disabled;
		currentMode = WiFiState::disabled;
	}
}

void WiFiInterface::Spin(bool full)
{
	// Main state machine.
	switch (state)
	{
	case NetworkState::starting1:
		if (full)
		{
			// The ESP toggles CS before it has finished starting up, so don't look at the CS signal too soon
			const uint32_t now = millis();
			if (now - lastTickMillis >= WiFiStartupMillis)
			{
				lastTickMillis = now;
				state = NetworkState::starting2;
			}
		}
		break;

	case NetworkState::starting2:
		if (full)
		{
			// See if the ESP8266 has kept its pins at their stable values for long enough
			const uint32_t now = millis();
			if (digitalRead(SamCsPin) && digitalRead(EspDataReadyPin) && !digitalRead(APIN_SPI_SCK))
			{
				if (now - lastTickMillis >= WiFiStableMillis)
				{
					// Setup the SPI controller in slave mode and assign the CS pin to it
					platform.Message(NetworkInfoMessage, "WiFi module started\n");
					SetupSpi();									// set up the SPI subsystem

					// Read the status to get the WiFi server version
					Receiver<NetworkStatusResponse> status;
					const int32_t rc = SendCommand(NetworkCommand::networkGetStatus, 0, 0, nullptr, 0, status);
					if (rc > 0)
					{
						SafeStrncpy(wiFiServerVersion, status.Value().versionText, ARRAY_SIZE(wiFiServerVersion));

						// Set the hostname before anything else is done
						if (SendCommand(NetworkCommand::networkSetHostName, 0, 0, reprap.GetNetwork().GetHostname(), HostNameLength, nullptr, 0) != ResponseEmpty)
						{
							reprap.GetPlatform().Message(NetworkInfoMessage, "Error: Could not set WiFi hostname\n");
						}

						state = NetworkState::active;
						espStatusChanged = true;				// make sure we fetch the current state and enable the ESP interrupt
					}
					else
					{
						// Something went wrong, maybe a bad firmware image was flashed
						// Disable the WiFi chip again in this case
						platform.MessageF(NetworkInfoMessage, "Error: Failed to initialise WiFi module, code %" PRIi32 "\n", rc);
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
		if (full && uploader != nullptr)
		{
			uploader->Spin();
		}
		break;

	case NetworkState::active:
		if (full)
		{
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
					rslt = SendCommand(NetworkCommand::networkStop, 0, 0, nullptr, 0, nullptr, 0);
				}
				else if (requestedMode == WiFiState::connected)
				{
					rslt = SendCommand(NetworkCommand::networkStartClient, 0, 0, requestedSsid, SsidLength, nullptr, 0);
				}
				else if (requestedMode == WiFiState::runningAsAccessPoint)
				{
					rslt = SendCommand(NetworkCommand::networkStartAccessPoint, 0, 0, nullptr, 0, nullptr, 0);
				}

				if (rslt >= 0)
				{
					state = NetworkState::changingMode;
				}
				else
				{
					Stop();
					platform.MessageF(NetworkInfoMessage, "Failed to change WiFi mode (code %" PRIi32 ")\n", rslt);
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
				sockets[currentSocket]->Poll(full);
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
		}
		break;

	case NetworkState::changingMode:
		// Here when we have asked the ESP to change mode. Don't leave this state until we have a new status report from the ESP.
		if (full && espStatusChanged && digitalRead(EspDataReadyPin))
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
				state = NetworkState::active;
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
				state = NetworkState::active;
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
		while (!debugPrintPending && Serial1.available() != 0)
		{
			const char c = (char)Serial1.read();
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

	if (full)
	{
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
}

// Translate a ESP8266 reset reason to text
const char* WiFiInterface::TranslateEspResetReason(uint32_t reason)
{
	// Mapping from known ESP reset codes to reasons
	static const char * const resetReasonTexts[] =
	{
		"Power on",
		"Hardware watchdog",
		"Exception",
		"Software watchdog",
		"Software restart",
		"Deep-sleep wakeup",
		"Turned on by main processor"
	};

	return (reason < sizeof(resetReasonTexts)/sizeof(resetReasonTexts[0]))
			? resetReasonTexts[reason]
			: "Unknown";
}

const char* WiFiInterface::TranslateNetworkState() const
{
	switch (state)
	{
	case NetworkState::disabled:		return "disabled";
	case NetworkState::starting1:
	case NetworkState::starting2:		return "starting";
	case NetworkState::active:			return "running";
	case NetworkState::changingMode:	return "changing mode";
	default:							return "unknown";
	}
}

void WiFiInterface::Diagnostics(MessageType mtype)
{
	platform.Message(mtype, "- WiFi -\n");
	platform.MessageF(mtype, "Network state is %s\n", TranslateNetworkState());
	platform.MessageF(mtype, "WiFi module is %s\n", TranslateWiFiState(currentMode));
	platform.MessageF(mtype, "Failed messages: pending %u, notready %u, noresp %u\n", transferAlreadyPendingCount, readyTimeoutCount, responseTimeoutCount);

#if 0
	// The underrun/overrun counters don't work at present
	platform.MessageF(mtype, "SPI underruns %u, overruns %u\n", spiTxUnderruns, spiRxOverruns);
#endif

	if (state != NetworkState::disabled && state != NetworkState::starting1 && state != NetworkState::starting2)
	{
		Receiver<NetworkStatusResponse> status;
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
				const char* const sleepMode = (r.sleepMode == 1) ? "none" : (r.sleepMode == 2) ? "light" : (r.sleepMode == 3) ? "modem" : "unknown";
				platform.MessageF(mtype, "WiFi signal strength %ddBm, reconnections %u, sleep mode %s\n", (int)r.rssi, reconnectCount, sleepMode);
			}
			else if (currentMode == WiFiState::runningAsAccessPoint)
			{
				platform.MessageF(mtype, "Connected clients %u\n", (unsigned int)r.numClients);
			}
			// status, ssid and hostName not displayed

			// Print LwIP stats and other values over the ESP's UART line
			if (SendCommand(NetworkCommand::diagnostics, 0, 0, nullptr, 0, nullptr, 0) != ResponseEmpty)
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
GCodeResult WiFiInterface::EnableInterface(int mode, const StringRef& ssid, const StringRef& reply)
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
			if (state != NetworkState::disabled)
			{
				Stop();
				platform.Message(GenericMessage, "WiFi module stopped\n");
			}
		}
		else
		{
			if (state == NetworkState::disabled)
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

int WiFiInterface::EnableState() const
{
	return (requestedMode == WiFiState::idle) ? 0
			: (requestedMode == WiFiState::connected) ? 1
				: (requestedMode == WiFiState::runningAsAccessPoint) ? 2
						: -1;
}

// Translate the wifi state to text.
// The 'connected' and 'runningAsAccessPoint' states include a space at the end because the caller is expected to append the access point name.
/*static*/ const char* WiFiInterface::TranslateWiFiState(WiFiState w)
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

void WiFiInterface::EspRequestsTransfer()
{
	espStatusChanged = true;
	DisableEspInterrupt();				// don't allow more interrupts until we have acknowledged this one
}

void WiFiInterface::SetIPAddress(IPAddress p_ip, IPAddress p_netmask, IPAddress p_gateway)
{
	ipAddress = p_ip;
	netmask = p_netmask;
	gateway = p_gateway;
}

GCodeResult WiFiInterface::HandleWiFiCode(int mcode, GCodeBuffer &gb, const StringRef& reply, OutputBuffer*& longReply)
{
	switch (mcode)
	{
	case 587:	// Add WiFi network or list remembered networks
		if (gb.Seen('S'))
		{
			WirelessConfigurationData config;
			memset(&config, 0, sizeof(config));
			String<ARRAY_SIZE(config.ssid)> ssid;
			bool ok = gb.GetQuotedString(ssid.GetRef());
			if (ok)
			{
				SafeStrncpy(config.ssid, ssid.c_str(), ARRAY_SIZE(config.ssid));
				String<ARRAY_SIZE(config.password)> password;
				ok = gb.Seen('P') && gb.GetQuotedString(password.GetRef());
				if (ok)
				{
					if (password.strlen() < 8 && password.strlen() != 0)			// WPA2 passwords must be at least 8 characters
					{
						reply.copy("WiFi password must be at least 8 characters");
						return GCodeResult::error;
					}
					SafeStrncpy(config.password, password.c_str(), ARRAY_SIZE(config.password));
				}
			}
			if (ok && gb.Seen('I'))
			{
				IPAddress temp;
				gb.GetIPAddress(temp);
				config.ip = temp.GetV4LittleEndian();
			}
			if (ok && gb.Seen('J'))
			{
				IPAddress temp;
				ok = gb.GetIPAddress(temp);
				config.gateway = temp.GetV4LittleEndian();
			}
			if (ok && gb.Seen('K'))
			{
				IPAddress temp;
				ok = gb.GetIPAddress(temp);
				config.netmask = temp.GetV4LittleEndian();
			}
			if (ok)
			{
				const int32_t rslt = SendCommand(NetworkCommand::networkAddSsid, 0, 0, &config, sizeof(config), nullptr, 0);
				if (rslt == ResponseEmpty)
				{
					return GCodeResult::ok;
				}
				else
				{
					reply.copy("Failed to add SSID to remembered list");
				}
			}
			else
			{
				reply.copy("Bad or missing parameter");
			}
		}
		else
		{
			// List remembered networks
			const size_t declaredBufferLength = (MaxRememberedNetworks + 1) * ReducedWirelessConfigurationDataSize;	// enough for all the remembered SSID data
			uint32_t buffer[NumDwords(declaredBufferLength)];
			const int32_t rslt = SendCommand(NetworkCommand::networkRetrieveSsidData, 0, 0, nullptr, 0, buffer, declaredBufferLength);
			if (rslt >= 0)
			{
				size_t offset = ReducedWirelessConfigurationDataSize;		// skip own SSID details
				while (offset + ReducedWirelessConfigurationDataSize <= (size_t)rslt)
				{
					WirelessConfigurationData* const wp = reinterpret_cast<WirelessConfigurationData *>(reinterpret_cast<char*>(buffer) + offset);
					if (wp->ssid[0] != 0)
					{
						if (longReply == nullptr)
						{
							if (!OutputBuffer::Allocate(longReply))
							{
								return GCodeResult::notFinished;			// try again later
							}
							longReply->copy("Remembered networks:");
						}
						wp->ssid[ARRAY_UPB(wp->ssid)] = 0;
						longReply->catf("\n%s IP=%s GW=%s NM=%s", wp->ssid, IP4String(wp->ip).c_str(), IP4String(wp->gateway).c_str(), IP4String(wp->netmask).c_str());
					}
					offset += ReducedWirelessConfigurationDataSize;
				}

				if (longReply == nullptr)
				{
					reply.copy("No remembered networks");
				}
				return GCodeResult::ok;
			}

			reply.copy("Failed to retrieve network list");
		}
		return GCodeResult::error;

	case 588:	// Forget WiFi network
		if (gb.Seen('S'))
		{
			String<SsidLength> ssidText;
			if (gb.GetQuotedString(ssidText.GetRef()))
			{
				if (strcmp(ssidText.c_str(), "*") == 0)
				{
					const int32_t rslt = SendCommand(NetworkCommand::networkFactoryReset, 0, 0, nullptr, 0, nullptr, 0);
					if (rslt == ResponseEmpty)
					{
						return GCodeResult::ok;
					}

					reply.copy("Failed to reset the WiFi module to factory settings");
					return GCodeResult::error;
				}

				uint32_t ssid32[NumDwords(SsidLength)];				// need a dword-aligned buffer for SendCommand
				memcpy(ssid32, ssidText.c_str(), SsidLength);
				const int32_t rslt = SendCommand(NetworkCommand::networkDeleteSsid, 0, 0, ssid32, SsidLength, nullptr, 0);
				if (rslt == ResponseEmpty)
				{
					return GCodeResult::ok;
				}

				reply.copy("Failed to remove SSID from remembered list");
				return GCodeResult::error;
			}
		}

		reply.copy("Bad or missing parameter");
		return GCodeResult::error;

	case 589:	// Configure access point
		if (gb.Seen('S'))
		{
			// Configure access point parameters
			WirelessConfigurationData config;
			memset(&config, 0, sizeof(config));
			String<SsidLength> ssid;
			bool ok = gb.GetQuotedString(ssid.GetRef());
			if (ok)
			{
				if (strcmp(ssid.c_str(), "*") == 0)
				{
					// Delete the access point details
					memset(&config, 0xFF, sizeof(config));
				}
				else
				{
					SafeStrncpy(config.ssid, ssid.c_str(), ARRAY_SIZE(config.ssid));
					String<ARRAY_SIZE(config.password)> password;
					ok = gb.Seen('P') && gb.GetQuotedString(password.GetRef());
					if (ok)
					{
						SafeStrncpy(config.password, password.c_str(), ARRAY_SIZE(config.password));
						if (password.strlen() < 8 && password.strlen() != 0)			// WPA2 passwords must be at least 8 characters
						{
							reply.copy("WiFi password must be at least 8 characters");
							return GCodeResult::error;
						}
						SafeStrncpy(config.password, password.c_str(), ARRAY_SIZE(config.password));
						if (gb.Seen('I'))
						{
							IPAddress temp;
							ok = gb.GetIPAddress(temp);
							config.ip = temp.GetV4LittleEndian();
							config.channel = (gb.Seen('C')) ? gb.GetIValue() : 0;
						}
						else
						{
							ok = false;
						}
					}
				}
			}
			if (ok)
			{
				const int32_t rslt = SendCommand(NetworkCommand::networkConfigureAccessPoint, 0, 0, &config, sizeof(config), nullptr, 0);
				if (rslt == ResponseEmpty)
				{
					return GCodeResult::ok;
				}

				reply.copy("Failed to configure access point parameters");
			}
			else
			{
				reply.copy("Bad or missing parameter");
			}
		}
		else
		{
			// Report access point parameters
			uint32_t buffer[NumDwords(ReducedWirelessConfigurationDataSize)];
			const int32_t rslt = SendCommand(NetworkCommand::networkRetrieveSsidData, 0, 0, nullptr, 0, buffer, ReducedWirelessConfigurationDataSize);
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
				reply.copy("Failed to retrieve own SSID data");
			}
		}
		return GCodeResult::error;

	default:	// should not happen
		return GCodeResult::error;
	}
}

// Set the DHCP hostname
void WiFiInterface::UpdateHostname(const char *hostname)
{
	// Update the hostname if possible
	if (state == NetworkState::active)
	{
		if (SendCommand(NetworkCommand::networkSetHostName, 0, 0, hostname, HostNameLength, nullptr, 0) != ResponseEmpty)
		{
			platform.Message(GenericMessage, "Error: Could not set WiFi hostname\n");
		}
	}
}

void WiFiInterface::SetMacAddress(const uint8_t mac[])
{
	for (size_t i = 0; i < 6; i++)
	{
		macAddress[i] = mac[i];
	}
	// TODO actually update the mac address on the wifi module. For now we don't support this.
}

void WiFiInterface::InitSockets()
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

void WiFiInterface::TerminateSockets()
{
	for (SocketNumber skt = 0; skt < NumWiFiTcpSockets; ++skt)
	{
		sockets[skt]->Terminate();
	}
}

void WiFiInterface::TerminateSockets(Port port)
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
void WiFiInterface::UpdateSocketStatus(uint16_t connectedSockets, uint16_t otherEndClosedSockets)
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
void WiFiInterface::OpenDataPort(Port port)
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
void WiFiInterface::TerminateDataPort()
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

#if USE_PDC
static Pdc *spi_pdc;
#endif

#if USE_DMAC

// Our choice of DMA channels to use
const uint32_t CONF_SPI_DMAC_TX_CH = 1;
const uint32_t CONF_SPI_DMAC_RX_CH = 2;

// Hardware IDs of the SPI transmit and receive DMA interfaces. See atsam datasheet.
const uint32_t DMA_HW_ID_SPI_TX = 1;
const uint32_t DMA_HW_ID_SPI_RX = 2;

#endif

#if USE_XDMAC

// XDMAC hardware
const uint32_t SPI0_XDMAC_TX_CH_NUM = 1;
const uint32_t SPI0_XDMAC_RX_CH_NUM = 2;

static xdmac_channel_config_t xdmac_tx_cfg, xdmac_rx_cfg;

#endif

static inline void spi_rx_dma_enable()
{
#if USE_PDC
	pdc_enable_transfer(spi_pdc, PERIPH_PTCR_RXTEN);
#endif

#if USE_DMAC
	dmac_channel_enable(DMAC, CONF_SPI_DMAC_RX_CH);
#endif

#if USE_XDMAC
	xdmac_channel_enable(XDMAC, DmacChanWiFiRx);
#endif
}

static inline void spi_tx_dma_enable()
{
#if USE_PDC
	pdc_enable_transfer(spi_pdc, PERIPH_PTCR_TXTEN);
#endif

#if USE_DMAC
	dmac_channel_enable(DMAC, CONF_SPI_DMAC_TX_CH);
#endif

#if USE_XDMAC
	xdmac_channel_enable(XDMAC, DmacChanWiFiTx);
#endif
}

static inline void spi_rx_dma_disable()
{
#if USE_PDC
	pdc_disable_transfer(spi_pdc, PERIPH_PTCR_RXTDIS);
#endif

#if USE_DMAC
	dmac_channel_disable(DMAC, CONF_SPI_DMAC_RX_CH);
#endif

#if USE_XDMAC
	xdmac_channel_disable(XDMAC, DmacChanWiFiRx);
#endif
}

static inline void spi_tx_dma_disable()
{
#if USE_PDC
	pdc_disable_transfer(spi_pdc, PERIPH_PTCR_TXTDIS);
#endif

#if USE_DMAC
	dmac_channel_disable(DMAC, CONF_SPI_DMAC_TX_CH);
#endif

#if USE_XDMAC
	xdmac_channel_disable(XDMAC, DmacChanWiFiTx);
#endif
}

static void spi_dma_disable()
{
	spi_tx_dma_disable();
	spi_rx_dma_disable();
}

static inline void spi_dma_enable()
{
#if USE_PDC
	pdc_enable_transfer(spi_pdc, PERIPH_PTCR_TXTEN | PERIPH_PTCR_RXTEN);
#endif

#if USE_DMAC || USE_XDMAC
	spi_rx_dma_enable();
	spi_tx_dma_enable();
#endif
}

static bool spi_dma_check_rx_complete()
{
#if USE_PDC
	return true;
#endif

#if USE_DMAC
	const uint32_t status = DMAC->DMAC_CHSR;
	if (   ((status & (DMAC_CHSR_ENA0 << CONF_SPI_DMAC_RX_CH)) == 0)	// controller is not enabled, perhaps because it finished a full buffer transfer
		|| ((status & (DMAC_CHSR_EMPT0 << CONF_SPI_DMAC_RX_CH)) != 0)	// controller is enabled, probably suspended, and the FIFO is empty
	   )
	{
		// Disable the channel.
		// We also need to set the resume bit, otherwise it remains suspended when we re-enable it.
		DMAC->DMAC_CHDR = (DMAC_CHDR_DIS0 << CONF_SPI_DMAC_RX_CH) | (DMAC_CHDR_RES0 << CONF_SPI_DMAC_RX_CH);
		return true;
	}
#endif

#if USE_XDMAC
	const uint32_t status = xdmac_channel_get_status(XDMAC);
	const uint32_t channelStatus = XDMAC->XDMAC_CHID[DmacChanWiFiRx].XDMAC_CC;
	if (	((status & (1 << DmacChanWiFiRx)) == 0)					// channel is not enabled
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

static void spi_tx_dma_setup(const void *buf, uint32_t transferLength)
{
#if USE_PDC
	pdc_packet_t pdc_spi_packet;
	pdc_spi_packet.ul_addr = reinterpret_cast<uint32_t>(buf);
	pdc_spi_packet.ul_size = transferLength;
	pdc_tx_init(spi_pdc, &pdc_spi_packet, NULL);
#endif

#if USE_DMAC
	DMAC->DMAC_EBCISR;		// clear any pending interrupts

	dmac_channel_set_source_addr(DMAC, CONF_SPI_DMAC_TX_CH, reinterpret_cast<uint32_t>(buf));
	dmac_channel_set_destination_addr(DMAC, CONF_SPI_DMAC_TX_CH, reinterpret_cast<uint32_t>(& SPI->SPI_TDR));
	dmac_channel_set_descriptor_addr(DMAC, CONF_SPI_DMAC_TX_CH, 0);
	dmac_channel_set_ctrlA(DMAC, CONF_SPI_DMAC_TX_CH, transferLength | DMAC_CTRLA_SRC_WIDTH_WORD | DMAC_CTRLA_DST_WIDTH_BYTE);
	dmac_channel_set_ctrlB(DMAC, CONF_SPI_DMAC_TX_CH,
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
	xdmac_tx_cfg.mbr_da = reinterpret_cast<uint32_t>(&(SPI0->SPI_TDR));
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
}

static void spi_rx_dma_setup(const void *buf, uint32_t transferLength)
{
#if USE_PDC
	pdc_packet_t pdc_spi_packet;
	pdc_spi_packet.ul_addr = reinterpret_cast<uint32_t>(buf);
	pdc_spi_packet.ul_size = transferLength;
	pdc_rx_init(spi_pdc, &pdc_spi_packet, NULL);
#endif

#if USE_DMAC
	DMAC->DMAC_EBCISR;		// clear any pending interrupts

	dmac_channel_set_source_addr(DMAC, CONF_SPI_DMAC_RX_CH, reinterpret_cast<uint32_t>(& SPI->SPI_RDR));
	dmac_channel_set_destination_addr(DMAC, CONF_SPI_DMAC_RX_CH, reinterpret_cast<uint32_t>(buf));
	dmac_channel_set_descriptor_addr(DMAC, CONF_SPI_DMAC_RX_CH, 0);
	dmac_channel_set_ctrlA(DMAC, CONF_SPI_DMAC_RX_CH, transferLength | DMAC_CTRLA_SRC_WIDTH_BYTE | DMAC_CTRLA_DST_WIDTH_WORD);
	dmac_channel_set_ctrlB(DMAC, CONF_SPI_DMAC_RX_CH,
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
	xdmac_rx_cfg.mbr_sa = reinterpret_cast<uint32_t>(&(SPI0->SPI_RDR));
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
}

/**
 * \brief Set SPI slave transfer.
 */
static void spi_slave_dma_setup(uint32_t dataOutSize, uint32_t dataInSize)
{
#if USE_PDC
	pdc_disable_transfer(spi_pdc, PERIPH_PTCR_TXTDIS | PERIPH_PTCR_RXTDIS);
	spi_rx_dma_setup(&bufferIn, dataInSize + sizeof(MessageHeaderEspToSam));
	spi_tx_dma_setup(&bufferOut, dataOutSize + sizeof(MessageHeaderSamToEsp));
#endif

#if USE_DMAC || USE_XDMAC
	spi_dma_disable();
	spi_rx_dma_setup(&bufferIn, dataInSize + sizeof(MessageHeaderEspToSam));
	spi_tx_dma_setup(&bufferOut, dataOutSize + sizeof(MessageHeaderSamToEsp));
#endif

	spi_dma_enable();
}

// Set up the SPI system
void WiFiInterface::SetupSpi()
{
#if USE_PDC
	spi_pdc = spi_get_pdc_base(SPI);
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

	// Set up the SPI pins
	ConfigurePin(g_APinDescription[APIN_SPI_SCK]);
	ConfigurePin(g_APinDescription[APIN_SPI_MOSI]);
	ConfigurePin(g_APinDescription[APIN_SPI_MISO]);
	ConfigurePin(g_APinDescription[APIN_SPI_SS0]);

	pmc_enable_periph_clk(ESP_SPI_ID);
	spi_dma_disable();
	spi_reset(ESP_SPI);				// this clears the transmit and receive registers and puts the SPI into slave mode

#if USE_DMAC
	// Configure DMA RX channel
	dmac_channel_set_configuration(DMAC, CONF_SPI_DMAC_RX_CH,
			DMAC_CFG_SRC_PER(DMA_HW_ID_SPI_RX) | DMAC_CFG_SRC_H2SEL | DMAC_CFG_SOD | DMAC_CFG_FIFOCFG_ASAP_CFG);

	// Configure DMA TX channel
	dmac_channel_set_configuration(DMAC, CONF_SPI_DMAC_TX_CH,
			DMAC_CFG_DST_PER(DMA_HW_ID_SPI_TX) | DMAC_CFG_DST_H2SEL | DMAC_CFG_SOD | DMAC_CFG_FIFOCFG_ASAP_CFG);
#endif

	(void)ESP_SPI->SPI_SR;				// clear any pending interrupt
	ESP_SPI->SPI_IDR = SPI_IER_NSSR;	// disable the interrupt

	NVIC_SetPriority(ESP_SPI_IRQn, NvicPrioritySpi);
	NVIC_EnableIRQ(ESP_SPI_IRQn);
}

// Send a command to the ESP and get the result
int32_t WiFiInterface::SendCommand(NetworkCommand cmd, SocketNumber socketNum, uint8_t flags, const void *dataOut, size_t dataOutLength, void* dataIn, size_t dataInLength)
{
	if (state == NetworkState::disabled)
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

	bufferOut.hdr.formatVersion = MyFormatVersion;
	bufferOut.hdr.command = cmd;
	bufferOut.hdr.socketNumber = socketNum;
	bufferOut.hdr.flags = flags;
	bufferOut.hdr.param32 = 0;
	bufferOut.hdr.dataLength = (uint16_t)dataOutLength;
	bufferOut.hdr.dataBufferAvailable = (uint16_t)dataInLength;
	if (dataOut != nullptr)
	{
		memcpy(bufferOut.data, dataOut, dataOutLength);
	}
	bufferIn.hdr.formatVersion = InvalidFormatVersion;
	transferPending = true;

	// DMA may have transferred an extra word to the SPI transmit data register. We need to clear this.
	// The only way I can find to do this is to issue a software reset to the SPI system.
	// Fortunately, this leaves the SPI system in slave mode.
	spi_reset(ESP_SPI);
	spi_set_bits_per_transfer(ESP_SPI, 0, SPI_CSR_BITS_8_BIT);

	// Set up the DMA controller
	spi_slave_dma_setup(dataOutLength, dataInLength);
	spi_enable(ESP_SPI);

	// Enable the end-of transfer interrupt
	(void)ESP_SPI->SPI_SR;						// clear any pending interrupt
	ESP_SPI->SPI_IER = SPI_IER_NSSR;			// enable the NSS rising interrupt

	// Tell the ESP that we are ready to accept data
	digitalWrite(SamTfrReadyPin, HIGH);

	// Wait for the DMA complete interrupt, with timeout
	// When we use RTOS we should allow other tasks to run here
	{
		const uint32_t now = millis();
		while (transferPending || !spi_dma_check_rx_complete())
		{
			if (digitalRead(SamCsPin) && millis() - now > WifiResponseTimeoutMillis)	// if no transfer in progress and timed out
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
		}
	}

	// Look at the response
	if (bufferIn.hdr.formatVersion != MyFormatVersion)
	{
		if (reprap.Debug(moduleNetwork))
		{
			debugPrintf("bad format version %02x\n", bufferIn.hdr.formatVersion);
		}
		return ResponseBadReplyFormatVersion;
	}

	if (   (bufferIn.hdr.state == WiFiState::autoReconnecting || bufferIn.hdr.state == WiFiState::reconnecting)
			&& currentMode != WiFiState::autoReconnecting && currentMode != WiFiState::reconnecting
		   )
	{
		++reconnectCount;
	}

	currentMode = bufferIn.hdr.state;
	const int32_t response = bufferIn.hdr.response;
	if (response > 0 && dataIn != nullptr)
	{
		memcpy(dataIn, bufferIn.data, min<size_t>(dataInLength, (size_t)response));
	}

	if (response < 0 && reprap.Debug(moduleNetwork))
	{
		debugPrintf("Network command %d socket %u returned error: %s\n", (int)cmd, socketNum, TranslateWiFiResponse(response));
	}

	return response;
}

void WiFiInterface::SendListenCommand(Port port, NetworkProtocol protocol, unsigned int maxConnections)
{
	ListenOrConnectData lcb;
	lcb.port = port;
	lcb.protocol = protocol;
	lcb.remoteIp = AnyIp;
	lcb.maxConnections = maxConnections;
	SendCommand(NetworkCommand::networkListen, 0, 0, &lcb, sizeof(lcb), nullptr, 0);
}

// Stop listening on a port
void WiFiInterface::StopListening(Port port)
{
	SendListenCommand(port, AnyProtocol, 0);
}

// This is called when ESP is signalling to us that an error occurred or there was a state change
void WiFiInterface::GetNewStatus()
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
		platform.MessageF(NetworkInfoMessage, "Error retrieving WiFi status message: %s\n", TranslateWiFiResponse(rslt));
	}
	else if (rslt > 0 && rcvr.Value().messageBuffer[0] != 0)
	{
		platform.MessageF(NetworkInfoMessage, "WiFi reported error: %s\n", rcvr.Value().messageBuffer);
	}
}

/*static*/ const char* WiFiInterface::TranslateWiFiResponse(int32_t response)
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

// SPI interrupt handlers, called when NSS goes high
void ESP_SPI_HANDLER(void)
{
	wifiInterface->SpiInterrupt();
}

void WiFiInterface::SpiInterrupt()
{
	const uint32_t status = ESP_SPI->SPI_SR;							// read status and clear interrupt
	ESP_SPI->SPI_IDR = SPI_IER_NSSR;									// disable the interrupt
	if ((status & SPI_SR_NSSR) != 0)
	{
#if USE_PDC
		pdc_disable_transfer(spi_pdc, PERIPH_PTCR_TXTDIS | PERIPH_PTCR_RXTDIS);
#endif

#if USE_DMAC
		spi_tx_dma_disable();
		dmac_channel_suspend(DMAC, CONF_SPI_DMAC_RX_CH);				// suspend the receive channel, don't disable it because the FIFO needs to empty first
#endif

#if USE_XDMAC
		spi_tx_dma_disable();
		xdmac_channel_readwrite_suspend(XDMAC, DmacChanWiFiRx);			// suspend the receive channel
#endif

		spi_disable(ESP_SPI);
		digitalWrite(SamTfrReadyPin, LOW);
		if ((status & SPI_SR_OVRES) != 0)
		{
			++spiRxOverruns;
		}
		if ((status & SPI_SR_UNDES) != 0)
		{
			++spiTxUnderruns;
		}
		transferPending = false;
	}
}

// Start the ESP
void WiFiInterface::StartWiFi()
{
	digitalWrite(EspResetPin, HIGH);
	ConfigurePin(g_APinDescription[APINS_Serial1]);				// connect the pins to UART1
	Serial1.begin(WiFiBaudRate);								// initialise the UART, to receive debug info
	debugMessageChars = 0;
	serialRunning = true;
	debugPrintPending = false;
}

// Reset the ESP8266 and leave held in reset
void WiFiInterface::ResetWiFi()
{
	pinMode(EspResetPin, OUTPUT_LOW);							// assert ESP8266 /RESET
	pinMode(APIN_Serial1_TXD, INPUT_PULLUP);						// just enable pullups on TxD and RxD pins for now to avoid floating pins
	pinMode(APIN_Serial1_RXD, INPUT_PULLUP);
	currentMode = WiFiState::disabled;

	if (serialRunning)
	{
		Serial1.end();
		serialRunning = false;
	}
}

// Reset the ESP8266 to take commands from the UART or from external input. The caller must wait for the reset to complete after calling this.
// ESP8266 boot modes:
// GPIO0	GPIO2	GPIO15
// 0		1		0		Firmware download from UART
// 1		1		0		Normal boot from flash memory
// 0		0		1		SD card boot (not used in on Duet)
void WiFiInterface::ResetWiFiForUpload(bool external)
{
	if (serialRunning)
	{
		Serial1.end();
		serialRunning = false;
	}

	// Make sure the ESP8266 is in the reset state
	pinMode(EspResetPin, OUTPUT_LOW);

#ifdef DUET_NG
	// Take the ESP8266 out of power down
	pinMode(EspEnablePin, OUTPUT_HIGH);
#endif

	// Set up our transfer request pin (GPIO4) as an output and set it low
	pinMode(SamTfrReadyPin, OUTPUT_LOW);

	// Set up our data ready pin (ESP GPIO0) as an output and set it low ready to boot the ESP from UART
	pinMode(EspDataReadyPin, OUTPUT_LOW);

	// GPIO2 also needs to be high to boot up. It's connected to MISO on the SAM, so set the pullup resistor on that pin
	pinMode(APIN_SPI_MISO, INPUT_PULLUP);

	// Set our CS input (ESP GPIO15) low ready for booting the ESP. This also clears the transfer ready latch.
	pinMode(SamCsPin, OUTPUT_LOW);

	// Make sure it has time to reset - no idea how long it needs, but 50ms should be plenty
	delay(50);

	if (external)
	{
		pinMode(APIN_Serial1_TXD, INPUT_PULLUP);					// just enable pullups on TxD and RxD pins
		pinMode(APIN_Serial1_RXD, INPUT_PULLUP);
	}
	else
	{
		ConfigurePin(g_APinDescription[APINS_Serial1]);			// connect the pins to UART1
	}

	// Release the reset on the ESP8266
	digitalWrite(EspResetPin, HIGH);
}

// End
