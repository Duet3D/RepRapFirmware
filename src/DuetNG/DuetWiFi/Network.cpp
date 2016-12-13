/****************************************************************************************************

 RepRapFirmware network comms to ESP8266-based device

 ****************************************************************************************************/

#include "WifiFirmwareUploader.h"
#include "RepRapFirmware.h"
#include "compiler.h"
#include "Pins.h"
#include "WifiFirmwareUploader.h"
#include "TransactionBuffer.h"
#include "TransactionBufferReader.h"

// Define exactly one of the following as 1, thje other as zero
// The PDC seems to be too slow to work reliably without getting transmit underruns, so we use the DMAC now.
#define USE_PDC		0		// use peripheral DMA controller
#define USE_DMAC	1		// use general DMA controller

#if USE_PDC
#include "pdc.h"
#endif

#if USE_DMAC
#include "dmac.h"
#endif

#include "matrix.h"

// Forward declarations of static functions
static void spi_dma_disable();
static bool spi_dma_check_rx_complete();

static TransactionBuffer inBuffer, outBuffer;
static uint32_t dummyOutBuffer[TransactionBuffer::headerDwords] = {0, 0, 0, 0, 0};

void EspTransferRequestIsr()
{
	reprap.GetNetwork()->EspRequestsTransfer();
}

/*-----------------------------------------------------------------------------------*/
// WiFi interface class

Network::Network(Platform* p) : platform(p), responseCode(0), responseBody(nullptr), responseText(nullptr), responseFile(nullptr),
		spiTxUnderruns(0), spiRxOverruns(0),
		state(disabled), activated(false), connectedToAp(false)

{
	strcpy(hostname, HOSTNAME);
	ClearIpAddress();
	wiFiServerVersion[0] = 0;
}

void Network::Init()
{
	// Make sure the ESP8266 is held in the reset state
	pinMode(EspResetPin, OUTPUT_LOW);
	uploader = new WifiFirmwareUploader(Serial1);
}

void Network::Activate()
{
	activated = true;
	if (state == enabled)
	{
		Start();
	}
}

void Network::Exit()
{
	Stop();
}

void Network::ClearIpAddress()
{
	for (size_t i = 0; i < ARRAY_SIZE(ipAddress); ++i)
	{
		ipAddress[i] = 0;
	}
}

// Start up the ESP. We assume it is not already started.
// ESP8266 boot modes:
// GPIO0	GPIO2	GPIO15
// 0		1		0		Firmware download from UART
// 1		1		0		Normal boot from flash memory
// 0		0		1		SD card boot (not used on Duet)
void Network::Start()
{
	// The ESP8266 is held in a reset state by a pulldown resistor until we enable it.
	// Make sure the ESP8266 is in the reset state
	pinMode(EspResetPin, OUTPUT_LOW);

	// Take the ESP8266 out of power down
	pinMode(EspEnablePin, OUTPUT_HIGH);

	// Set up our transfer request pin (GPIO4) as an output and set it low
	pinMode(SamTfrReadyPin, OUTPUT_LOW);

	// Set up our data ready pin (ESP GPIO0) as an output and set it high ready to boot the ESP from flash
	pinMode(EspTransferRequestPin, OUTPUT_HIGH);

	// GPIO2 also needs to be high to boot. It's connected to MISO on the SAM, so set the pullup resistor on that pin
	pinMode(APIN_SPI_MISO, INPUT_PULLUP);

	// Set our CS input (ESP GPIO15) low ready for booting the ESP. This also clears the transfer ready latch.
	pinMode(SamCsPin, OUTPUT_LOW);

	// Make sure it has time to reset - no idea how long it needs, but 20ms should be plenty
	delay(50);

	// Release the reset on the ESP8266
	digitalWrite(EspResetPin, HIGH);

	// Give it time to sample GPIO0 and GPIO15
	// GPIO0 has to be held high for sufficient time:
	// - 10ms is not enough
	// - 18ms after reset is released, an oscillating signal appears on GPIO0 for 55ms
	// - so 18ms is probably long enough. Use 25ms for safety.
	delay(50);

	// Relinquish control of our CS pin so that the ESP can take it over
	pinMode(SamCsPin, INPUT);

	// Set the data request pin to be an input
	pinMode(EspTransferRequestPin, INPUT_PULLUP);
	attachInterrupt(EspTransferRequestPin, EspTransferRequestIsr, RISING);

	// The ESP takes about 300ms before it starts talking to use, so don't wait for it here, do that in Spin()

	// Clear the transaction buffers
	inBuffer.Clear();
	outBuffer.Clear();

	state = starting;

	(void) SPI->SPI_SR;				// clear any pending interrupt
	NVIC_SetPriority(SPI_IRQn, 10);
	NVIC_EnableIRQ(SPI_IRQn);

	connectedToAp = false;
	spiTxUnderruns = spiRxOverruns = 0;
}

// Stop the ESP
void Network::Stop()
{
	if (state != disabled)
	{
		digitalWrite(SamTfrReadyPin, LOW);			// tell the ESP we can't receive
		for (int i = 0; i < 10 && (state == receivePending || state == sendReceivePending); ++i)
		{
			delay(1);
		}
		digitalWrite(EspResetPin, LOW);	// put the ESP back into reset
		NVIC_DisableIRQ(SPI_IRQn);
		spi_disable(SPI);
		spi_dma_check_rx_complete();
		spi_dma_disable();

		ClearIpAddress();
		state = disabled;
		connectedToAp = false;
	}
}

void Network::Spin()
{
//	static float lastTime = 0.0;

	// Main state machine.
	// Take care with this, because ISRs may cause the following state transitions:
	//  idle -> transferPending
	//  transferPending -> processing
	switch (state)
	{
	case starting:
		// See if the ESP8266 has set CS high yet
		if (digitalRead(SamCsPin))
		{
			// Setup the SPI controller in slave mode and assign the CS pin to it
			platform->Message(HOST_MESSAGE, "WiFi server starting up\n");
			SetupSpi();						// set up the SPI subsystem
			state = idle;
			TryStartTransfer();
		}
		break;

	case transferDone:
//		platform->Message(HOST_MESSAGE, "Transfer done\n");
		if (spi_dma_check_rx_complete())
		{
			if (inBuffer.IsReady())
			{
//				platform->MessageF(DEBUG_MESSAGE, "Rec %u\n", inBuffer.GetFragment());
				if (inBuffer.IsValid())
				{
					inBuffer.AppendNull();
//					platform->Message(HOST_MESSAGE, "Got data\n");
				}
				else
				{
					if (reprap.Debug(moduleNetwork))
					{
						platform->MessageF(DEBUG_MESSAGE, "Bad msg in: ip=%u.%u.%u.%u opcode=%04x frag=%u length=%u\n",
								inBuffer.GetIp() & 255,
								(inBuffer.GetIp() >> 8) & 255,
								(inBuffer.GetIp() >> 16) & 255,
								(inBuffer.GetIp() >> 24) & 255,
								inBuffer.GetOpcode(),
								inBuffer.GetFragment(),
								inBuffer.GetLength()
								);
					}
					inBuffer.Clear();
				}
			}
			else
			{
//				platform->MessageF(DEBUG_MESSAGE, "Rec null %u %u %u %u %u\n",
//						inBuffer.GetOpcode(), inBuffer.GetIp(), inBuffer.GetSeq(), inBuffer.GetFragment(), inBuffer.GetLength());
			}
			state = processing;
		}
		else
		{
			break;
		}
		// no break
	case processing:
		// Deal with incoming data, if any
		if (inBuffer.IsReady())
		{
			ProcessIncomingData(inBuffer);			// this may or may not clear inBuffer
		}
		if (!inBuffer.IsEmpty())
		{
			break;									// still processing
		}
		responseFragment = 0;
		state = sending;
		// no break
	case sending:
		if (outBuffer.IsEmpty())
		{
			// See if we have more of the current response to send
			if (responseBody != nullptr)
			{
				// We have a reply contained in an OutputBuffer
				outBuffer.SetMessage(trTypeResponse | ttRr, responseIp, responseFragment);
				if (responseFragment == 0)
				{
					// Put the return code and content length at the start of the message
					outBuffer.AppendU32(responseCode);
					outBuffer.AppendU32(responseBody->Length());
				}

				do
				{
					const size_t len = responseBody->BytesLeft();
					const size_t bytesWritten = outBuffer.AppendData(responseBody->Read(0), len);
					if (bytesWritten < len)
					{
						// Output buffer is full so will will need to send another fragment
						(void)responseBody->Read(bytesWritten);		// say how much data we have taken from the buffer
						break;
					}
					responseBody = OutputBuffer::Release(responseBody);
				}
				while (responseBody != nullptr);

				if (responseBody == nullptr)
				{
					outBuffer.SetLastFragment();
					DebugPrintResponse();
					state = idle;
				}
				else
				{
					++responseFragment;
					DebugPrintResponse();
				}
			}
			else if (responseText != nullptr)
			{
				// We have a simple text reply to send
				outBuffer.SetMessage(trTypeResponse | ttRr, responseIp, responseFragment);
				if (responseFragment == 0)
				{
					// Put the return code and content length at the start of the message
					outBuffer.AppendU32(responseCode);
					outBuffer.AppendU32(strlen(responseText));
				}

				const size_t len = strlen(responseText);
				const size_t lenSent = outBuffer.AppendData(responseText, len);
				if (lenSent < len)
				{
					responseText += lenSent;
					++responseFragment;
					DebugPrintResponse();
				}
				else
				{
					responseText = nullptr;
					outBuffer.SetLastFragment();
					DebugPrintResponse();
					state = idle;
				}
			}
			else if (responseFile != nullptr)
			{
				// We have a file reply to send
				outBuffer.SetMessage(trTypeResponse | ttRr, responseIp, responseFragment);
				if (responseFragment == 0)
				{
					// Put the return code and content length at the start of the message
					outBuffer.AppendU32(responseCode);
					outBuffer.AppendU32(responseFileBytes);
				}

				size_t spaceLeft;
				char *p = outBuffer.GetBuffer(spaceLeft);
				uint32_t bytesToRead = min<uint32_t>(spaceLeft, responseFileBytes);
				int bytesRead = responseFile->Read(p, bytesToRead);
				if (bytesRead >= 0 && (uint32_t)bytesRead <= bytesToRead)
				{
					outBuffer.DataAppended((uint32_t)bytesRead);
				}

				bool finished;
				if (bytesRead == (int)bytesToRead)
				{
					responseFileBytes -= (uint32_t)bytesRead;
					finished = (responseFileBytes == 0);
				}
				else
				{
					// We have a file read error, however it's too late to signal it unless this is the first fragment
					finished = true;
				}

				if (finished)
				{
					responseFile->Close();
					responseFile = nullptr;
					outBuffer.SetLastFragment();
					DebugPrintResponse();
					state = idle;
				}
				else
				{
					++responseFragment;
					DebugPrintResponse();
				}
			}
			else
			{
				state = idle;
			}
			TryStartTransfer();
		}
		break;

	case idle:
		TryStartTransfer();
		break;

	case disabled:
		uploader->Spin();
		break;

	default:
		break;
	}

	platform->ClassReport(longWait);
}

void Network::DebugPrintResponse()
{
	if (reprap.Debug(moduleNetwork))
	{
		char buffer[200];
		StringRef reply(buffer, ARRAY_SIZE(buffer));
		outBuffer.AppendNull();
		size_t len;
		const char* s = (const char*)outBuffer.GetData(len);
		uint32_t frag = outBuffer.GetFragment() & ~lastFragment;

		reply.printf("Resp %u: ", outBuffer.GetFragment());
		if (frag == 0 && len >= 8)
		{
			// First fragment, so there is a 2-word header
			reply.catf("%08x %08x ", *(const uint32_t*)s, *(const uint32_t*)(s + 4));
			s += 8;
			len -= 8;
		}
		if (len < 38)
		{
			reply.catf("%s\n", s);
		}
		else
		{
			reply.catf("%c%c%c%c...s\n", s[0], s[1], s[2], s[3], s + len - 30);
		}

		platform->Message(HOST_MESSAGE, reply.Pointer());
	}
}

// Translate a ESP8266 reset reason to text
const char* Network::TranslateEspResetReason(uint32_t reason)
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

void Network::ProcessIncomingData(TransactionBuffer &buf)
{
	uint32_t opcode = inBuffer.GetOpcode();
	switch(opcode & 0xFF0000FF)
	{
	case trTypeInfo | ttNetworkInfo:
		// Network info received from host
		// The first 4 bytes specify the format of the remaining data, as follows:
		// Format 1:
		//		4 bytes of IP address
		//		4 bytes of free heap
		//		4 bytes of reset reason
		//		4 bytes of flash chip size
		//		2 bytes of operating state (1 = client, 2 = access point)
		//		2 bytes of ESP8266 Vcc according to its ADC
		//		16 chars of WiFi firmware version
		//		64 chars of host name, null terminated
		//		32 chars of ssid (either ssid we are connected to or our own AP name), null terminated
		{
			TransactionBufferReader reader(buf);
			uint32_t infoVersion = reader.GetPrimitive<uint32_t>();
			if (infoVersion == 1 || infoVersion == 2)
			{
				reader.GetArray(ipAddress, 4);
				const uint32_t freeHeap = reader.GetPrimitive<uint32_t>();
				const char *resetReason = TranslateEspResetReason(reader.GetPrimitive<uint32_t>());
				const uint32_t flashSize = reader.GetPrimitive<uint32_t>();
				int32_t rssi;
				if (infoVersion == 2)
				{
					rssi = reader.GetPrimitive<int32_t>();
				}
				const uint16_t wifiState = reader.GetPrimitive<uint16_t>();
				const uint16_t espVcc = reader.GetPrimitive<uint16_t>();
				const char *firmwareVersion = reader.GetString(16);
				strncpy(wiFiServerVersion, firmwareVersion, ARRAY_SIZE(wiFiServerVersion));
				const char *hostName = reader.GetString(64);
				const char *ssid = reader.GetString(32);
				if (reader.IsOk())
				{
					platform->MessageF(HOST_MESSAGE,
										"DuetWiFiServer version %s\n"
										"Flash size %u, free RAM %u bytes, WiFi Vcc %.2fV, host name: %s, reset reason: %s\n",
										firmwareVersion, flashSize, freeHeap, (float)espVcc/1024, hostName, resetReason);
					if (wifiState == 1)
					{
						if (infoVersion == 2)
						{
							platform->MessageF(HOST_MESSAGE, "WiFi server connected to access point %s, IP=%u.%u.%u.%u, signal strength=%ddBm\n",
												ssid, ipAddress[0], ipAddress[1], ipAddress[2], ipAddress[3], rssi);
						}
						else
						{
							platform->MessageF(HOST_MESSAGE, "WiFi server connected to access point %s, IP=%u.%u.%u.%u\n",
												ssid, ipAddress[0], ipAddress[1], ipAddress[2], ipAddress[3]);
						}
					}
					else if (wifiState == 2)
					{
						platform->MessageF(HOST_MESSAGE, "WiFi is running as an access point with name %s\n", ssid);
					}
					else
					{
						platform->MessageF(HOST_MESSAGE, "Unknown WiFi state %d\n", wifiState);
					}
					connectedToAp = true;
				}
			}
		}
		inBuffer.Clear();
		break;

	case trTypeInfo | ttNetworkInfoOld:
		// Old style network info received from host. Remove this code when everyone is using updated DuetWiFiServer firmware.
		// Data is 4 bytes of IP address, 4 bytes of free heap, 4 bytes of reset reason, 64 chars of host name, and 32 bytes of ssid
		{
			TransactionBufferReader reader(buf);
			reader.GetArray(ipAddress, 4);
			uint32_t freeHeap = reader.GetPrimitive<uint32_t>();
			const char *resetReason = TranslateEspResetReason(reader.GetPrimitive<uint32_t>());
			const char *hostName = reader.GetString(64);
			const char *ssid = reader.GetString(32);
			if (reader.IsOk())
			{
				platform->MessageF(HOST_MESSAGE, "WiFi server connected to access point %s, IP=%u.%u.%u.%u\n",
									ssid, ipAddress[0], ipAddress[1], ipAddress[2], ipAddress[3]);
				platform->MessageF(HOST_MESSAGE, "WiFi host name %s, free memory %u bytes, reset reason: %s\n", hostName, freeHeap, resetReason);
				connectedToAp = true;
			}
		}
		inBuffer.Clear();
		break;

	case trTypeRequest | ttRr:
#if 0
		{
			size_t length;
			const char* data = (const char*)inBuffer.GetData(length);
			if (length > 30) { data += (length - 30); }
			platform->MessageF(DEBUG_MESSAGE, "IP %u.%u.%u.%u Frag %u %s\n",
								inBuffer.GetIp() & 255,
								(inBuffer.GetIp() >> 8) & 255,
								(inBuffer.GetIp() >> 16) & 255,
								(inBuffer.GetIp() >> 24) & 255,
								inBuffer.GetFragment(),
								data);
		}
#else
		// Do nothing - the webserver module will pick it up
#endif
		break;

	default:
		{
			size_t length;
			const char* data = (const char*)inBuffer.GetData(length);
			platform->MessageF(DEBUG_MESSAGE, "Received opcode %08x length %u data %s\n", opcode, length, data);
		}
		inBuffer.Clear();
		break;
	}
}

// Called by the webserver module to get an incoming request
const char *Network::GetRequest(uint32_t& ip, size_t& length, uint32_t& fragment) const
{
	if (state == processing)
	{
		uint32_t opcode = inBuffer.GetOpcode();
		if ((opcode & 0xFF0000FF) == (trTypeRequest | ttRr))
		{
			const void *data = inBuffer.GetData(length);
			if (length > 0)
			{
				ip = inBuffer.GetIp();
				fragment = inBuffer.GetFragment();
				if ((fragment & 0x7FFFFFFF) == 0)
				{
					length += 1;					// allow client to read the null at the end too
				}
//				platform->MessageF(HOST_MESSAGE, "Req: %s\n", (const char*)data);
				return (const char*)data;
			}
			else
			{
				platform->Message(DEBUG_MESSAGE, "Bad request\n");
				inBuffer.Clear();					// bad request
			}
		}
	}
	return nullptr;
}

// Send a reply from an OutputBuffer chain. Release the chain when we have finished with it.
void Network::SendReply(uint32_t ip, unsigned int code, OutputBuffer *body)
{
	if (responseBody != nullptr || responseText != nullptr || responseFile != nullptr)
	{
		platform->Message(HOST_MESSAGE, "response already being sent in SendReply(ob*)\n");
	}
	else
	{
		responseIp = ip;
		responseCode = code;
		responseBody = body;

#if 0
		//debug
		{
			char buf[101];
			size_t len = min<size_t>(ARRAY_UPB(buf), responseBody->DataLength());
			strncpy(buf, responseBody->Data(), len);
			buf[len] = 0;
			platform->MessageF(HOST_MESSAGE, "%s %u %s\n", (responseCode & rcJson) ? "JSON reply" : "Reply", responseCode & rcNumber, buf);
		}
#endif
		// Say we have taken the request
		if (state == processing)
		{
			uint32_t opcode = inBuffer.GetOpcode();
			if ((opcode & 0xFF0000FF) == (trTypeRequest | ttRr))
			{
				inBuffer.Clear();
			}
		}
	}
}

// Send a reply from a null-terminated string
void Network::SendReply(uint32_t ip, unsigned int code, const char *text)
{
	if (responseBody != nullptr || responseText != nullptr || responseFile != nullptr)
	{
		platform->Message(HOST_MESSAGE, "response already being sent in SendReply(cc*)\n");
	}
	else
	{
		responseIp = ip;
		responseCode = code;
		responseText = text;

		//debug
		//platform->MessageF(HOST_MESSAGE, "%s %u %s\n", (responseCode & rcJson) ? "JSON reply" : "Reply", responseCode & rcNumber, text);

		// Say we have taken the request
		if (state == processing)
		{
			uint32_t opcode = inBuffer.GetOpcode();
			if ((opcode & 0xFF0000FF) == (trTypeRequest | ttRr))
			{
				inBuffer.Clear();
			}
		}
	}
}

// Send a file as the reply. Close the file at the end.
void Network::SendReply(uint32_t ip, unsigned int code, FileStore *file)
{
	if (responseBody != nullptr || responseText != nullptr || responseFile != nullptr)
	{
		platform->Message(HOST_MESSAGE, "response already being sent in SendReply(fs*)\n");
		file->Close();
	}
	else
	{
		responseIp = ip;
		responseCode = code;
		responseFile = file;
		responseFileBytes = file->Length();

		// Say we have taken the request
		if (state == processing)
		{
			uint32_t opcode = inBuffer.GetOpcode();
			if ((opcode & 0xFF0000FF) == (trTypeRequest | ttRr))
			{
				inBuffer.Clear();
			}
		}
	}
}

// This is called when we have read a message fragment and there is no reply to send
void Network::DiscardMessage()
{
	if (responseBody != nullptr || responseText != nullptr || responseFile != nullptr)
	{
		platform->Message(HOST_MESSAGE, "response being sent in DiscardMessage\n");
	}

	//debug
	//platform->MessageF(HOST_MESSAGE, "%s %u %s\n", (responseCode & rcJson) ? "JSON reply" : "Reply", responseCode & rcNumber, text);

	// Say we have taken the request
	if (state == processing)
	{
		uint32_t opcode = inBuffer.GetOpcode();
		if ((opcode & 0xFF0000FF) == (trTypeRequest | ttRr))
		{
			inBuffer.Clear();
		}

		// For faster response, change the state to idle so we can accept another packet immediately
		state = idle;
		TryStartTransfer();
	}
}

void Network::Diagnostics(MessageType mtype)
{
	platform->Message(mtype, "=== Network ===\n");
	const char* text = (state == starting) ? "starting"
						: (state == disabled) ? "disabled"
							: (state == enabled) ? "enabled but not running"
								: "running";
	platform->MessageF(mtype, "WiFiServer is %s\n", text);
	platform->MessageF(mtype, "SPI underruns %u, overruns %u\n", spiTxUnderruns, spiRxOverruns);
}

void Network::Enable()
{
	if (state == disabled)
	{
		state = enabled;
		if (activated)
		{
			Start();
		}
	}
}

void Network::Disable()
{
	if (activated && state != disabled)
	{
		Stop();
		platform->Message(GENERIC_MESSAGE, "WiFi server stopped\n");
	}
}

bool Network::IsEnabled() const
{
	return state != disabled;
}

const uint8_t *Network::IPAddress() const
{
	return ipAddress;
}

uint16_t Network::GetHttpPort() const
{
	return DEFAULT_HTTP_PORT;
}

void Network::SetHttpPort(uint16_t port)
{
	// Not supported
}

// Set the DHCP hostname. Removes all whitespaces and converts the name to lower-case.
void Network::SetHostname(const char *name)
{
	size_t i = 0;
	while (*name && i < ARRAY_UPB(hostname))
	{
		char c = *name++;
		if (c >= 'A' && c <= 'Z')
		{
			c += 'a' - 'A';
		}

		if ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || (c == '-') || (c == '_'))
		{
			hostname[i++] = c;
		}
	}

	if (i)
	{
		hostname[i] = 0;
	}
	else
	{
		strcpy(hostname, HOSTNAME);
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

static inline void spi_rx_dma_enable()
{
#if USE_PDC
	pdc_enable_transfer(spi_pdc, PERIPH_PTCR_RXTEN);
#endif

#if USE_DMAC
	dmac_channel_enable(DMAC, CONF_SPI_DMAC_RX_CH);
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
}

static inline void spi_rx_dma_disable()
{
#if USE_PDC
	pdc_disable_transfer(spi_pdc, PERIPH_PTCR_RXTDIS);
#endif

#if USE_DMAC
	dmac_channel_disable(DMAC, CONF_SPI_DMAC_RX_CH);
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
}

static void spi_dma_disable()
{
	spi_tx_dma_disable();
	spi_rx_dma_disable();
}

static bool spi_dma_check_rx_complete()
{
#if USE_PDC
	return true;
#endif

#if USE_DMAC
	uint32_t status = DMAC->DMAC_CHSR;
	if (   ((status & (DMAC_CHSR_ENA0 << CONF_SPI_DMAC_RX_CH)) == 0)	// controller is not enabled, perhaps because it finished a full buffer transfer
		|| ((status & (DMAC_CHSR_EMPT0 << CONF_SPI_DMAC_RX_CH)) != 0)	// controller is enabled, probably suspended, and the FIFO is empty
	   )
	{
		// Disable the channel.
		// We also need to set the resume bit, otherwise it remains suspended when we re-enable it.
		DMAC->DMAC_CHDR = (DMAC_CHDR_DIS0 << CONF_SPI_DMAC_RX_CH) | (DMAC_CHDR_RES0 << CONF_SPI_DMAC_RX_CH);
		return true;
	}
	return false;
#endif
}

static void spi_tx_dma_setup(const TransactionBuffer *buf, uint32_t maxTransmitLength)
{
#if USE_PDC
	pdc_packet_t pdc_spi_packet;
	pdc_spi_packet.ul_addr = reinterpret_cast<uint32_t>(buf);
	pdc_spi_packet.ul_size = buf->PacketLength() * 4;			// need length in bytes
	pdc_tx_init(spi_pdc, &pdc_spi_packet, NULL);
#endif

#if USE_DMAC
	DMAC->DMAC_EBCISR;		// clear any pending interrupts

	dmac_channel_set_source_addr(DMAC, CONF_SPI_DMAC_TX_CH, reinterpret_cast<uint32_t>(buf));
	dmac_channel_set_destination_addr(DMAC, CONF_SPI_DMAC_TX_CH, reinterpret_cast<uint32_t>(& SPI->SPI_TDR));
	dmac_channel_set_descriptor_addr(DMAC, CONF_SPI_DMAC_TX_CH, 0);
	dmac_channel_set_ctrlA(DMAC, CONF_SPI_DMAC_TX_CH, maxTransmitLength | DMAC_CTRLA_SRC_WIDTH_WORD | DMAC_CTRLA_DST_WIDTH_BYTE);
	dmac_channel_set_ctrlB(DMAC, CONF_SPI_DMAC_TX_CH,
		DMAC_CTRLB_SRC_DSCR | DMAC_CTRLB_DST_DSCR | DMAC_CTRLB_FC_MEM2PER_DMA_FC | DMAC_CTRLB_SRC_INCR_INCREMENTING | DMAC_CTRLB_DST_INCR_FIXED);
#endif
}

static void spi_rx_dma_setup(const TransactionBuffer *buf)
{
#if USE_PDC
	pdc_packet_t pdc_spi_packet;
	pdc_spi_packet.ul_addr = reinterpret_cast<uint32_t>(buf);
	pdc_spi_packet.ul_size = TransactionBuffer::MaxReceiveBytes;
	pdc_rx_init(spi_pdc, &pdc_spi_packet, NULL);
#endif

#if USE_DMAC
	DMAC->DMAC_EBCISR;		// clear any pending interrupts

	dmac_channel_set_source_addr(DMAC, CONF_SPI_DMAC_RX_CH, reinterpret_cast<uint32_t>(& SPI->SPI_RDR));
	dmac_channel_set_destination_addr(DMAC, CONF_SPI_DMAC_RX_CH, reinterpret_cast<uint32_t>(buf));
	dmac_channel_set_descriptor_addr(DMAC, CONF_SPI_DMAC_RX_CH, 0);
	dmac_channel_set_ctrlA(DMAC, CONF_SPI_DMAC_RX_CH, TransactionBuffer::MaxTransferBytes | DMAC_CTRLA_SRC_WIDTH_BYTE | DMAC_CTRLA_DST_WIDTH_WORD);
	dmac_channel_set_ctrlB(DMAC, CONF_SPI_DMAC_RX_CH,
		DMAC_CTRLB_SRC_DSCR | DMAC_CTRLB_DST_DSCR | DMAC_CTRLB_FC_PER2MEM_DMA_FC | DMAC_CTRLB_SRC_INCR_FIXED | DMAC_CTRLB_DST_INCR_INCREMENTING);
#endif
}

/**
 * \brief Set SPI slave transfer.
 */
static void spi_slave_dma_setup(bool dataToSend, bool allowReceive)
{
#if USE_PDC
	pdc_disable_transfer(spi_pdc, PERIPH_PTCR_TXTDIS | PERIPH_PTCR_RXTDIS);

	TransactionBuffer *outBufPointer = (dataToSend) ? &outBuffer : reinterpret_cast<TransactionBuffer*>(&dummyOutBuffer);
	spi_tx_dma_setup(outBufPointer);
	if (allowReceive)
	{
		outBufPointer->SetDataTaken();
		spi_rx_dma_setup(&inBuffer);
		pdc_enable_transfer(spi_pdc, PERIPH_PTCR_TXTEN | PERIPH_PTCR_RXTEN);
	}
	else
	{
		outBufPointer->ClearDataTaken();
		pdc_enable_transfer(spi_pdc, PERIPH_PTCR_TXTEN);
	}
#endif

#if USE_DMAC
	spi_dma_disable();

	TransactionBuffer *outBufPointer = (dataToSend) ? &outBuffer : reinterpret_cast<TransactionBuffer*>(&dummyOutBuffer);
	if (allowReceive)
	{
		spi_rx_dma_setup(&inBuffer);
		spi_rx_dma_enable();
		outBufPointer->SetDataTaken();
	}
	else
	{
		outBufPointer->ClearDataTaken();
	}

	spi_tx_dma_setup(outBufPointer, (dataToSend) ? TransactionBuffer::MaxTransferBytes : 4 * TransactionBuffer::headerDwords);
	spi_tx_dma_enable();
#endif
}

// Set up the SPI system
void Network::SetupSpi()
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

	pmc_enable_periph_clk(ID_SPI);
	spi_dma_disable();
	spi_reset(SPI);				// this clears the transmit and receive registers and puts the SPI into slave mode

	// Set up the SPI pins
	ConfigurePin(g_APinDescription[APIN_SPI_SCK]);
	ConfigurePin(g_APinDescription[APIN_SPI_MOSI]);
	ConfigurePin(g_APinDescription[APIN_SPI_MISO]);
	ConfigurePin(g_APinDescription[APIN_SPI_SS0]);

#if USE_DMAC
	// Configure DMA RX channel
	dmac_channel_set_configuration(DMAC, CONF_SPI_DMAC_RX_CH,
			DMAC_CFG_SRC_PER(DMA_HW_ID_SPI_RX) | DMAC_CFG_SRC_H2SEL | DMAC_CFG_SOD | DMAC_CFG_FIFOCFG_ASAP_CFG);

	// Configure DMA TX channel
	dmac_channel_set_configuration(DMAC, CONF_SPI_DMAC_TX_CH,
			DMAC_CFG_DST_PER(DMA_HW_ID_SPI_TX) | DMAC_CFG_DST_H2SEL | DMAC_CFG_SOD | DMAC_CFG_FIFOCFG_ASAP_CFG);
#endif
}

// Start a transfer if necessary. Not called from an ISR.
void Network::TryStartTransfer()
{
	cpu_irq_disable();
	if (state == idle)
	{
		if (outBuffer.IsReady())
		{
			PrepareForTransfer(true, true);
		}
		else if (digitalRead(EspTransferRequestPin))
		{
			PrepareForTransfer(false, true);
		}
	}
	else if (state == sending && outBuffer.IsReady())
	{
		PrepareForTransfer(true, false);
	}
	cpu_irq_enable();
}

// This is called from the ISR when the ESP requests to send data
void Network::EspRequestsTransfer()
{
	irqflags_t flags = cpu_irq_save();
	if (state == idle)
	{
		PrepareForTransfer(false, true);
	}
	cpu_irq_restore(flags);
}

// Set up the DMA controller and assert transfer ready. Must be called with interrupts disabled.
void Network::PrepareForTransfer(bool dataToSend, bool allowReceive)
pre(state == idle || state == sending)
{
	if (allowReceive)
	{
		state = (dataToSend) ? sendReceivePending : receivePending;
	}

	// DMA may have transferred an extra word to the SPI transmit data register. We need to clear this.
	// The only way I can find to do this is to issue a software reset to the SPI system.
	// Fortunately, this leaves the SPI system in slave mode.
	spi_reset(SPI);
	spi_set_bits_per_transfer(SPI, 0, SPI_CSR_BITS_8_BIT);

	// Set up the DMA controller
	spi_slave_dma_setup(dataToSend, allowReceive);
	spi_enable(SPI);

	// Enable the end-of transfer interrupt
	(void)SPI->SPI_SR;						// clear any pending interrupt
	SPI->SPI_IER = SPI_IER_NSSR;			// enable the NSS rising interrupt

	// Tell the ESP that we are ready to accept data
	digitalWrite(SamTfrReadyPin, HIGH);
}

// SPI interrupt handler, called when NSS goes high
void SPI_Handler()
{
	reprap.GetNetwork()->SpiInterrupt();
}

void Network::SpiInterrupt()
{
	uint32_t status = SPI->SPI_SR;			// read status and clear interrupt
	SPI->SPI_IDR = SPI_IER_NSSR;			// disable the interrupt
	if ((status & SPI_SR_NSSR) != 0)
	{
		if (state == sendReceivePending || state == receivePending)
		{
#if USE_PDC
			pdc_disable_transfer(spi_pdc, PERIPH_PTCR_TXTDIS | PERIPH_PTCR_RXTDIS);
#endif

#if USE_DMAC
			spi_tx_dma_disable();
			dmac_channel_suspend(DMAC, CONF_SPI_DMAC_RX_CH);	// suspend the receive channel, don't disable it because the FIFO needs to empty first
#endif
			spi_disable(SPI);
			digitalWrite(SamTfrReadyPin, LOW);
			if (state == sendReceivePending)
			{
				outBuffer.Clear();								// don't send the data again
			}
			if ((status & SPI_SR_OVRES) != 0)
			{
				++spiRxOverruns;
			}
			if (state == sendReceivePending && (status & SPI_SR_UNDES) != 0)
			{
				++spiTxUnderruns;
			}
			state = transferDone;
		}
		else if (state == sending)
		{
			spi_tx_dma_disable();
			spi_disable(SPI);
			digitalWrite(SamTfrReadyPin, LOW);
			outBuffer.Clear();									// don't send the data again
			if ((status & SPI_SR_UNDES) != 0)
			{
				++spiTxUnderruns;
			}
		}
	}
}

// Reset the ESP8266 and leave held in reset
void Network::ResetWiFi()
{
	pinMode(EspResetPin, OUTPUT_LOW);
}

// Reset the ESP8266 to take commands from the UART. The caller must wait for the reset to complete after calling this.
// ESP8266 boot modes:
// GPIO0	GPIO2	GPIO15
// 0		1		0		Firmware download from UART
// 1		1		0		Normal boot from flash memory
// 0		0		1		SD card boot (not used in on Duet)
void Network::ResetWiFiForUpload()
{
	// Make sure the ESP8266 is in the reset state
	pinMode(EspResetPin, OUTPUT_LOW);

	// Take the ESP8266 out of power down
	pinMode(EspEnablePin, OUTPUT_HIGH);

	// Set up our transfer request pin (GPIO4) as an output and set it low
	pinMode(SamTfrReadyPin, OUTPUT_LOW);

	// Set up our data ready pin (ESP GPIO0) as an output and set it low ready to boot the ESP from UART
	pinMode(EspTransferRequestPin, OUTPUT_LOW);

	// GPIO2 also needs to be high to boot up. It's connected to MISO on the SAM, so set the pullup resistor on that pin
	pinMode(APIN_SPI_MISO, INPUT_PULLUP);

	// Set our CS input (ESP GPIO15) low ready for booting the ESP. This also clears the transfer ready latch.
	pinMode(SamCsPin, OUTPUT_LOW);

	// Make sure it has time to reset - no idea how long it needs, but 50ms should be plenty
	delay(50);

	// Release the reset on the ESP8266
	digitalWrite(EspResetPin, HIGH);
}

// Reset the ESP8266 to take commands from an external input. The caller must wait for the reset to complete after calling this.
void Network::ResetWiFiForExternalUpload()
{
	ResetWiFiForUpload();

	// Set our TxD pin low to make things easier for the FTDI chip to drive the ESP RxD input
	pinMode(APIN_UART1_TXD, OUTPUT_LOW);
}

// End
