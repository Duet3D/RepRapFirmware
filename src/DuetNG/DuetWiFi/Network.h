/****************************************************************************************************

RepRapFirmware - Network: RepRapPro Ormerod with Duet controller

Separated out from Platform.h by dc42 and extended by zpl

****************************************************************************************************/

#ifndef NETWORK_H
#define NETWORK_H

#include <cctype>
#include <cstring>
#include <cstdlib>
#include <climits>

#include "MessageType.h"

// Return code definitions
const uint32_t rcNumber = 0x0000FFFF;
const uint32_t rcJson = 0x00010000;
const uint32_t rcKeepOpen = 0x00020000;

static const uint8_t IP_ADDRESS[4] = { 192, 168, 1, 10 };				// Need some sort of default...
static const uint8_t NET_MASK[4] = { 255, 255, 255, 0 };
static const uint8_t GATE_WAY[4] = { 192, 168, 1, 1 };
static const uint16_t DEFAULT_HTTP_PORT = 80;

class TransactionBuffer;
class WifiFirmwareUploader;

// The main network class that drives the network.
class Network
{
	enum TransferState
	{
		disabled,					// WiFi not active
		enabled,					// WiFi enabled but not started yet
		starting,					// starting up (waiting for WiFi to initialise)
		idle,						// nothing happening
		receivePending,				// we have asserted TransferReady and await completion of a receive-only transaction
		sendReceivePending,			// we have asserted TransferReady and await completion of a transmit/receive
		transferDone,				// transfer completed but receive DMA fifo may not have been flushed yet
		processing,					// a transaction has been completed but we haven't released the input buffer yet
		sending						// a transaction has been completed and we are sending the response
	};

public:
	const uint8_t *GetIPAddress() const;
	void SetIPAddress(const uint8_t ipAddress[], const uint8_t netmask[], const uint8_t gateway[]);

	Network(Platform* p);
	void Init();
	void Activate();
	void Exit();
	void Spin();
	void SpiInterrupt();
	void Diagnostics(MessageType mtype);
	void Start();
	void Stop();

	bool InLwip() const { return false; }

	void Enable();
	void Disable();
	bool IsEnabled() const;

	void SetHttpPort(uint16_t port);
	uint16_t GetHttpPort() const;

	void SetHostname(const char *name);
	void EspRequestsTransfer();

	const char *GetRequest(uint32_t& ip, size_t& length, uint32_t& fragment) const;
	void SendReply(uint32_t ip, unsigned int code, OutputBuffer *body);
	void SendReply(uint32_t ip, unsigned int code, const char *text);
	void SendReply(uint32_t ip, unsigned int code, FileStore *file);
	void DiscardMessage();

	WifiFirmwareUploader *GetWifiUploader() { return uploader; }

	static void ResetWiFi();
	static void ResetWiFiForUpload();
	static void ResetWiFiForExternalUpload();

	const char *GetWiFiServerVersion() const { return wiFiServerVersion; }

private:
	void SetupSpi();
	void PrepareForTransfer(bool dataToSend, bool allowReceive);
	void ProcessIncomingData(TransactionBuffer &buf);
	void ClearIpAddress();
	void TryStartTransfer();
	void DebugPrintResponse();

	static const char* TranslateEspResetReason(uint32_t reason);

	Platform *platform;
	WifiFirmwareUploader *uploader;

	uint32_t responseIp;
	uint32_t responseCode;
	uint32_t responseFragment;
	OutputBuffer *responseBody;
	const char* responseText;
	FileStore *responseFile;
	uint32_t responseFileBytes;

	uint32_t spiTxUnderruns;
	uint32_t spiRxOverruns;

    float longWait;
    TransferState state;
    bool activated;
    bool connectedToAp;
    uint8_t ipAddress[4];
	char hostname[16];								// Limit DHCP hostname to 15 characters + terminating 0
	char wiFiServerVersion[16];
};

#endif
