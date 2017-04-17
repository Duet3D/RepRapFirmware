/****************************************************************************************************

RepRapFirmware - Network: RepRapPro Ormerod with Duet controller

Separated out from Platform.h by dc42 and extended by zpl

****************************************************************************************************/

#ifndef NETWORK_H
#define NETWORK_H

#include "RepRapFirmware.h"
#include "MessageType.h"
#include "MessageFormats.h"

// Return code definitions
const uint32_t rcNumber = 0x0000FFFF;
const uint32_t rcJson = 0x00010000;
const uint32_t rcKeepOpen = 0x00020000;

static const uint8_t DefaultIpAddress[4] = { 192, 168, 1, 10 };
static const uint8_t DefaultNetMask[4] = { 255, 255, 255, 0 };
static const uint8_t DefaultGateway[4] = { 192, 168, 1, 1 };

class WifiFirmwareUploader;

// The main network class that drives the network.
class Network
{
	enum class NetworkState
	{
		disabled,					// WiFi module disabled
		starting,					// starting up
		running,					// running, but not necessarily in the requested mode
		changingMode,				// running and in the process of switching between modes
	};

public:
	const uint8_t *GetIPAddress() const;
	void SetIPAddress(const uint8_t ipAddress[], const uint8_t netmask[], const uint8_t gateway[]);

	Network(Platform* p);
	void Init();
	void Activate();
	void Exit();
	void Spin(bool full);
	void SpiInterrupt();
	void Diagnostics(MessageType mtype);
	void Start();
	void Stop();

	void EnableProtocol(int protocol, int port, bool secure, StringRef& reply);
	void DisableProtocol(int protocol, StringRef& reply);
	void ReportProtocols(StringRef& reply) const;

	bool InLwip() const { return false; }

	void Enable(int mode, StringRef& reply);			// enable or disable the network
	int EnableState() const;

	void SetHostname(const char *name);
	const char *GetRequest(uint32_t& ip, size_t& length, uint32_t& fragment) const;
	void SendReply(uint32_t ip, unsigned int code, OutputBuffer *body);
	void SendReply(uint32_t ip, unsigned int code, const char *text);
	void SendReply(uint32_t ip, unsigned int code, FileStore *file);
	void DiscardMessage();

	WifiFirmwareUploader *GetWifiUploader() { return uploader; }

	static void ResetWiFi();
	static void ResetWiFiForUpload(bool external);

	const char *GetWiFiServerVersion() const { return wiFiServerVersion; }
	int32_t SendCommand(NetworkCommand cmd, uint8_t socket, const void * dataOut, size_t dataOutLength, void* dataIn, size_t dataInLength);

	bool GetNetworkState(StringRef& reply);
	const char* TranslateNetworkState() const;
	static const char* TranslateWiFiState(WiFiState w);

	void EspRequestsTransfer();

private:
	void SetupSpi();
	void ClearIpAddress();

	void StartProtocol(size_t protocol)
	pre(protocol < NumProtocols);

	void ShutdownProtocol(size_t protocol)
	pre(protocol < NumProtocols);

	void ReportOneProtocol(size_t protocol, StringRef& reply) const
	pre(protocol < NumProtocols);

	static const char* TranslateEspResetReason(uint32_t reason);

	Platform * const platform;
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

	uint32_t timer;

    float longWait;
    NetworkState state;
    WiFiState requestedMode;
    WiFiState currentMode;
    bool activated;
    volatile bool espStatusChanged;

    uint8_t ipAddress[4];
	char hostname[16];								// Limit DHCP hostname to 15 characters + terminating 0
	char wiFiServerVersion[16];
};

#endif
