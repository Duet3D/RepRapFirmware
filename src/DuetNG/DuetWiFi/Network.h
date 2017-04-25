/****************************************************************************************************

RepRapFirmware - Network: RepRapPro Ormerod with Duet controller

Separated out from Platform.h by dc42 and extended by zpl

****************************************************************************************************/

#ifndef NETWORK_H
#define NETWORK_H

#include "NetworkDefs.h"
#include "RepRapFirmware.h"
#include "MessageType.h"
#include "Socket.h"
#include "MessageFormats.h"

class NetworkResponder;
class HttpResponder;
class FtpResponder;
class TelnetResponder;
class WifiFirmwareUploader;

const unsigned int NumHttpResponders = 4;		// the number of concurrent HTTP requests we can process

// The main network class that drives the network.
class Network
{
public:
	Network(Platform* p);

	void Init();
	void Activate();
	void Exit();
	void Spin(bool full);
	void Diagnostics(MessageType mtype);
	void Start();
	void Stop();

	void EnableProtocol(int protocol, int port, bool secure, StringRef& reply);
	void DisableProtocol(int protocol, StringRef& reply);
	void ReportProtocols(StringRef& reply) const;

	void Enable(int mode, StringRef& reply);			// enable or disable the network
	bool GetNetworkState(StringRef& reply);
	int EnableState() const;

	void SetHostname(const char *name);

	bool FindResponder(Socket *skt, Protocol protocol);

	void OpenDataPort(Port port);
	void CloseDataPort();

	void HandleHttpGCodeReply(const char *msg);
	void HandleTelnetGCodeReply(const char *msg);
	void HandleHttpGCodeReply(OutputBuffer *buf);
	void HandleTelnetGCodeReply(OutputBuffer *buf);
	uint32_t GetHttpReplySeq();

	// The remaining functions are specific to the WiFi version
	WifiFirmwareUploader *GetWifiUploader() { return uploader; }

	static void ResetWiFi();
	static void ResetWiFiForUpload(bool external);

	const char *GetWiFiServerVersion() const { return wiFiServerVersion; }
	int32_t SendCommand(NetworkCommand cmd, uint8_t socket, const void * dataOut, size_t dataOutLength, void* dataIn, size_t dataInLength);

	const char* TranslateNetworkState() const;
	static const char* TranslateWiFiState(WiFiState w);

	void SpiInterrupt();
	void EspRequestsTransfer();

private:
	enum class NetworkState
	{
		disabled,					// WiFi module disabled
		starting,					// starting up
		active,						// running, but not necessarily in the requested mode
		changingMode,				// running and in the process of switching between modes
	};

	void InitSockets();
	void TerminateSockets();
	void TerminateSockets(Port port);

	void StartProtocol(Protocol protocol)
	pre(protocol < NumProtocols);

	void ShutdownProtocol(Protocol protocol)
	pre(protocol < NumProtocols);

	void ReportOneProtocol(Protocol protocol, StringRef& reply) const
	pre(protocol < NumProtocols);

	void SetIPAddress(const uint8_t ipAddress[], const uint8_t netmask[], const uint8_t gateway[]);

	void SetupSpi();

	void SendListenCommand(Port port, unsigned int maxConnections);

	static const char* TranslateEspResetReason(uint32_t reason);

	Platform * const platform;
	NetworkResponder *responders;
	NetworkResponder *nextResponderToPoll;
	FtpResponder *ftpResponder;
	TelnetResponder *telnetResponder;
	float longWait;
	uint32_t lastTickMillis;

	WifiFirmwareUploader *uploader;

	Socket sockets[NumTcpSockets];
	size_t currentSocket;

	Port portNumbers[NumProtocols];					// port number used for each protocol
	Port ftpDataPort;
	bool protocolEnabled[NumProtocols];				// whether each protocol is enabled

	NetworkState state;
	WiFiState requestedMode;
	WiFiState currentMode;
	bool activated;
	volatile bool espStatusChanged;

	uint8_t ipAddress[4];
	uint8_t netmask[4];
	uint8_t gateway[4];
	char hostname[16];								// Limit DHCP hostname to 15 characters + terminating 0

	uint32_t spiTxUnderruns;
	uint32_t spiRxOverruns;

	char wiFiServerVersion[16];
};

#endif
