/****************************************************************************************************

RepRapFirmware - Network: RepRapPro Ormerod with Duet controller

Separated out from Platform.h by dc42 and extended by chrishamm

****************************************************************************************************/

#ifndef NETWORK_H
#define NETWORK_H

#include "ESP8266/Socket.h"
#include "NetworkDefs.h"
#include "RepRapFirmware.h"
#include "MessageType.h"
#include "MessageFormats.h"
#include "GCodes/GCodeResult.h"

class NetworkResponder;
class HttpResponder;
class FtpResponder;
class TelnetResponder;
class WifiFirmwareUploader;

const unsigned int NumHttpResponders = 4;		// the number of concurrent HTTP requests we can process

// Class to allow us to receive some data allowing for some extra bytes being stored by the DMAC
template<class T> class Receiver
{
public:
	void *DmaPointer() { return &object; }
	size_t Size() const { return sizeof(T); }
	T& Value() { return object; }
private:
	T object;
	uint32_t padding;
};

// The main network class that drives the network.
class Network
{
public:
	friend class Socket;

	Network(Platform& p);

	void Init();
	void Activate();
	void Exit();
	void Spin(bool full);
	void Diagnostics(MessageType mtype);
	void Start();
	void Stop();

	void EnableProtocol(int protocol, int port, int secure, StringRef& reply);
	void DisableProtocol(int protocol, StringRef& reply);
	void ReportProtocols(StringRef& reply) const;

	void Enable(int mode, const StringRef& ssid, StringRef& reply);			// enable or disable the network
	GCodeResult HandleWiFiCode(int mcode, GCodeBuffer& gb, StringRef& reply, OutputBuffer*& longReply);
	bool GetNetworkState(StringRef& reply);
	int EnableState() const;

	void SetHostname(const char *name);

	bool FindResponder(Socket *skt, Port localPort);

	const uint8_t *GetIPAddress() const { return ipAddress; }
	void OpenDataPort(Port port);
	void TerminateDataPort();
	void DataPortClosing();

	void HandleHttpGCodeReply(const char *msg);
	void HandleTelnetGCodeReply(const char *msg);
	void HandleHttpGCodeReply(OutputBuffer *buf);
	void HandleTelnetGCodeReply(OutputBuffer *buf);
	uint32_t GetHttpReplySeq();

	// The remaining functions are specific to the WiFi version
	WifiFirmwareUploader& GetWifiUploader() { return *uploader; }

	void StartWiFi();
	void ResetWiFi();
	void ResetWiFiForUpload(bool external);

	const char *GetWiFiServerVersion() const { return wiFiServerVersion; }

	const char* TranslateNetworkState() const;
	static const char* TranslateWiFiState(WiFiState w);

	void SpiInterrupt();
	void EspRequestsTransfer();
	void UpdateSocketStatus(uint16_t connectedSockets, uint16_t otherEndClosedSockets);

private:
	enum class NetworkState
	{
		disabled,					// WiFi module disabled
		starting1,					// starting up
		starting2,					// starting up
		active,						// running, but not necessarily in the requested mode
		changingMode,				// running and in the process of switching between modes
	};

	void InitSockets();
	void TerminateSockets();
	void TerminateSockets(Port port);
	void StopListening(Port port);

	void StartProtocol(Protocol protocol)
	pre(protocol < NumProtocols);

	void ShutdownProtocol(Protocol protocol)
	pre(protocol < NumProtocols);

	void ReportOneProtocol(Protocol protocol, StringRef& reply) const
	pre(protocol < NumProtocols);

	void SetIPAddress(const uint8_t ipAddress[], const uint8_t netmask[], const uint8_t gateway[]);

	void SetupSpi();

	int32_t SendCommand(NetworkCommand cmd, SocketNumber socket, uint8_t flags, const void *dataOut, size_t dataOutLength, void* dataIn, size_t dataInLength);

	template<class T> int32_t SendCommand(NetworkCommand cmd, SocketNumber socket, uint8_t flags, const void *dataOut, size_t dataOutLength, Receiver<T>& recvr)
	{
		return SendCommand(cmd, socket, flags, dataOut, dataOutLength, recvr.DmaPointer(), recvr.Size());
	}

	void SendListenCommand(Port port, Protocol protocol, unsigned int maxConnections);
	void GetNewStatus();

	static const char* TranslateEspResetReason(uint32_t reason);

	Platform& platform;
	NetworkResponder *responders;
	NetworkResponder *nextResponderToPoll;
	FtpResponder *ftpResponder;
	TelnetResponder *telnetResponder;
	uint32_t longWait;
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
	char requestedSsid[SsidLength + 1];
	char actualSsid[SsidLength + 1];

	unsigned int spiTxUnderruns;
	unsigned int spiRxOverruns;
	unsigned int reconnectCount;
	unsigned int transferAlreadyPendingCount;
	unsigned int readyTimeoutCount;
	unsigned int responseTimeoutCount;

	char wiFiServerVersion[16];

	// For processing debug messages from the WiFi module
	bool serialRunning;
	bool debugPrintPending;
	String<100> debugMessageBuffer;
};

#endif
