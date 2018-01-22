/*
 * WiFiInterface.h
 *
 *  Created on: 27 Nov 2017
 *      Authors: Christian and David
 */

#ifndef SRC_SAME70_WIFIINTERFACE_H_
#define SRC_SAME70_WIFIINTERFACE_H_

#include "NetworkDefs.h"
#include "RepRapFirmware.h"
#include "MessageType.h"
#include "MessageFormats.h"
#include "NetworkInterface.h"
#include "WiFiSocket.h"
#include "GCodes/GCodeResult.h"

class WifiFirmwareUploader;


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
class WiFiInterface : public NetworkInterface
{
public:
	friend class WiFiSocket;

	WiFiInterface(Platform& p);

	void Init() override;
	void Activate() override;
	void Exit() override;
	void Spin(bool full) override;
	void Diagnostics(MessageType mtype) override;
	void Start() override;
	void Stop() override;

	void EnableProtocol(int protocol, int port, int secure, StringRef& reply) override;
	void DisableProtocol(int protocol, StringRef& reply) override;
	void ReportProtocols(StringRef& reply) const override;

	void Enable(int mode, const StringRef& ssid, StringRef& reply) override;			// enable or disable the network
	bool GetNetworkState(StringRef& reply) override;
	int EnableState() const override;

	GCodeResult HandleWiFiCode(int mcode, GCodeBuffer &gb, StringRef& reply, OutputBuffer*& longReply);

	void SetHostname(const char *hostname);

	const uint8_t *GetIPAddress() const override { return ipAddress; }
	void SetIPAddress(const uint8_t ipAddress[], const uint8_t netmask[], const uint8_t gateway[]) override;

	void OpenDataPort(Port port) override;
	void TerminateDataPort() override;
	void DataPortClosing() override;

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

	Protocol GetProtocolByLocalPort(Port port) const;

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
	uint32_t lastTickMillis;

	WifiFirmwareUploader *uploader;

	WiFiSocket *sockets[NumTcpSockets];
	size_t currentSocket;

	Port portNumbers[NumProtocols];					// port number used for each protocol
	Port ftpDataPort;
	bool closeDataPort;
	bool protocolEnabled[NumProtocols];				// whether each protocol is enabled

	NetworkState state;
	WiFiState requestedMode;
	WiFiState currentMode;
	bool activated;
	volatile bool espStatusChanged;

	uint8_t ipAddress[4];
	uint8_t netmask[4];
	uint8_t gateway[4];
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
