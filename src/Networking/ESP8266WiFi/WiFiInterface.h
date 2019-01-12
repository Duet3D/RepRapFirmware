/*
 * WiFiInterface.h
 *
 *  Created on: 27 Nov 2017
 *      Authors: Christian and David
 */

#ifndef SRC_NETWORKING_WIFIINTERFACE_H_
#define SRC_NETWORKING_WIFIINTERFACE_H_

#include "Networking/NetworkDefs.h"
#include "RepRapFirmware.h"
#include "MessageType.h"
#include "Networking/NetworkInterface.h"
#include "GCodes/GCodeResult.h"
#include "MessageFormats.h"

// Forward declarations
class WiFiSocket;
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
	void Start();
	void Stop();

	GCodeResult EnableInterface(int mode, const StringRef& ssid, const StringRef& reply) override;			// enable or disable the network
	GCodeResult EnableProtocol(NetworkProtocol protocol, int port, int secure, const StringRef& reply) override;
	GCodeResult DisableProtocol(NetworkProtocol protocol, const StringRef& reply) override;
	GCodeResult ReportProtocols(const StringRef& reply) const override;

	GCodeResult GetNetworkState(const StringRef& reply) override;
	int EnableState() const override;
	bool InNetworkStack() const override { return false; }
	bool IsWiFiInterface() const override { return true; }

	void UpdateHostname(const char *hostname) override;
	IPAddress GetIPAddress() const override { return ipAddress; }
	void SetIPAddress(IPAddress p_ip, IPAddress p_netmask, IPAddress p_gateway) override;
	void SetMacAddress(const uint8_t mac[]) override;
	const uint8_t *GetMacAddress() const override { return macAddress; }

	void OpenDataPort(Port port) override;
	void TerminateDataPort() override;

	// The remaining functions are specific to the WiFi version
	GCodeResult HandleWiFiCode(int mcode, GCodeBuffer &gb, const StringRef& reply, OutputBuffer*& longReply);
	WifiFirmwareUploader *GetWifiUploader() const { return uploader; }
	void StartWiFi();
	void ResetWiFi();
	void ResetWiFiForUpload(bool external);
	const char *GetWiFiServerVersion() const { return wiFiServerVersion; }
	const char* TranslateNetworkState() const;
	static const char* TranslateWiFiState(WiFiState w);
	void SpiInterrupt();
	void EspRequestsTransfer();
	void UpdateSocketStatus(uint16_t connectedSockets, uint16_t otherEndClosedSockets);

protected:
	DECLARE_OBJECT_MODEL

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

	void StartProtocol(NetworkProtocol protocol)
	pre(protocol < NumProtocols);

	void ShutdownProtocol(NetworkProtocol protocol)
	pre(protocol < NumProtocols);

	void ReportOneProtocol(NetworkProtocol protocol, const StringRef& reply) const
	pre(protocol < NumProtocols);

	NetworkProtocol GetProtocolByLocalPort(Port port) const;

	void SetupSpi();

	int32_t SendCommand(NetworkCommand cmd, SocketNumber socket, uint8_t flags, const void *dataOut, size_t dataOutLength, void* dataIn, size_t dataInLength);

	template<class T> int32_t SendCommand(NetworkCommand cmd, SocketNumber socket, uint8_t flags, const void *dataOut, size_t dataOutLength, Receiver<T>& recvr)
	{
		return SendCommand(cmd, socket, flags, dataOut, dataOutLength, recvr.DmaPointer(), recvr.Size());
	}

	void SendListenCommand(Port port, NetworkProtocol protocol, unsigned int maxConnections);
	void GetNewStatus();
	static const char* TranslateWiFiResponse(int32_t response);

	static const char* TranslateEspResetReason(uint32_t reason);

	Platform& platform;
	uint32_t lastTickMillis;

	WifiFirmwareUploader *uploader;

	WiFiSocket *sockets[NumWiFiTcpSockets];
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

	IPAddress ipAddress;
	IPAddress netmask;
	IPAddress gateway;
	uint8_t macAddress[6];
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
	char debugMessageBuffer[200];
	size_t debugMessageChars;
};

#endif
