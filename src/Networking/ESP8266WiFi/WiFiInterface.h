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
	void *DmaPointer() noexcept { return &object; }
	size_t Size() const noexcept { return sizeof(T); }
	T& Value() noexcept { return object; }
private:
	T object;
	uint32_t padding;
};

// The main network class that drives the network.
class WiFiInterface : public NetworkInterface
{
public:
	friend class WiFiSocket;

	WiFiInterface(Platform& p) noexcept;

	void Init() noexcept override;
	void Activate() noexcept override;
	void Exit() noexcept override;
	void Spin() noexcept override;
	void Diagnostics(MessageType mtype) noexcept override;
	void Start() noexcept;
	void Stop() noexcept;

	GCodeResult EnableInterface(int mode, const StringRef& ssid, const StringRef& reply) noexcept override;			// enable or disable the network
	GCodeResult EnableProtocol(NetworkProtocol protocol, int port, int secure, const StringRef& reply) noexcept override;
	GCodeResult DisableProtocol(NetworkProtocol protocol, const StringRef& reply) noexcept override;
	GCodeResult ReportProtocols(const StringRef& reply) const noexcept override;

	GCodeResult GetNetworkState(const StringRef& reply) noexcept override;
	int EnableState() const noexcept override;
	bool IsWiFiInterface() const noexcept override { return true; }

	void UpdateHostname(const char *hostname) noexcept override;
	IPAddress GetIPAddress() const noexcept override { return ipAddress; }
	void SetIPAddress(IPAddress p_ip, IPAddress p_netmask, IPAddress p_gateway) noexcept override;
	GCodeResult SetMacAddress(const MacAddress& mac, const StringRef& reply) noexcept override;
	const MacAddress& GetMacAddress() const noexcept override { return macAddress; }

	void OpenDataPort(Port port) noexcept override;
	void TerminateDataPort() noexcept override;

	// The remaining functions are specific to the WiFi version
	GCodeResult HandleWiFiCode(int mcode, GCodeBuffer &gb, const StringRef& reply, OutputBuffer*& longReply) THROWS(GCodeException);
	WifiFirmwareUploader *GetWifiUploader() const noexcept { return uploader; }
	void StartWiFi() noexcept;
	void ResetWiFi() noexcept;
	void ResetWiFiForUpload(bool external) noexcept;
	const char *GetWiFiServerVersion() const noexcept { return wiFiServerVersion; }
	static const char* TranslateWiFiState(WiFiState w) noexcept;
	void SpiInterrupt() noexcept;
	void EspRequestsTransfer() noexcept;
	void UpdateSocketStatus(uint16_t connectedSockets, uint16_t otherEndClosedSockets) noexcept;

protected:
	DECLARE_OBJECT_MODEL

private:
	void InitSockets() noexcept;
	void TerminateSockets() noexcept;
	void TerminateSockets(Port port) noexcept;
	void StopListening(Port port) noexcept;

	void StartProtocol(NetworkProtocol protocol) noexcept
	pre(protocol < NumProtocols);

	void ShutdownProtocol(NetworkProtocol protocol) noexcept
	pre(protocol < NumProtocols);

	void ReportOneProtocol(NetworkProtocol protocol, const StringRef& reply) const noexcept
	pre(protocol < NumProtocols);

	NetworkProtocol GetProtocolByLocalPort(Port port) const noexcept;

	void SetupSpi() noexcept;

	int32_t SendCommand(NetworkCommand cmd, SocketNumber socket, uint8_t flags, const void *dataOut, size_t dataOutLength, void* dataIn, size_t dataInLength) noexcept;

	template<class T> int32_t SendCommand(NetworkCommand cmd, SocketNumber socket, uint8_t flags, const void *dataOut, size_t dataOutLength, Receiver<T>& recvr) noexcept
	{
		return SendCommand(cmd, socket, flags, dataOut, dataOutLength, recvr.DmaPointer(), recvr.Size());
	}

	void SendListenCommand(Port port, NetworkProtocol protocol, unsigned int maxConnections) noexcept;
	void GetNewStatus() noexcept;
	static const char* TranslateWiFiResponse(int32_t response) noexcept;

	static const char* TranslateEspResetReason(uint32_t reason) noexcept;

	Platform& platform;
	uint32_t lastTickMillis;

	WifiFirmwareUploader *uploader;
	TaskHandle espWaitingTask;

	WiFiSocket *sockets[NumWiFiTcpSockets];
	size_t currentSocket;

	Port portNumbers[NumProtocols];					// port number used for each protocol
	Port ftpDataPort;
	bool closeDataPort;
	bool protocolEnabled[NumProtocols];				// whether each protocol is enabled

	WiFiState requestedMode;
	WiFiState currentMode;
	bool activated;
	volatile bool espStatusChanged;

	IPAddress ipAddress;
	IPAddress netmask;
	IPAddress gateway;
	MacAddress macAddress;
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
