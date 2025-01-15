/*
 * WiFiInterface.h
 *
 *  Created on: 27 Nov 2017
 *      Authors: Christian and David
 */

#ifndef SRC_NETWORKING_WIFIINTERFACE_H_
#define SRC_NETWORKING_WIFIINTERFACE_H_

#include <RepRapFirmware.h>

#if HAS_WIFI_NETWORKING

#include <Networking/NetworkDefs.h>
#include <Networking/NetworkInterface.h>
#include <MessageFormats.h>

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

struct MessageBufferOut
{
	MessageHeaderSamToEsp hdr;
	uint8_t data[MaxDataLength];	// data to send
};

struct alignas(16) MessageBufferIn
{
	MessageHeaderEspToSam hdr;
	uint8_t data[MaxDataLength];	// data to send
};

// The main network class that drives the network.
class WiFiInterface : public NetworkInterface
{
public:
	friend class WiFiSocket;

	explicit WiFiInterface(Platform& p) noexcept;

	void Init() noexcept override;
	void Activate() noexcept override;
	void Exit() noexcept override;
	void Spin() noexcept override;
	void Diagnostics(MessageType mtype) noexcept override;
	void Start() noexcept;
	void Stop() noexcept;

	GCodeResult EnableInterface(int mode, const StringRef& ssid, const StringRef& reply) noexcept override;			// enable or disable the network

	GCodeResult GetNetworkState(const StringRef& reply) noexcept override;
	int EnableState() const noexcept override;
	bool IsWiFiInterface() const noexcept override { return true; }

	void UpdateHostname(const char *_ecv_array hostname) noexcept override;

	IPAddress GetIPAddress() const noexcept override { return ipAddress; }
	IPAddress GetNetmask() const noexcept override { return netmask; }
	IPAddress GetGateway() const noexcept override { return gateway; }
	bool UsingDhcp() const noexcept override { return usingDhcp; }
	void SetIPAddress(IPAddress p_ip, IPAddress p_netmask, IPAddress p_gateway) noexcept override;
	GCodeResult SetMacAddress(const MacAddress& mac, const StringRef& reply) noexcept override;
	const MacAddress& GetMacAddress() const noexcept override { return macAddress; }

	void OpenDataPort(TcpPort port) noexcept override;
	void TerminateDataPort() noexcept override;

	// The remaining functions are specific to the WiFi version
	GCodeResult HandleWiFiCode(int mcode, GCodeBuffer &gb, const StringRef& reply, OutputBuffer *_ecv_null & longReply) THROWS(GCodeException);
	WifiFirmwareUploader *_ecv_null GetWifiUploader() const noexcept { return uploader; }
	void StartWiFi() noexcept;
	void ResetWiFi() noexcept;
	void ResetWiFiForUpload(bool external) noexcept;
	const char *_ecv_array GetWiFiServerVersion() const noexcept { return wiFiServerVersion.c_str(); }
	static const char *_ecv_array TranslateWiFiState(WiFiState w) noexcept;
	void SpiInterrupt() noexcept;
	void EspRequestsTransfer() noexcept;
	void UpdateSocketStatus(uint16_t connectedSockets, uint16_t otherEndClosedSockets, int8_t p_rssi) noexcept;

protected:
	DECLARE_OBJECT_MODEL

	// Disable a network protocol that is enabled. If 'permanent' is true we will leave this protocol disables, otherwise we are about to re-enable it with different parameters.
	void IfaceStartProtocol(NetworkProtocol protocol) noexcept override;

	// Enable a network protocol that is currently disabled
	void IfaceShutdownProtocol(NetworkProtocol protocol, bool permanent) noexcept override;

private:
	void InitSockets() noexcept;
	void TerminateSockets() noexcept;
	void TerminateSockets(TcpPort port, bool local = true) noexcept;
	void StopListening(TcpPort port) noexcept;

#if HAS_CLIENTS
	void ConnectProtocol(NetworkProtocol protocol) noexcept
		pre(protocol < NumSelectableProtocols);
#endif

	NetworkProtocol GetProtocolByLocalPort(TcpPort port) const noexcept;

	void SetupSpi() noexcept;

	int32_t SendCommand(NetworkCommand cmd, SocketNumber socket, uint8_t flags, uint32_t param32, const void *_ecv_null dataOut, size_t dataOutLength, void *_ecv_null dataIn, size_t dataInLength) noexcept;

	template<class T> int32_t SendCommand(NetworkCommand cmd, SocketNumber socket, uint8_t flags, const void *_ecv_null dataOut, size_t dataOutLength, Receiver<T>& recvr) noexcept
	{
		return SendCommand(cmd, socket, flags, 0, dataOut, dataOutLength, recvr.DmaPointer(), recvr.Size());
	}

	void SendListenCommand(TcpPort port, NetworkProtocol protocol, unsigned int maxConnections) noexcept;
	void SendConnectCommand(TcpPort port, NetworkProtocol protocol, uint32_t ip) noexcept;
	void GetNewStatus() noexcept;
	void spi_slave_dma_setup(uint32_t dataOutSize, uint32_t dataInSize) noexcept;

	int32_t SendCredential(size_t credIndex, const uint8_t *_ecv_array buffer, size_t bufferSize) noexcept;
	int32_t SendFileCredential(GCodeBuffer &gb, size_t credIndex) noexcept;
	int32_t SendTextCredential(GCodeBuffer &gb, size_t credIndex) noexcept;
	size_t CheckCredential(GCodeBuffer &gb, bool file = false) THROWS(GCodeException);

	static const char *_ecv_array TranslateWiFiResponse(int32_t response) noexcept;
	static const char *_ecv_array TranslateEspResetReason(uint32_t reason) noexcept;

	Platform& platform;
	uint32_t lastTickMillis;
	bool lastDataReadyPinState;
	uint8_t risingEdges;

	MessageBufferOut *_ecv_null bufferOut;
	MessageBufferIn *_ecv_null bufferIn;

	WifiFirmwareUploader *_ecv_null uploader;
	TaskHandle _ecv_null espWaitingTask;

	WiFiSocket *sockets[NumWiFiTcpSockets];
	size_t currentSocket;

	TcpPort ftpDataPort;
	bool closeDataPort;

	WiFiState requestedMode;
	WiFiState currentMode;
	bool activated;
	volatile bool espStatusChanged;

	IPAddress ipAddress;
	IPAddress netmask;
	IPAddress gateway;
	MacAddress macAddress;
	String<SsidLength> requestedSsid;
	String<SsidLength> actualSsid;

	unsigned int spiTxUnderruns;
	unsigned int spiRxOverruns;
	unsigned int reconnectCount;
	unsigned int transferAlreadyPendingCount = 0;
	unsigned int readyTimeoutCount = 0;
	unsigned int responseTimeoutCount = 0;

	String<StringLength20> wiFiServerVersion;

	uint8_t startupRetryCount;
	int8_t rssi = 0;
	bool usingDhcp = true;

	// For processing debug messages from the WiFi module
	bool serialRunning;
	bool debugPrintPending;
	char debugMessageBuffer[200];
	size_t debugMessageChars;
};

#endif	// HAS_WIFI_NETWORKING

#endif
