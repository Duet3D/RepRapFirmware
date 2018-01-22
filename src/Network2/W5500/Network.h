/****************************************************************************************************

RepRapFirmware - Network: RepRapPro Ormerod with Duet controller

Separated out from Platform.h by dc42 and extended by chrishamm

****************************************************************************************************/

#ifndef NETWORK_H
#define NETWORK_H

#include "Socket.h"
#include "NetworkDefs.h"
#include "MessageType.h"

class NetworkResponder;
class HttpResponder;
class FtpResponder;
class TelnetResponder;

// We have 8 sockets available on the W5500.
const size_t NumHttpSockets = 4;				// sockets 0-3 are for HTTP
const SocketNumber FtpSocketNumber = 4;
const SocketNumber FtpDataSocketNumber = 5;		// TODO can we allocate this dynamically when required, to allow more http sockets most of the time?
const SocketNumber TelnetSocketNumber = 6;
const size_t NumTcpSockets = 7;
const SocketNumber DhcpSocketNumber = 7;		// TODO can we allocate this dynamically when required, to allow more http sockets most of the time?

const unsigned int NumHttpResponders = 4;		// the number of concurrent HTTP requests we can process

class Platform;

// The main network class that drives the network.
class Network
{
public:
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

	void Enable(int mode, StringRef& reply);			// enable or disable the network
	bool GetNetworkState(StringRef& reply);
	int EnableState() const;

	void SetHostname(const char *name);

	bool FindResponder(Socket *skt, Protocol protocol);

	const uint8_t *GetIPAddress() const { return ipAddress; }
	void OpenDataPort(Port port);
	void TerminateDataPort();
	void DataPortClosing();

	void HandleHttpGCodeReply(const char *msg);
	void HandleTelnetGCodeReply(const char *msg);
	void HandleHttpGCodeReply(OutputBuffer *buf);
	void HandleTelnetGCodeReply(OutputBuffer *buf);
	uint32_t GetHttpReplySeq();

private:
	enum class NetworkState
	{
		disabled,					// Network disabled
		enabled,					// Network enabled but not started yet
		establishingLink,			// starting up, waiting for link
		obtainingIP,				// link established, waiting for DHCP
		connected,					// just established a connection
		active						// network running
	};

	void InitSockets();
	void TerminateSockets();

	void StartProtocol(Protocol protocol)
	pre(protocol < NumProtocols);

	void ShutdownProtocol(Protocol protocol)
	pre(protocol < NumProtocols);

	void ReportOneProtocol(Protocol protocol, StringRef& reply) const
	pre(protocol < NumProtocols);

	void SetIPAddress(const uint8_t p_ipAddress[], const uint8_t p_netmask[], const uint8_t p_gateway[]);

	Platform& platform;
	NetworkResponder *responders;
	NetworkResponder *nextResponderToPoll;
	FtpResponder *ftpResponder;
	TelnetResponder *telnetResponder;
	uint32_t longWait;
	uint32_t lastTickMillis;

	Socket sockets[NumTcpSockets];
	size_t nextSocketToPoll;						// next TCP socket number to poll for read/write operations

	Port portNumbers[NumProtocols];					// port number used for each protocol
	bool protocolEnabled[NumProtocols];				// whether each protocol is enabled

	NetworkState state;
	bool activated;
	bool usingDhcp;

	uint8_t ipAddress[4];
	uint8_t netmask[4];
	uint8_t gateway[4];
	char hostname[16];								// Limit DHCP hostname to 15 characters + terminating 0
};

#endif
