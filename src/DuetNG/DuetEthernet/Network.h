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

// We have 8 sockets available on the W5500.
const size_t NumHttpSockets = 4;				// sockets 0-3 are for HTTP
const SocketNumber FtpSocketNumber = 4;
const SocketNumber FtpDataSocketNumber = 5;		// TODO can we allocate this dynamically when required, to allow more http sockets most of the time?
const SocketNumber TelnetSocketNumber = 6;
const size_t NumTcpSockets = 7;
const SocketNumber DhcpSocketNumber = 7;		// TODO can we allocate this dynamically when required, to allow more http sockets most of the time?

class Platform;

// The main network class that drives the network.
class Network
{
public:
	const uint8_t *GetIPAddress() const;
	void SetIPAddress(const uint8_t p_ipAddress[], const uint8_t p_netmask[], const uint8_t p_gateway[]);

	Network(Platform* p);
	void Init();
	void Activate();
	void Exit();
	void Spin();
	void Diagnostics(MessageType mtype);
	void Start();
	void Stop();

	bool Lock();
	void Unlock();
	bool InLwip() const;

	void Enable();
	void Disable();
	bool IsEnabled() const;

	void SetHttpPort(uint16_t port);
	uint16_t GetHttpPort() const;

	void SetHostname(const char *name);

	// Interfaces for the Webserver

	NetworkTransaction *GetTransaction(Connection conn = NoConnection);

	void OpenDataPort(uint16_t port);
	uint16_t GetDataPort() const;
	void CloseDataPort();

	void SaveDataConnection();
	void SaveFTPConnection();
	void SaveTelnetConnection();

	bool AcquireFTPTransaction();
	bool AcquireDataTransaction();
	bool AcquireTelnetTransaction();

	static uint16_t GetLocalPort(Connection conn);
	static uint16_t GetRemotePort(Connection conn);
	static uint32_t GetRemoteIP(Connection conn);
	static bool IsConnected(Connection conn);
	static bool IsTerminated(Connection conn);
	static void Terminate(Connection conn);

private:
	enum class NetworkState
	{
		disabled,					// WiFi not active
		enabled,					// WiFi enabled but not started yet
		establishingLink,			// starting up (waiting for initialisation)
		obtainingIP,
		active
	};

	void AppendTransaction(NetworkTransaction* * list, NetworkTransaction *r);
	void PrependTransaction(NetworkTransaction* * list, NetworkTransaction *r);
	bool AcquireTransaction(Socket *cs);

	void InitSockets();
	void TerminateSockets();

	Platform *platform;
    float longWait;
    uint32_t lastTickMillis;

    Socket sockets[NumTcpSockets];
    size_t nextSocketToPoll;						// next TCP socket number to poll for read/write operations
    size_t nextSocketToProcess;						// next TCP socket number to process an incoming request from

	NetworkTransaction * freeTransactions;
	NetworkTransaction * readyTransactions;
	NetworkTransaction * writingTransactions;

    uint16_t httpPort;
	uint8_t ipAddress[4];
	uint8_t netmask[4];
	uint8_t gateway[4];
	char hostname[16];								// Limit DHCP hostname to 15 characters + terminating 0

	NetworkState state;
    bool activated;
    bool usingDhcp;
};

#endif
