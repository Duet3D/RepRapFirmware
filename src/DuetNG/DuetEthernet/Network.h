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
	void Spin(bool full);
	void Diagnostics(MessageType mtype);
	void Start();
	void Stop();

	bool Lock();
	void Unlock();
	bool InLwip() const;

	void Enable();
	void Disable();
	bool IsEnabled() const;

	void SetHttpPort(Port port);
	Port GetHttpPort() const;

	void SetHostname(const char *name);

	// Interfaces for the Webserver

	NetworkTransaction *GetTransaction(Connection conn = NoConnection);

	void OpenDataPort(Port port);
	Port GetDataPort() const;
	void CloseDataPort();

	void SaveDataConnection() {}
	void SaveFTPConnection() {}
	void SaveTelnetConnection() {}

	bool AcquireFTPTransaction() { return AcquireTransaction(FtpSocketNumber); }
	bool AcquireDataTransaction() { return AcquireTransaction(FtpDataSocketNumber); }
	bool AcquireTelnetTransaction() { return AcquireTransaction(TelnetSocketNumber); }

	void Defer(NetworkTransaction *tr);

	static Port GetLocalPort(Connection conn);
	static Port GetRemotePort(Connection conn);
	static uint32_t GetRemoteIP(Connection conn);
	static bool IsConnected(Connection conn);
	static bool IsTerminated(Connection conn);
	static void Terminate(Connection conn);

private:
	enum class NetworkState
	{
		disabled,					// Network disabled
		enabled,					// Network enabled but not started yet
		establishingLink,			// starting up, waiting for link
		obtainingIP,				// link established, waiting for DHCP
		active						// network running
	};

	void InitSockets();
	void TerminateSockets();

	bool AcquireTransaction(size_t socketNumber)
	pre(socketNumber < NumTcpSockets);

	Platform * const platform;
	float longWait;
	uint32_t lastTickMillis;

	Socket sockets[NumTcpSockets];
	size_t nextSocketToPoll;						// next TCP socket number to poll for read/write operations
	size_t currentTransactionSocketNumber;			// the socket number of the last transaction we passed to the web server

	Port httpPort;
	uint8_t ipAddress[4];
	uint8_t netmask[4];
	uint8_t gateway[4];
	char hostname[16];								// Limit DHCP hostname to 15 characters + terminating 0

	NetworkState state;
	bool activated;
	bool usingDhcp;
};

#endif
