/*
 * EthernetInterface.cpp
 *
 *  Created on: 24 Nov 2017
 *      Authors: David and Christian
 */

// Define this to keep the ASF status codes from being included. Without it ERR_TIMEOUT is defined twice
#define NO_STATUS_CODES

#include "LwipEthernetInterface.h"

#if HAS_LWIP_NETWORKING

#include <Networking/NetworkBuffer.h>
#include "LwipSocket.h"

#include <Platform/Platform.h>
#include <Platform/RepRap.h>
#include <Networking/HttpResponder.h>
#include <Networking/FtpResponder.h>
#include <Networking/TelnetResponder.h>
#if SUPPORT_MQTT
#include <Networking/MQTT/MqttClient.h>
#endif
#if SUPPORT_MULTICAST_DISCOVERY
# include <Networking/MulticastDiscovery/MulticastResponder.h>
#endif

#include <General/IP4String.h>
#include <Version.h>								// version is reported by MDNS
#include "GMAC/ethernet_sam.h"

#if SAME70
# include <Hardware/SAME70/Ethernet/GmacInterface.h>
#elif SAME5x
# include <Hardware/SAME5x/Ethernet/GmacInterface.h>
#endif

extern "C"
{

#ifdef LWIP_STATS
#include "lwip/stats.h"
#endif

#include "lwip/mem.h"
#include "lwip/dhcp.h"
#include "lwip/tcp.h"
#include "lwip/altcp.h"
#if LWIP_ALTCP_TLS
#include "lwip/altcp_tls.h"
#endif
#include "lwip/timeouts.h"

#include "lwip/apps/netbiosns.h"
#include "lwip/apps/mdns.h"

extern struct netif gs_net_if;
}

// LwIP heap buffer: defined here and referenced via LWIP_RAM_HEAP_POINTER in lwipopts.h.
// Allocated dynamically in Start() to avoid consuming RAM when networking is never enabled.
// Must have C linkage so that LwIP's C code (mem.c) can access it without name mangling.
extern "C" { void *lwipRamHeap = nullptr; }

#if LWIP_ALTCP_TLS_MBEDTLS
// mbedTLS allocator wrappers - routes mbedtls_calloc/free through the lwIP heap.
// Declared in LibMbedTls/configs/config-rrf.h with size_t parameters to avoid
// mem_size_t type differences between SAME54 (u16_t) and SAME70 (u32_t).
extern "C" void *mbedtls_lwip_calloc(size_t count, size_t size)
{
	return mem_calloc((mem_size_t)count, (mem_size_t)size);
}
extern "C" void mbedtls_lwip_free(void *ptr)
{
	mem_free(ptr);
}
#endif

// Interface to get access to the PBUF memory pool so that when we are not going to enable Ethernet we can use the memory for something else
// These functions may used on the Duet 3 Mini when the WiFi interface is enabled or SBC mode is enabled.
// They may also be used Duet 3 6HC/6XD to allocate the SBC buffer.
// The pool descriptors are read-only and set up by static initialisation

#include "AllocateFromPbufPool.h"

static uint8_t *pbufPoolBase = nullptr;
static size_t pbufPoolBytesLeft = 0;

// Flag that allocation from the PBUF pool is allowed and set up the variables
void InitAllocationFromPbufPool() noexcept
{
	if (pbufPoolBase == nullptr)
	{
		// First call so set up our variables
		const memp_desc *const memp = memp_pools[MEMP_PBUF_POOL];
		pbufPoolBytesLeft = memp->num * (uint32_t)memp->size;
		pbufPoolBase = memp->base;				// Lwip guarantees that this is 4-byte aligned
	}
}

// Function to allocate memory from the static memory allocated to the Lwip PBUF pool. Use this only when Lwip Ethernet will definitely not be used.
// Return a pointer to the allocated memory, or nullptr if there was insufficient memory left in the pool.
void *AllocateFromPbufPool(size_t bytes) noexcept
{
	if (pbufPoolBase != nullptr)
	{
		bytes = (bytes + 3) & (~3);				// round up to a multiple of 4 bytes to keep the memory 4-byte aligned
		if (bytes <= pbufPoolBytesLeft)
		{
			void *const ret = pbufPoolBase;
			pbufPoolBase += bytes;
			pbufPoolBytesLeft -= bytes;
			return ret;
		}
	}
	return nullptr;
}

// MDNS service strings for plain protocols (index matches NetworkProtocol)
const char * const MdnsServiceStrings[NumSelectableProtocols] =
{
	"_http", "_ftp", "_telnet",
#if SUPPORT_MULTICAST_DISCOVERY
	"_duet_discovery",
#else
	nullptr,
#endif
	nullptr		// MQTT is client-only, no mDNS service
};

#if LWIP_ALTCP_TLS
// MDNS service strings for TLS-secured protocol variants (nullptr = none)
const char * const MdnsTlsServiceStrings[NumSelectableProtocols] =
{
	"_https", "_ftps", "_telnets", nullptr, nullptr
};
#endif

const char * const MdnsTxtRecords[2] = { "product=" FIRMWARE_NAME, "version=" VERSION };
constexpr unsigned int MdnsTtl = 10 * 60;			// same value as on the Duet 0.6/0.8.5

constexpr uint8_t ListenBacklog = 8;
#if SAME70
constexpr uint8_t TlsListenBacklog = 4;				// Limit backlog for incoming TLS connections to reduce RAM usage
#else
constexpr uint8_t TlsListenBacklog = 2;
#endif

/*-----------------------------------------------------------------------------------*/

static LwipEthernetInterface *ethernetInterface;

# include <RTOSIface/RTOSIface.h>
Mutex lwipMutex;

extern "C"
{
	// Callback functions for LWIP (may be called from ISR)
	// This occasionally seems to get called with a null pcb argument, so check for that here
	static err_t conn_accept(void *arg, altcp_pcb *pcb, err_t err) noexcept
	{
		LWIP_UNUSED_ARG(arg);
		LWIP_UNUSED_ARG(err);

		if (pcb != nullptr)
		{
			if (ethernetInterface->ConnectionEstablished(pcb))
			{
				// A socket has accepted this connection and will deal with it
				return ERR_OK;
			}

			if (reprap.Debug(Module::Network))
			{
				debugPrintf("LWIP accept rejected: lport=%u rport=%u, aborting\n", altcp_get_port(pcb, 1), altcp_get_port(pcb, 0));
			}
			altcp_abort(pcb);
		}
		return ERR_ABRT;
	}

}	// end extern "C"

/*-----------------------------------------------------------------------------------*/

LwipEthernetInterface::LwipEthernetInterface(Platform& p) noexcept
	: platform(p), closeDataPort(false), activated(false), initialised(false), usingDhcp(false)
{
	ethernetInterface = this;

	// Create the sockets
	for (size_t i : ARRAY_INDICES(sockets))
	{
		sockets[i] = new LwipSocket(this);
	}
}

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(LwipEthernetInterface, __VA_ARGS__)

constexpr ObjectModelTableEntry LwipEthernetInterface::objectModelTable[] =
{
	// These entries must be in alphabetical order
	{ "actualIP",			OBJECT_MODEL_FUNC(self->ipAddress),			ObjectModelEntryFlags::none },
	{ "gateway",			OBJECT_MODEL_FUNC(self->gateway),			ObjectModelEntryFlags::none },
	{ "mac",				OBJECT_MODEL_FUNC(self->macAddress),		ObjectModelEntryFlags::none },
	{ "state",				OBJECT_MODEL_FUNC(self->GetStateName()),	ObjectModelEntryFlags::none },
	{ "subnet",				OBJECT_MODEL_FUNC(self->netmask),			ObjectModelEntryFlags::none },
	{ "type",				OBJECT_MODEL_FUNC_NOSELF("ethernet"),		ObjectModelEntryFlags::verbose },
};

constexpr uint8_t LwipEthernetInterface::objectModelTableDescriptor[] = { 1, 6 };

DEFINE_GET_OBJECT_MODEL_TABLE(LwipEthernetInterface)

void LwipEthernetInterface::Init() noexcept
{
	interfaceMutex.Create("LwipIface");
	lwipMutex.Create("LwipCore");

	// Clear the PCBs
	for (altcp_pcb*& pcb : listeningPcbs)
	{
		pcb = nullptr;
	}
#if LWIP_ALTCP_TLS
	for (altcp_pcb*& pcb : tlsListeningPcbs)
	{
		pcb = nullptr;
	}
#endif

	macAddress = platform.GetDefaultMacAddress();
}

void LwipEthernetInterface::IfaceStartProtocol(NetworkProtocol protocol) noexcept
{
	StartProtocol(protocol);

	MutexLocker lock(lwipMutex);
	RebuildMdnsServices();
}

void LwipEthernetInterface::IfaceShutdownProtocol(NetworkProtocol protocol, bool permanent) noexcept
{
	ShutdownProtocol(protocol);
	if (permanent)
	{
		MutexLocker lock(lwipMutex);
		RebuildMdnsServices();
	}
}

#if HAS_CLIENTS
void LwipEthernetInterface::ConnectProtocol(NetworkProtocol protocol) noexcept
{
	altcp_pcb *pcb = altcp_new(nullptr);
	if (pcb == nullptr)
	{
		platform.Message(ErrorMessage, "unable to allocate a pcb\n");
	}
	else
	{
		ip_addr_t remote;
		memset(&remote, 0, sizeof(remote));
		remote.addr = ipAddresses[protocol];
		err_t res = altcp_connect(pcb, &remote, portNumbers[protocol], conn_accept);

		if (res != ERR_OK)
		{
			platform.Message(ErrorMessage, "tcp_connect call failed\n");
		}
	}

	switch(protocol)
	{
#if SUPPORT_MQTT
	case MqttProtocol:
		sockets[MqttSocketNumber]->Init(MqttSocketNumber, portNumbers[protocol], protocol, true);
		break;
#endif
	}
}
#endif

#if LWIP_ALTCP_TLS

// Maximum allowed size for a PEM certificate or key file
static constexpr FilePosition MaxPemFileSize = 2048;

// Read a PEM file from /sys into a newly allocated buffer. Returns the buffer and sets len.
// The buffer includes a null terminator (required by mbedTLS PEM parsing) which is included in len.
// Returns nullptr if the file cannot be opened, is too large, or cannot be read.
uint8_t *LwipEthernetInterface::ReadPemFile(const char *filename, size_t& len) noexcept
{
	FileStore *f = MassStorage::OpenFile(filename, OpenMode::read, 0);
	if (f == nullptr)
	{
		return nullptr;
	}

	const FilePosition fileLen = f->Length();
	if (fileLen > MaxPemFileSize)
	{
		f->Close();
		return nullptr;
	}

	// Allocate fileLen + 1 for null terminator (mbedTLS PEM parser requires this)
	uint8_t *buf = new (std::nothrow) uint8_t[fileLen + 1];
	if (buf == nullptr)
	{
		f->Close();
		return nullptr;
	}

	const int bytesRead = f->Read(reinterpret_cast<char *>(buf), fileLen);
	f->Close();

	if (bytesRead != (int)fileLen)
	{
		delete[] buf;
		return nullptr;
	}

	buf[fileLen] = 0;		// null terminate for PEM parser
	len = fileLen + 1;
	return buf;
}

// Load TLS certificate and private key from TlsCertFile and TlsKeyFile
// Returns true if the TLS config was created successfully
bool LwipEthernetInterface::LoadTlsCertificates(const StringRef& reply) noexcept
{
	if (tlsConfig != nullptr)
	{
		return true;		// already loaded
	}

	size_t certLen = 0, keyLen = 0;
	uint8_t *certBuf = ReadPemFile(TlsCertFile, certLen);
	if (certBuf == nullptr)
	{
		reply.printf("Failed to load TLS certificate %s", TlsCertFile);
		return false;
	}

	uint8_t *keyBuf = ReadPemFile(TlsKeyFile, keyLen);
	if (keyBuf == nullptr)
	{
		delete[] certBuf;
		reply.printf("Failed to load TLS key %s", TlsKeyFile);
		return false;
	}

	MutexLocker lock(lwipMutex);
	tlsConfig = altcp_tls_create_config_server_privkey_cert(keyBuf, keyLen, nullptr, 0, certBuf, certLen);

	delete[] certBuf;
	delete[] keyBuf;

	if (tlsConfig == nullptr)
	{
		reply.copy("Failed to create TLS config - check certificate and key files");
		return false;
	}

	return true;
}

// Free the TLS config if no protocol is still using TLS. This needs to be called with lwipMutex held
void LwipEthernetInterface::FreeTlsConfig() noexcept
{
	if (tlsConfig == nullptr)
	{
		return;
	}

	// Keep the config as long as any protocol still has TLS enabled
	for (size_t i = 0; i < NumSelectableProtocols; ++i)
	{
		if (tlsProtocolEnabled[i])
		{
			return;
		}
	}

	altcp_tls_free_config(tlsConfig);
	tlsConfig = nullptr;
}

#endif // LWIP_ALTCP_TLS

void LwipEthernetInterface::StartProtocol(NetworkProtocol protocol) noexcept
{
	// Create the plain (non-TLS) listener if plain protocol is enabled
	if (protocolEnabled[protocol]
		&& listeningPcbs[protocol] == nullptr
#if SUPPORT_MULTICAST_DISCOVERY
		&& protocol != MulticastDiscoveryProtocol
#endif
#if SUPPORT_MQTT
		&& protocol != MqttProtocol
#endif
	)
	{
		MutexLocker lock(lwipMutex);
		altcp_pcb *pcb = altcp_new(nullptr);
		if (pcb == nullptr)
		{
			platform.Message(ErrorMessage, "unable to allocate a pcb\n");
		}
		else
		{
			if (altcp_bind(pcb, IP_ADDR_ANY, portNumbers[protocol]) != ERR_OK)
			{
				platform.Message(ErrorMessage, "tcp_bind call failed\n");
				altcp_abort(pcb);
			}
			else
			{
				altcp_pcb *const listeningPcb = altcp_listen_with_backlog(pcb, ListenBacklog);
				if (listeningPcb == nullptr)
				{
					platform.Message(ErrorMessage, "tcp_listen call failed\n");
					altcp_abort(pcb);
				}
				else
				{
					listeningPcbs[protocol] = listeningPcb;
					altcp_accept(listeningPcbs[protocol], conn_accept);
				}
			}
		}
	}

#if LWIP_ALTCP_TLS
	// Create the TLS listener if TLS is enabled for this protocol
	if (tlsProtocolEnabled[protocol] && tlsListeningPcbs[protocol] == nullptr)
	{
		if (tlsConfig == nullptr)
		{
			// TLS certificates were not loaded successfully, stop here
			return;
		}

		MutexLocker lock(lwipMutex);
		altcp_pcb *pcb = altcp_tls_new(tlsConfig, IPADDR_TYPE_V4);
		if (pcb == nullptr)
		{
			platform.Message(ErrorMessage, "unable to allocate a TLS pcb\n");
		}
		else
		{
			if (altcp_bind(pcb, IP_ADDR_ANY, tlsPortNumbers[protocol]) != ERR_OK)
			{
				platform.Message(ErrorMessage, "tcp_bind call for TLS failed\n");
				altcp_abort(pcb);
			}
			else
			{
				altcp_pcb *const listeningPcb = altcp_listen_with_backlog(pcb, TlsListenBacklog);
				if (listeningPcb == nullptr)
				{
					platform.Message(ErrorMessage, "tcp_listen call for TLS failed\n");
					altcp_abort(pcb);
				}
				else
				{
					tlsListeningPcbs[protocol] = listeningPcb;
					altcp_accept(tlsListeningPcbs[protocol], conn_accept);
				}
			}
		}
	}
#endif

	switch(protocol)
	{
	case HttpProtocol:
		for (SocketNumber skt = 0; skt < NumHttpSockets; ++skt)
		{
			if (protocolEnabled[protocol])
			{
				sockets[skt]->Init(skt, portNumbers[protocol], protocol);
#if LWIP_ALTCP_TLS
				if (tlsProtocolEnabled[protocol])
				{
					sockets[skt]->InitTls(tlsPortNumbers[protocol]);
				}
#endif
			}
#if LWIP_ALTCP_TLS
			else if (tlsProtocolEnabled[protocol])
			{
				sockets[skt]->Init(skt, tlsPortNumbers[protocol], protocol);
				sockets[skt]->InitTls(tlsPortNumbers[protocol]);
			}
#endif
		}
		break;

	case FtpProtocol:
		if (protocolEnabled[protocol])
		{
			sockets[FtpSocketNumber]->Init(FtpSocketNumber, portNumbers[protocol], protocol);
#if LWIP_ALTCP_TLS
			if (tlsProtocolEnabled[protocol])
			{
				sockets[FtpSocketNumber]->InitTls(tlsPortNumbers[protocol]);
			}
#endif
		}
#if LWIP_ALTCP_TLS
		else if (tlsProtocolEnabled[protocol])
		{
			sockets[FtpSocketNumber]->Init(FtpSocketNumber, tlsPortNumbers[protocol], protocol);
			sockets[FtpSocketNumber]->InitTls(tlsPortNumbers[protocol]);
		}
#endif
		break;

	case TelnetProtocol:
		if (protocolEnabled[protocol])
		{
			sockets[TelnetSocketNumber]->Init(TelnetSocketNumber, portNumbers[protocol], protocol);
#if LWIP_ALTCP_TLS
			if (tlsProtocolEnabled[protocol])
			{
				sockets[TelnetSocketNumber]->InitTls(tlsPortNumbers[protocol]);
			}
#endif
		}
#if LWIP_ALTCP_TLS
		else if (tlsProtocolEnabled[protocol])
		{
			sockets[TelnetSocketNumber]->Init(TelnetSocketNumber, tlsPortNumbers[protocol], protocol);
			sockets[TelnetSocketNumber]->InitTls(tlsPortNumbers[protocol]);
		}
#endif
		break;

#if SUPPORT_MULTICAST_DISCOVERY
	case MulticastDiscoveryProtocol:
		MulticastResponder::Start(portNumbers[protocol]);
		break;
#endif

	default:
		break;
	}
}

void LwipEthernetInterface::ShutdownProtocol(NetworkProtocol protocol) noexcept
{
	switch(protocol)
	{
	case HttpProtocol:
		for (SocketNumber skt = 0; skt < NumHttpSockets; ++skt)
		{
			sockets[skt]->TerminateAndDisable();
		}
		break;

	case FtpProtocol:
		sockets[FtpSocketNumber]->TerminateAndDisable();
		sockets[FtpDataSocketNumber]->TerminateAndDisable();
		break;

	case TelnetProtocol:
		sockets[TelnetSocketNumber]->TerminateAndDisable();
		break;

#if SUPPORT_MULTICAST_DISCOVERY
	case MulticastDiscoveryProtocol:
		MulticastResponder::Stop();
		break;
#endif

#if SUPPORT_MQTT
	case MqttProtocol:
		sockets[MqttSocketNumber]->TerminateAndDisable();
		break;
#endif
	default:
		break;
	}

	MutexLocker lock(lwipMutex);

	if (listeningPcbs[protocol] != nullptr)
	{
		altcp_close(listeningPcbs[protocol]);
		listeningPcbs[protocol] = nullptr;
	}

#if LWIP_ALTCP_TLS
	if (tlsListeningPcbs[protocol] != nullptr)
	{
		altcp_close(tlsListeningPcbs[protocol]);
		tlsListeningPcbs[protocol] = nullptr;
	}

	FreeTlsConfig();
#endif
}

// This is called at the end of config.g processing.
// Start the network if it was enabled
void LwipEthernetInterface::Activate() noexcept
{
	if (!activated)
	{
		activated = true;
		if (GetState() == NetworkState::enabled)
		{
			Start();
		}
		else
		{
			platform.Message(NetworkInfoMessage, "Ethernet is disabled.\n");
		}
	}
}

void LwipEthernetInterface::Exit() noexcept
{
	Stop();
	ethernet_terminate();
}

// Get the network state into the reply buffer, returning true if there is some sort of error
GCodeResult LwipEthernetInterface::GetNetworkState(const StringRef& reply) noexcept
{
	ethernet_get_ipaddress(ipAddress, netmask, gateway);
	const int enableState = EnableState();
	reply.printf("Ethernet is %s, configured IP address: %s, actual IP address: %s",
					(enableState == 0) ? "disabled" : "enabled",
					IP4String(platform.GetIPAddress()).c_str(), IP4String(ipAddress).c_str());
	return GCodeResult::ok;
}

// Start up the network
void LwipEthernetInterface::Start() noexcept
{
	digitalWrite(EthernetPhyResetPin, true);			// bring the Ethernet Phy out of reset

	if (initialised)
	{
		// Bring the netif up again
		netif_set_up(&gs_net_if);
	}
	else
	{
		const char *hostname = reprap.GetNetwork().GetHostname();

		// Allocate the LwIP heap buffer.  We do this here (lazily) rather than using a static BSS array
		// so that systems that never enable Ethernet don't consume this RAM at all.
		// The size depends on whether TLS is going to be used.
#ifdef MEM_SIZE_WITH_TLS
		const size_t heapSize = tlsAllowed ? (MEM_SIZE_WITH_TLS) : (MEM_SIZE_WITHOUT_TLS);
#else
		const size_t heapSize = MEM_SIZE_WITHOUT_TLS;
#endif
		// LwIP's mem_init() requires: LWIP_MEM_ALIGN_SIZE(heapSize) + 2 * SIZEOF_STRUCT_MEM bytes plus
		// up to MEM_ALIGNMENT-1 bytes for alignment. We add MEM_ALIGNMENT extra to be safe.
		const size_t heapAlloc = heapSize + 32 + MEM_ALIGNMENT;	// 32 > 2*SIZEOF_STRUCT_MEM, safe margin for alignment
		lwipRamHeap = new uint8_t[heapAlloc];
		mem_set_size((mem_size_t)heapSize);

		// Allow the MAC address to be set only before LwIP is started...
		ethernet_configure_interface(macAddress.bytes, hostname);
		init_ethernet(DefaultIpAddress, DefaultNetMask, DefaultGateway);

		if (ethernetif_GetPhyInitResult() != GMAC_OK)
		{
			SetState(NetworkState::initFailed);
			return;
		}

		// Initialise mDNS subsystem
		mdns_resp_init();
		mdns_resp_add_netif(&gs_net_if, hostname);

		// Initialise NetBIOS responder
		netbiosns_init();
		netbiosns_set_name(hostname);

		initialised = true;
	}

	SetState(NetworkState::establishingLink);
}

// Stop the network
void LwipEthernetInterface::Stop() noexcept
{
	if (GetState() != NetworkState::disabled)
	{
		netif_set_down(&gs_net_if);

		SetPinMode(EthernetPhyResetPin, OUTPUT_LOW);		// hold the Ethernet Phy chip in reset
		SetState(NetworkState::disabled);
	}
}

// Main spin loop. If 'full' is true then we are being called from the main spin loop. If false then we are being called during HSMCI idle time.
void LwipEthernetInterface::Spin() noexcept
{
	MutexLocker lock(lwipMutex);

	switch(GetState())
	{
	case NetworkState::enabled:
	case NetworkState::disabled:
	default:
		// Nothing to do
		break;

	case NetworkState::establishingLink:
		if (ethernet_establish_link())
		{
			usingDhcp = platform.GetIPAddress().IsNull();
			if (usingDhcp)
			{
				// IP address is all zeros, so use DHCP
				SetState(NetworkState::obtainingIP);
//				debugPrintf("Link established, getting IP address\n");
				IPAddress nullAddress;
				ethernet_set_configuration(nullAddress, nullAddress, nullAddress);
				dhcp_start(&gs_net_if);
			}
			else
			{
				// Using static IP address
				SetState(NetworkState::connected);
//				debugPrintf("Link established, network running\n");
				ethernet_set_configuration(platform.GetIPAddress(), platform.NetMask(), platform.GateWay());
			}
		}
		break;

	case NetworkState::obtainingIP:
		if (ethernet_link_established())
		{
			// Service pending lwIP timers
			sys_check_timeouts();

			// Have we obtained an IP address yet?
			ethernet_get_ipaddress(ipAddress, netmask, gateway);
			if (!ipAddress.IsNull())
			{
				// Notify the mDNS responder about this
				SetState(NetworkState::connected);
//				debugPrintf("IP address obtained, network running\n");
			}
		}
		else
		{
//			debugPrintf("Lost phy link\n");
			TerminateSockets();
			SetState(NetworkState::establishingLink);
		}
		break;

	case NetworkState::connected:
		InitSockets();
		RebuildMdnsServices();
		ethernet_get_ipaddress(ipAddress, netmask, gateway);
		platform.MessageF(NetworkInfoMessage, "Ethernet running, IP address = %s\n", IP4String(ipAddress).c_str());
		SetState(NetworkState::active);
		reprap.NetworkUpdated();
		break;

	case NetworkState::active:
		// Check that the link is still up
		if (ethernet_link_established())
		{
			// Service pending lwIP timers
			sys_check_timeouts();

#if HAS_CLIENTS
			// Maintain client connections
			for (uint8_t p = 0; p < NumSelectableProtocols; p++)
			{
				if (protocolEnabled[p])
				{
					if (reprap.GetNetwork().StartClient(this, p))
					{
						ConnectProtocol(p);
					}
				}
				else
				{
					reprap.GetNetwork().StopClient(this, p);
				}
			}
#endif

			// Poll all TCP sockets
			for (LwipSocket *s : sockets)
			{
				s->Poll();
			}

			// Check if the data port needs to be closed
			if (closeDataPort && !sockets[FtpDataSocketNumber]->IsClosing())
			{
				TerminateDataPort();
			}
		}
		else
		{
//			debugPrintf("Lost phy link\n");
			TerminateSockets();
			SetState(NetworkState::establishingLink);
		}
		break;
	}
}

void LwipEthernetInterface::Diagnostics(const StringRef& reply) noexcept
{
	reply.lcatf("= Ethernet =\nInterface state: %s", GetStateName());
	ethernetif_diagnostics(reply);
	for (const LwipSocket *s : sockets)
	{
		reply.catf(" %d", s->GetState());
	}

#if LWIP_STATS
	// lwIP is only initialised in Start(), which Activate() skips when the interface is disabled.
	// Calling stats_display() before that deref's uninitialised memp tables and crashes
	if (reprap.Debug(Module::Network) && GetState() != NetworkState::disabled)
	{
		// This prints LwIP diagnostics data to the USB port - blocking!
		stats_display();
	}
#endif
}

// Enable or disable the network. For Ethernet the ssid parameter is not used.
// tlsParam: -1 = clear stored TLS material (/sys/server.{crt,key}) and come up plain
//            0 = plain mode (no TLS heap allocation)
//            1 = enable TLS (load /sys/server.{crt,key}, allocate TLS-sized LwIP heap)
GCodeResult LwipEthernetInterface::EnableInterface(int mode, const StringRef& ssid, const StringRef& reply, int tlsParam) noexcept
{
#if LWIP_ALTCP_TLS
	// Handle T-1 first: securely wipe and delete the SD-resident cert/key. Subsequent bring-up is plain.
	// Only honoured while the interface is down, matching the documented rule
	if (tlsParam < 0 && !activated)
	{
		bool wiped = false;
		String<MaxFilenameLength> path;
		path.copy(TlsCertFile);
		if (MassStorage::SecureDelete(path.GetRef(), ErrorMessageMode::noMessage))
		{
			wiped = true;
		}
		path.copy(TlsKeyFile);
		if (MassStorage::SecureDelete(path.GetRef(), ErrorMessageMode::noMessage))
		{
			wiped = true;
		}
		if (wiped)
		{
			platform.Message(NetworkInfoMessage, "TLS: stored cert/key wiped and deleted from /sys/\n");
		}
	}
	const bool tlsAllowedParam = (tlsParam > 0);
#else
	(void)tlsParam;
	const bool tlsAllowedParam = false;
#endif

	if (!activated)
	{
		if (mode == 0)
		{
			SetState(NetworkState::disabled);
		}
		else
		{
#if LWIP_ALTCP_TLS
			if (!initialised)
			{
				tlsAllowed = tlsAllowedParam;
			}
#endif
			Start();
#if LWIP_ALTCP_TLS
			if (tlsAllowed && !LoadTlsCertificates(reply))
			{
				return GCodeResult::warning;
			}
#endif
		}
	}
	else if (mode == 0)
	{
		if (GetState() != NetworkState::disabled)
		{
			Stop();
			platform.Message(NetworkInfoMessage, "Network stopped\n");
		}

	}
	else if (GetState() == NetworkState::disabled)
	{
#if LWIP_ALTCP_TLS
		if (!initialised || (tlsConfig == nullptr && tlsAllowed && !tlsAllowedParam))
		{
			tlsAllowed = tlsAllowedParam;
		}
#endif
		SetState(NetworkState::enabled);
		Start();
#if LWIP_ALTCP_TLS
		if (tlsAllowed && !LoadTlsCertificates(reply))
		{
			return GCodeResult::warning;
		}
#endif
	}
	return GCodeResult::ok;
}

int LwipEthernetInterface::EnableState() const noexcept
{
	return (GetState() == NetworkState::disabled) ? 0 : 1;
}

bool LwipEthernetInterface::ConnectionEstablished(altcp_pcb *pcb) noexcept
{
	for (LwipSocket *s : sockets)
	{
		if (s->AcceptConnection(pcb))
		{
			// Socket has accepted the incoming connection
			return true;
		}
	}

	// No more free socket for this connection, terminate it
	return false;
}

void LwipEthernetInterface::SetIPAddress(IPAddress p_ipAddress, IPAddress p_netmask, IPAddress p_gateway) noexcept
{
	MutexLocker lock(lwipMutex);

	if (GetState() == NetworkState::obtainingIP || GetState() == NetworkState::active)
	{
		const bool wantDhcp = p_ipAddress.IsNull();
		if (wantDhcp)
		{
			// Acquire dynamic IP address
			if (!usingDhcp)
			{
				SetState(NetworkState::obtainingIP);
				IPAddress nullAddress;
				ethernet_set_configuration(nullAddress, nullAddress, nullAddress);
				dhcp_start(&gs_net_if);
				usingDhcp = true;
			}
		}
		else
		{
			// Set static IP address
			if (usingDhcp)
			{
				if (GetState() == NetworkState::obtainingIP)
				{
					dhcp_stop(&gs_net_if);
				}
				SetState(NetworkState::active);
				usingDhcp = false;
			}

			ethernet_set_configuration(p_ipAddress, p_netmask, p_gateway);
			mdns_resp_netif_settings_changed(&gs_net_if);
		}
	}
}

void LwipEthernetInterface::UpdateHostname(const char *hostname) noexcept
{
	if (initialised)
	{
		MutexLocker lock(lwipMutex);
		netbiosns_set_name(hostname);
		RebuildMdnsServices();			// This updates the mDNS hostname too
	}
}

GCodeResult LwipEthernetInterface::SetMacAddress(const MacAddress& mac, const StringRef& reply) noexcept
{
	macAddress = mac;
	return GCodeResult::ok;
}

bool LwipEthernetInterface::OpenDataPort(TcpPort port, bool useTls) noexcept
{
	if (listeningPcbs[FtpDataProtocol] != nullptr)
	{
		if (reprap.Debug(Module::Network))
		{
			debugPrintf("LWIP OpenDataPort replacing existing FTP data listener\n");
		}
		closeDataPort = true;
		TerminateDataPort();
	}

	MutexLocker lock(lwipMutex);

#if LWIP_ALTCP_TLS
	altcp_pcb *pcb = useTls ? altcp_tls_new(tlsConfig, IPADDR_TYPE_V4) : altcp_new(nullptr);
#else
	altcp_pcb *pcb = altcp_new(nullptr);
#endif
	if (pcb == nullptr)
	{
		if (reprap.Debug(Module::Network))
		{
			debugPrintf("LWIP OpenDataPort alloc failed: port=%u tls=%d\n", port, useTls);
		}
		return false;
	}

	if (altcp_bind(pcb, IP_ADDR_ANY, port) != ERR_OK)
	{
		if (reprap.Debug(Module::Network))
		{
			debugPrintf("tcp_bind call failed for FTP data connection\n");
		}
		altcp_abort(pcb);
		return false;
	}

	altcp_pcb *const listeningPcb = altcp_listen(pcb);
	if (listeningPcb == nullptr)
	{

		if (reprap.Debug(Module::Network))
		{
			debugPrintf("tcp_listen call failed for FTP data connection\n");
		}
		altcp_abort(pcb);
		return false;
	}

	altcp_accept(listeningPcb, conn_accept);
	listeningPcbs[FtpDataProtocol] = listeningPcb;
	sockets[FtpDataSocketNumber]->Init(FtpDataSocketNumber, port, FtpDataProtocol);
#if LWIP_ALTCP_TLS
	if (useTls && tlsConfig != nullptr)
	{
		sockets[FtpDataSocketNumber]->InitTls(port);
	}
#endif
	if (reprap.Debug(Module::Network))
	{
		debugPrintf("LWIP OpenDataPort ok: port=%u tls=%d\n", port, useTls);
	}

	return true;
}

// Close FTP data port and purge associated resources
void LwipEthernetInterface::TerminateDataPort() noexcept
{
	if (reprap.Debug(Module::Network))
	{
		debugPrintf("LWIP TerminateDataPort: closeDataPort=%d socketClosing=%d listener=%d\n", closeDataPort, sockets[FtpDataSocketNumber]->IsClosing(), listeningPcbs[FtpDataProtocol] != nullptr);
	}

	if (closeDataPort || !sockets[FtpDataSocketNumber]->IsClosing())
	{
		closeDataPort = false;
		sockets[FtpDataSocketNumber]->TerminateAndDisable();

		MutexLocker lock(lwipMutex);
		if (listeningPcbs[FtpDataProtocol] != nullptr)
		{
			if (altcp_close(listeningPcbs[FtpDataProtocol]) != ERR_OK)
			{
				altcp_abort(listeningPcbs[FtpDataProtocol]);
			}
			listeningPcbs[FtpDataProtocol] = nullptr;
		}
	}
	else
	{
		// The socket may be waiting for a ACKs and a graceful disconnect.
		// Give it some more time
		closeDataPort = true;
	}
}

void LwipEthernetInterface::InitSockets() noexcept
{
	for (size_t i = 0; i < NumSelectableProtocols; ++i)
	{
		if (protocolEnabled[i]
#if LWIP_ALTCP_TLS
			|| tlsProtocolEnabled[i]
#endif
		)
		{
			StartProtocol(i);
		}
	}
}

void LwipEthernetInterface::TerminateSockets() noexcept
{
	for (LwipSocket *socket : sockets)
	{
		socket->Terminate();
	}

	// Also drop all listener PCBs so InitSockets() recreates them cleanly.
	for (altcp_pcb *&pcb : listeningPcbs)
	{
		if (pcb != nullptr)
		{
			altcp_accept(pcb, nullptr);
			if (altcp_close(pcb) != ERR_OK)
			{
				altcp_abort(pcb);
			}
			pcb = nullptr;
		}
	}

#if LWIP_ALTCP_TLS
	for (altcp_pcb *&pcb : tlsListeningPcbs)
	{
		if (pcb != nullptr)
		{
			altcp_accept(pcb, nullptr);
			if (altcp_close(pcb) != ERR_OK)
			{
				altcp_abort(pcb);
			}
			pcb = nullptr;
		}
	}
#endif
}

void GetServiceTxtEntries(struct mdns_service *service, void *txt_userdata)
{
	for (size_t i = 0; i < ARRAY_SIZE(MdnsTxtRecords); i++)
	{
		mdns_resp_add_service_txtitem(service, MdnsTxtRecords[i], strlen(MdnsTxtRecords[i]));
	}
}

void LwipEthernetInterface::RebuildMdnsServices() noexcept
{
	mdns_resp_remove_netif(&gs_net_if);
	mdns_resp_add_netif(&gs_net_if, reprap.GetNetwork().GetHostname());
	mdns_resp_add_service(&gs_net_if, "echo", "_echo", DNSSD_PROTO_TCP, 0, nullptr, nullptr);

	for (size_t protocol = 0; protocol < NumSelectableProtocols; protocol++)
	{
#if SUPPORT_MQTT
		if (protocol == MqttProtocol) { continue; }		// MQTT is client-only
#endif
#if !SUPPORT_MULTICAST_DISCOVERY
		if (protocol == MulticastDiscoveryProtocol) { continue; }
#endif
		if (protocolEnabled[protocol] && MdnsServiceStrings[protocol] != nullptr)
		{
			service_get_txt_fn_t txtFunc = (protocol == HttpProtocol) ? GetServiceTxtEntries : nullptr;
			mdns_resp_add_service(&gs_net_if, ProtocolNames[protocol], MdnsServiceStrings[protocol], DNSSD_PROTO_TCP, portNumbers[protocol], txtFunc, nullptr);
		}
#if LWIP_ALTCP_TLS
		if (tlsProtocolEnabled[protocol] && MdnsTlsServiceStrings[protocol] != nullptr)
		{
			service_get_txt_fn_t txtFunc = (protocol == HttpProtocol) ? GetServiceTxtEntries : nullptr;
			mdns_resp_add_service(&gs_net_if, ProtocolNames[protocol], MdnsTlsServiceStrings[protocol], DNSSD_PROTO_TCP, tlsPortNumbers[protocol], txtFunc, nullptr);
		}
#endif
	}

	mdns_resp_netif_settings_changed(&gs_net_if);
}

#endif	// HAS_LWIP_NETWORKING

// End
