// Implementation of Multicast Discovery protocol

#include "fgmc_protocol.h"

#if SUPPORT_MULTICAST_DISCOVERY

#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include <Networking/Network.h>
#include <Networking/MulticastDiscovery/MulticastResponder.h>
#include <CAN/CanInterface.h>
#include <Version.h>
#include <cstring>

constexpr const char *NetworkConfigOverrideFileName = "network-override.g";

// Disable gcc warning about strncpy not being able to include a terminating null.
// We don't need a terminating null when filling in response fields (if we did then we would use SafeStrcpy instead).
#pragma GCC diagnostic ignored "-Wstringop-truncation"

FGMCProtocol::FGMCProtocol() noexcept
    : iface_id_(0),
      fgmc_device_id_(FGMCHwTypeId::FGMC_DEVICE_ID_ZERO),
      fgmc_application_type_(0),
      tx_netbuf_{0}
{
}

static void StoreBase64(char buf[8], uint32_t val) noexcept
{
	for (unsigned int i = 0; i < 8; ++i)
	{
		const uint8_t v = val & 0x0f;
		buf[i] = (v < 10) ? v + '0' : v + ('A' - 10);
		val >>= 4;
	}
}

// Fill in the 16-character unique ID string
void FGMCProtocol::BuildUniqueId() noexcept
{
	const UniqueId& id = reprap.GetPlatform().GetUniqueId();
	StoreBase64(uniqueId, id.GetDwords()[0] ^ id.GetDwords()[2]);
	StoreBase64(uniqueId + 8, id.GetDwords()[1] ^ id.GetDwords()[3]);
}

void FGMCProtocol::init() noexcept
{
	BuildUniqueId();
	for (uint32_t i = 0; i < IP_MAX_IFACES; i++)
	{
		ifaceData[i].configuredIpAddress = reprap.GetNetwork().GetIPAddress(i);
		ifaceData[i].configuredNetmask = reprap.GetNetwork().GetNetmask(i);
		ifaceData[i].configuredGateway = reprap.GetNetwork().GetGateway(i);
	}

	fgmc_device_id_ = FGMCHwTypeId::FGMC_DEVICE_ID_DUET3;
}

void FGMCProtocol::handleStream(unsigned int iFaceId, const uint8_t* inputBufferAddress, uint32_t rxLength) noexcept
{
	// backup sciopta connection handle
	// if a null pointer received => platform should execute a exception
	this->iface_id_ = iFaceId;

	if (rxLength >= static_cast<uint32_t>(sizeof(FGMC_GenericHeader)))
	{
		const FGMC_GenericHeader* const pInGenericHeader = reinterpret_cast<const FGMC_GenericHeader*>(inputBufferAddress);

		// read incoming packetid
		const uint32_t packetId = pInGenericHeader->fgmc_packet_id_;
		InterfaceData& ifData = ifaceData[iface_id_];
		if (!ifData.insertPacketId(packetId))
		{
			return;
		}

		switch (pInGenericHeader->fgmc_command_)
		{
		case FGMCCommand::MCD_COMMAND_UNETINF:
			if (pInGenericHeader->fgmc_destination_id_[0] == '\0')
			{
				// fgmc command: upload network information
				cmdUnetinf(packetId);
			}
			break;

		case FGMCCommand::MCD_COMMAND_DNETINF:
			if (strncmp(pInGenericHeader->fgmc_destination_id_, uniqueId, SIZE_FGMC_DEST_ID) == 0)
			{
				// fgmc command: download network information
				FGMC_ReqDownloadNetInfoHeader* const pInCmdPointer =
					reinterpret_cast<FGMC_ReqDownloadNetInfoHeader*>(reinterpret_cast<uint32_t>(inputBufferAddress) + static_cast<uint32_t>(sizeof(FGMC_GenericHeader)));
				cmdDnetinf(pInCmdPointer, packetId);
			}
			break;

		case FGMCCommand::MCD_COMMAND_REBOOT:
			if (strncmp(pInGenericHeader->fgmc_destination_id_, uniqueId, SIZE_FGMC_DEST_ID) == 0)
			{
				// fgmc command: reboot
				cmdReboot(packetId);
			}
			break;

		case FGMCCommand::MCD_COMMAND_IDENTIFY:
			if (strncmp(pInGenericHeader->fgmc_destination_id_, uniqueId, SIZE_FGMC_DEST_ID) == 0)
			{
				// fgmc command: identify
				FGMC_ReqIdentify* pInCmdPointer =
					reinterpret_cast<FGMC_ReqIdentify*>(reinterpret_cast<uint32_t>(inputBufferAddress) + static_cast<uint32_t>(sizeof(FGMC_GenericHeader)));
				cmdIdentify(pInCmdPointer, packetId);
			}
			break;

		case FGMCCommand::MCD_COMMAND_GET_FIRMWARE_VERSION:
			if (strncmp(pInGenericHeader->fgmc_destination_id_, uniqueId, SIZE_FGMC_DEST_ID) == 0)
			{
				// fgmc command: get firmware version
				cmdGetFirmwareVersion(packetId);
			}
			break;

		case FGMCCommand::MCD_COMMAND_GET_SUPPORTED_COMMANDS:
			if (strncmp(pInGenericHeader->fgmc_destination_id_, uniqueId, SIZE_FGMC_DEST_ID) == 0)
			{
				// fgmc command: get supported commands
				cmdGetSupportedCommands(packetId);
			}
			break;

		default:
			// unknown command will be skipped
			break;
		}
	}
}

void FGMCProtocol::sendGenericHeader(uint8_t* tx_netbuf, FGMCCommand cmd, uint32_t length, uint32_t packetId, uint32_t segmentIndex, uint32_t segmentCount) noexcept
{
	FGMC_GenericHeader* pOutGenericHeader = reinterpret_cast<FGMC_GenericHeader*>(tx_netbuf);

	//-----------------------------------------------------------------------------------
	// Generic Multicast Header
	//-----------------------------------------------------------------------------------
	memcpy(pOutGenericHeader->fgmc_name_, FGMC_NAME, SIZE_FGMC_NAME);
	pOutGenericHeader->fgmc_length_ = length;
	pOutGenericHeader->fgmc_hw_type_id_ = fgmc_device_id_;
	// device unique fgmc destination id
	strncpy(pOutGenericHeader->fgmc_destination_id_, uniqueId, SIZE_FGMC_DEST_ID);
	pOutGenericHeader->fgmc_packet_id_ = packetId + 1;
	pOutGenericHeader->fgmc_segment_index_ = segmentIndex;
	pOutGenericHeader->fgmc_segment_count_ = segmentCount;
	pOutGenericHeader->fgmc_command_ = cmd;
	pOutGenericHeader->fgmc_command_version_ = FGMCCommandVersion::MCD_COMMAND_VERSION;
	pOutGenericHeader->fgmc_error_code_ = FGMCErrorCode::FGMC_ERROR_CODE_NORMAL;

	// Send Message
	(void)ifaceData[iface_id_].insertPacketId(pOutGenericHeader->fgmc_packet_id_);

	MulticastResponder::SendResponse(tx_netbuf, length);
}

void FGMCProtocol::cmdUnetinf(uint32_t inPacketId) noexcept
{
	FGMC_ResUploadNetInfoHeader* pOutCmdHeader = reinterpret_cast<FGMC_ResUploadNetInfoHeader*>(tx_netbuf_);
	(void)memset(pOutCmdHeader, 0x00, sizeof(FGMC_ResUploadNetInfoHeader));

	//-----------------------------------------------------------------------------------
	// The Upload Netinformation Structure
	//-----------------------------------------------------------------------------------

	// ip address type
	pOutCmdHeader->fgmc_ip_address_type_ = (reprap.GetNetwork().UsingDhcp(iface_id_)) ? 1 : 0;

	// mac address
	memcpy(pOutCmdHeader->fgmc_mac_address_, reprap.GetNetwork().GetMacAddress(iface_id_).bytes, 6);

	// IPv4
	const InterfaceData& ifData = ifaceData[iface_id_];
	const uint32_t ipaddress = ifData.configuredIpAddress.GetV4LittleEndian();
	(void)memcpy(&pOutCmdHeader->fgmc_ip_v4_static_address_[0], &ipaddress, SIZE_IP_V4);

	const uint32_t subnetmask = ifData.configuredNetmask.GetV4LittleEndian();
	(void)memcpy(&pOutCmdHeader->fgmc_ip_v4_static_netmask_[0], &subnetmask, SIZE_IP_V4);

	const uint32_t gateway = ifData.configuredGateway.GetV4LittleEndian();
	(void)memcpy(&pOutCmdHeader->fgmc_ip_v4_static_gateway_[0], &gateway, SIZE_IP_V4);

	pOutCmdHeader->fgmc_ip_v4_static_dns_[0] = 0;
	pOutCmdHeader->fgmc_ip_v4_static_dns_[1] = 0;
	pOutCmdHeader->fgmc_ip_v4_static_dns_[2] = 0;
	pOutCmdHeader->fgmc_ip_v4_static_dns_[3] = 0;

	// IPv6
	(void)memset(&pOutCmdHeader->fgmc_ip_v6_static_6_addr_[0], 0x00, SIZE_IP_V6);
	(void)memset(&pOutCmdHeader->fgmc_ip_v6_static_netmask_[0], 0x00, SIZE_IP_V6);
	(void)memset(&pOutCmdHeader->fgmc_ip_v6_static_gateway_[0], 0x00, SIZE_IP_V6);
	(void)memset(&pOutCmdHeader->fgmc_ip_v6_static_dns_[0], 0x00, SIZE_IP_V6);

	// IPv4 active
	const uint32_t ipaddressActive = reprap.GetNetwork().GetIPAddress(iface_id_).GetV4LittleEndian();
	(void)memcpy(&pOutCmdHeader->fgmc_ip_v4_address_[0], &ipaddressActive, SIZE_IP_V4);

	const uint32_t subnetmaskActive = reprap.GetNetwork().GetNetmask(iface_id_).GetV4LittleEndian();
	(void)memcpy(&pOutCmdHeader->fgmc_ip_v4_netmask_[0], &subnetmaskActive, SIZE_IP_V4);

	const uint32_t gatewayActive = reprap.GetNetwork().GetGateway(iface_id_).GetV4LittleEndian();
	(void)memcpy(&pOutCmdHeader->fgmc_ip_v4_gateway_[0], &gatewayActive, SIZE_IP_V4);

	pOutCmdHeader->fgmc_ip_v4_dns_[0] = 0;
	pOutCmdHeader->fgmc_ip_v4_dns_[1] = 0;
	pOutCmdHeader->fgmc_ip_v4_dns_[2] = 0;
	pOutCmdHeader->fgmc_ip_v4_dns_[3] = 0;

	// NOC-CODE
	(void)strncpy(pOutCmdHeader->fgmc_noc_code_, "DUET3" BOARD_SHORT_NAME, SIZE_NOC_CODE);

	// iTTL
	pOutCmdHeader->fgmc_ttl_ = 255;

	// Connection State
	pOutCmdHeader->fgmc_connection_state_ = 0;

	// Device Type
	(void)strncpy(pOutCmdHeader->fgmc_device_type_, BOARD_NAME, SIZE_DEVICE_TYPE);

	// Device Serial Number. This is 64 characters long. We split the 128-bit unique ID into four 32-bit words and print each as 10 decimal digits.
	const UniqueId& id = reprap.GetPlatform().GetUniqueId();
	SafeSnprintf(pOutCmdHeader->fgmc_serial_number_,
					sizeof(pOutCmdHeader->fgmc_serial_number_),
					"%010" PRIu32 "%010" PRIu32 "%010" PRIu32 "%010" PRIu32,
					id.GetDwords()[0], id.GetDwords()[1], id.GetDwords()[2], id.GetDwords()[3]);

	// Application Type
	pOutCmdHeader->fgmc_application_type_ = fgmc_application_type_;

	// Application Version
	pOutCmdHeader->fgmc_application_version_ = 0;

	// Application Version Revision
	pOutCmdHeader->fgmc_application_version_revision_ = 0;

	// Generic Info
	(void)memset(pOutCmdHeader->fgmc_generic_info_, 0x00, SIZE_GENERIC_INFO);

	// Device Name
	strncpy(pOutCmdHeader->fgmc_device_name_, reprap.GetName(), ARRAY_SIZE(pOutCmdHeader->fgmc_device_name_));

	//-----------------------------------------------------------------------------------
	// Generic Multicast Header
	//-----------------------------------------------------------------------------------
	sendGenericHeader(tx_netbuf_, FGMCCommand::MCD_COMMAND_UNETINF, sizeof(FGMC_ResUploadNetInfoHeader), inPacketId, 0, 1);
}

void FGMCProtocol::cmdDnetinf(FGMC_ReqDownloadNetInfoHeader* pInCmdHeader, uint32_t inPacketId) noexcept
{
	FGMC_ResDownloadNetInfoHeader* pOutCmdHeader = reinterpret_cast<FGMC_ResDownloadNetInfoHeader*>(tx_netbuf_);
	(void)memset(pOutCmdHeader, 0x00, sizeof(FGMC_ResDownloadNetInfoHeader));

	//-----------------------------------------------------------------------------------
	// The Download Netinformation Structure
	//-----------------------------------------------------------------------------------

	// set new network settings
	InterfaceData& ifData = ifaceData[iface_id_];
	const bool dhcpEnable = static_cast<bool>(pInCmdHeader->fgmc_ip_address_type_);
	if (dhcpEnable)
	{
		ifData.configuredIpAddress.SetNull();
	}
	else
	{
		ifData.configuredIpAddress.SetV4LittleEndian(LoadLEU32(pInCmdHeader->fgmc_ip_v4_static_address_));
	}

	ifData.configuredNetmask.SetV4LittleEndian(LoadLEU32(pInCmdHeader->fgmc_ip_v4_static_netmask_));
	ifData.configuredGateway.SetV4LittleEndian(LoadLEU32(pInCmdHeader->fgmc_ip_v4_static_gateway_));

	// set new device name
	// filter out " (X19)  / (X18)
	for (uint32_t i = 0; i < SIZE_DEVICE_NAME; i++)
	{
		if (pInCmdHeader->fgmc_device_name_[i] == '\0')
		{
			if (i > 6)
			{
				if ((pInCmdHeader->fgmc_device_name_[i - 6] == ' ') && (pInCmdHeader->fgmc_device_name_[i - 5] == '(') &&
					(pInCmdHeader->fgmc_device_name_[i - 4] == 'X') && (pInCmdHeader->fgmc_device_name_[i - 3] == '1') &&
					(pInCmdHeader->fgmc_device_name_[i - 1] == ')'))
				{
					for (uint32_t j = 1; j <= 6; j++)
					{
						pInCmdHeader->fgmc_device_name_[i - j] = '\0';
					}
				}
			}
			break;
		}
	}

	// Create a new network-override.g file
	FileStore *const fs = reprap.GetPlatform().OpenSysFile(NetworkConfigOverrideFileName, OpenMode::write);
	if (fs == nullptr)
	{
		reprap.GetPlatform().MessageF(ErrorMessage, "Unable to create file %s\n", NetworkConfigOverrideFileName);
	}
	else
	{
		String<StringLength256> buf;
		buf.printf(	"M550 P\"%s\"\n"
					"M552 P%u.%u.%u.%u\n"
					"M553 P%u.%u.%u.%u\n"
					"M554 P%u.%u.%u.%u\n",
					pInCmdHeader->fgmc_device_name_,
					ifData.configuredIpAddress.GetQuad(0), ifData.configuredIpAddress.GetQuad(1), ifData.configuredIpAddress.GetQuad(2), ifData.configuredIpAddress.GetQuad(3),
					ifData.configuredNetmask.GetQuad(0), ifData.configuredNetmask.GetQuad(1), ifData.configuredNetmask.GetQuad(2), ifData.configuredNetmask.GetQuad(3),
					ifData.configuredGateway.GetQuad(0), ifData.configuredGateway.GetQuad(1), ifData.configuredGateway.GetQuad(2), ifData.configuredGateway.GetQuad(3)
				  );

		if (!fs->Write(buf.c_str()))
		{
			reprap.GetPlatform().MessageF(ErrorMessage, "Unable to write file %s\n", NetworkConfigOverrideFileName);
		}
		if (!fs->Close())
		{
			reprap.GetPlatform().MessageF(ErrorMessage, "Unable to close file %s\n", NetworkConfigOverrideFileName);
		}
	}

	//-----------------------------------------------------------------------------------
	// Generic Multicast Header
	//-----------------------------------------------------------------------------------
	sendGenericHeader(tx_netbuf_, FGMCCommand::MCD_COMMAND_DNETINF, sizeof(FGMC_ResDownloadNetInfoHeader), inPacketId, 0, 1);
}

void FGMCProtocol::cmdReboot(uint32_t inPacketId) noexcept
{
	(void)memset(tx_netbuf_, 0x00, sizeof(FGMC_GenericHeader));

	//-----------------------------------------------------------------------------------
	// Generic Multicast Header
	//-----------------------------------------------------------------------------------
	sendGenericHeader(tx_netbuf_, FGMCCommand::MCD_COMMAND_REBOOT, sizeof(FGMC_GenericHeader), inPacketId, 0, 1);
	MulticastResponder::ScheduleReboot();
}

void FGMCProtocol::cmdIdentify(FGMC_ReqIdentify* pInCmdHeader, uint32_t inPacketId) noexcept
{
	FGMC_ResIdentify* pOutCmdHeader = reinterpret_cast<FGMC_ResIdentify*>(tx_netbuf_);
	(void)memset(pOutCmdHeader, 0x00, sizeof(FGMC_ResIdentify));

	switch (pInCmdHeader->identification_type_)
	{
	case IDENTIFY_DEFAULT:
		CanInterface::SetStatusLedIdentify(30);						// identify for about 30 seconds
		break;

	case IDENTIFY_ON:
		CanInterface::SetStatusLedIdentify(0);						// identify until cancelled
		break;

	case IDENTIFY_OFF:
	default:
		CanInterface::SetStatusLedNormal();							// cancel identify
		break;
	}

	//-----------------------------------------------------------------------------------
	// Generic Multicast Header
	//-----------------------------------------------------------------------------------
	sendGenericHeader(tx_netbuf_, FGMCCommand::MCD_COMMAND_IDENTIFY, sizeof(FGMC_ResIdentify), inPacketId, 0, 1);
}

void FGMCProtocol::cmdGetFirmwareVersion(uint32_t inPacketId) noexcept
{
	FGMC_ResGetFwVersion* pOutCmdHeader = reinterpret_cast<FGMC_ResGetFwVersion*>(tx_netbuf_);
	(void)memset(pOutCmdHeader, 0x00, sizeof(FGMC_ResGetFwVersion));

	//-----------------------------------------------------------------------------------
	// Get Firmware Version
	//-----------------------------------------------------------------------------------
	(void)strncpy(pOutCmdHeader->module_name_, BOARD_NAME, SIZE_DEVICE_TYPE);

	SafeSnprintf(pOutCmdHeader->module_version_, SIZE_MODULE_VERSION, "%s version %s", FIRMWARE_NAME, VERSION);

	//-----------------------------------------------------------------------------------
	// Generic Multicast Header
	//-----------------------------------------------------------------------------------
	sendGenericHeader(tx_netbuf_, FGMCCommand::MCD_COMMAND_GET_FIRMWARE_VERSION, sizeof(FGMC_ResGetFwVersion), inPacketId, 0, 1);
}

void FGMCProtocol::cmdGetSupportedCommands(uint32_t inPacketId) noexcept
{
	uint32_t segmentIndex = 0;
	uint32_t segmentCount = 0;

	FGMCCommand supportedCommands[] =
	{
		FGMCCommand::MCD_COMMAND_UNETINF,
		FGMCCommand::MCD_COMMAND_DNETINF,
		FGMCCommand::MCD_COMMAND_REBOOT,
		FGMCCommand::MCD_COMMAND_IDENTIFY,
		FGMCCommand::MCD_COMMAND_GET_FIRMWARE_VERSION,
		FGMCCommand::MCD_COMMAND_GET_SUPPORTED_COMMANDS,
	};

	for (uint8_t i = 0; i < ARRAY_SIZE(supportedCommands); i++)
	{
		FGMC_ResGetSupportedCommands* pOutCmdHeader = reinterpret_cast<FGMC_ResGetSupportedCommands*>(tx_netbuf_);
		(void)memset(pOutCmdHeader, 0x00, sizeof(FGMC_ResGetSupportedCommands));

		//-----------------------------------------------------------------------------------
		// Get Supported Commands
		//-----------------------------------------------------------------------------------
		pOutCmdHeader->cmd_ = static_cast<uint32_t>(supportedCommands[i]);
		pOutCmdHeader->cmd_version_ = 0;

		//-----------------------------------------------------------------------------------
		// Generic Multicast Header
		//-----------------------------------------------------------------------------------
		segmentIndex = static_cast<uint32_t>(i);
		segmentCount = static_cast<uint32_t>(ARRAY_SIZE(supportedCommands));
		sendGenericHeader(tx_netbuf_, FGMCCommand::MCD_COMMAND_GET_SUPPORTED_COMMANDS, sizeof(FGMC_ResGetSupportedCommands), inPacketId, segmentIndex, segmentCount);
	}
}

FGMCProtocol::InterfaceData::InterfaceData() noexcept
	: packetIdBuffer{0},
	  packetIdIndex(0)
{
}

// Insert packet ID into buffer, returning true if success, false if it was already there,
bool FGMCProtocol::InterfaceData::insertPacketId(uint32_t packetId) noexcept
{
	for (uint32_t i = 0; i < InterfaceData::ringBufferSize; i++)
	{
		if (packetId == packetIdBuffer[i])
		{
			return false;
		}
	}

	packetIdBuffer[packetIdIndex] = packetId;
	packetIdIndex++;
	if (packetIdIndex >= InterfaceData::ringBufferSize)
	{
		packetIdIndex = 0;
	}
	return true;
}

#endif

// End
