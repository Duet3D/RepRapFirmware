// Implementation of MulticastDiscovery protocol

#include "fgmc_protocol.h"

#if SUPPORT_MULTICAST_DISCOVERY

#include <Platform/RepRap.h>
#include <Networking/Network.h>
#include <Networking/MulticastDiscovery/MulticastResponder.h>
#include <CAN/CanInterface.h>
#include <Version.h>
#include <cstring>

// Disable gcc warning about strncpy not being able to include a terminating null.
// We don't need a terminating null when filling in response fields (if we did then we would use SafeStrcpy instead).
#pragma GCC diagnostic ignored "-Wstringop-truncation"

FGMCProtocol::FGMCProtocol() noexcept
    : iface_id_(0),
      eeprom_product_key_{0},
      unique_id_{0},
      eeprom_noc_code_{0},
      fgmc_device_id_(FGMCHwTypeId::FGMC_DEVICE_ID_ZERO),
      fgmc_application_type_(0),
      tx_netbuf_{0},
      packetIdBuffer{0},
      packetIdIndex{0}
{
}

// Build a unique ID from the MAC address
void FGMCProtocol::macToString(uint32_t interface) noexcept
{
  for (uint32_t i = 0; i < SIZE_FGMC_DEST_ID; i++)
  {
    unique_id_[interface][i] = '\0';
  }

  const MacAddress& macAddress = reprap.GetNetwork().GetMacAddress(interface);
  for (uint32_t i = 0; i < 6; i++)
  {
    const uint8_t byte = macAddress.bytes[i];
    uint8_t lowValue = (byte & 0x0Fu);
    if (lowValue < 0xA)
    {
      lowValue = lowValue + 48;
    }
    else
    {
      lowValue = lowValue + 55;
    }

    uint8_t highValue = (byte >> 4u) & 0x0Fu;
    if (highValue < 0xA)
    {
      highValue = highValue + 48;
    }
    else
    {
      highValue = highValue + 55;
    }
    unique_id_[interface][i * 2] = static_cast<char>(lowValue);
    unique_id_[interface][(i * 2) + 1] = static_cast<char>(highValue);
  }
}

void FGMCProtocol::init() noexcept
{
#if 0
	char deviceName[SIZE_DEVICE_NAME] = {0};

	// Read eeprom data
	ret = deviceInfo.GetFestoNocCode(&eeprom_noc_code_[0], SIZE_EEPROM_NOC_CODE);
	if (ret != base::ReturnCode::kRcSuccess) {
		retTotal = ret;
	}
	ret = deviceInfo.GetFestoProductKey(&eeprom_product_key_[0], SIZE_EEPROM_PRODUCT_KEY);
	if (ret != base::ReturnCode::kRcSuccess) {
		retTotal = ret;
	}
	fgmc_application_type_ = deviceInfo.GetFestoPartNumber();
	(void)strncpy(&deviceName[0], &eeprom_noc_code_[0], SIZE_EEPROM_NOC_CODE);

	// check eeprom values available
	if ((eeprom_noc_code_[0] == '\0')) {  // || (eeprom_product_key_[0] == '\0')
		// EEPROMs not described
		eeprom_product_key_[0] = '1';
		eeprom_product_key_[1] = '\0';
		(void)memcpy(&eeprom_noc_code_[0], &INVALID_EEPROM_DATA_STRING, INVALID_EEPROM_DATA_STRING_LEN);
		(void)memcpy(&deviceName[0], &INVALID_EEPROM_DATA_STRING, INVALID_EEPROM_DATA_STRING_LEN);
	}

#endif

	for (uint32_t i = 0; i < IP_MAX_IFACES; i++)
	{
		macToString(i);
	}

	fgmc_device_id_ = FGMCHwTypeId::FGMC_DEVICE_ID_DUET3;
}

void FGMCProtocol::handleStream(unsigned int iFaceId, uint8_t* inputBufferAddress, uint32_t rxLength) noexcept
{
	// backup sciopta connection handle
	// if a null pointer received => plattform should execute a exception
	this->iface_id_ = iFaceId;

	if (rxLength >= static_cast<uint32_t>(sizeof(FGMC_GenericHeader)))
	{
		FGMC_GenericHeader* pInGenericHeader = reinterpret_cast<FGMC_GenericHeader*>(inputBufferAddress);

		// read incoming packetid
		uint32_t packetId = pInGenericHeader->fgmc_packet_id_;

		if (isPacketInBuffer(packetId))
		{
			return;  // TODO: Filter out iFace 1, if iFace 0 is active (PACKET-ID) ?
		}

		insertPacketId(packetId);

		switch (pInGenericHeader->fgmc_command_)
		{
		case FGMCCommand::MCD_COMMAND_UNETINF:
			if (pInGenericHeader->fgmc_destination_id_[0] == '\0')
			{
				// fgmc command: upload network information
				cmdUnetinf(packetId);
			}
			break;

		#if 0	// not supported for now
		case FGMCCommand::MCD_COMMAND_DNETINF:
			if (strncmp(pInGenericHeader->fgmc_destination_id_, unique_id_[iface_id_], SIZE_FGMC_DEST_ID) == 0)
			{
				// fgmc command: download network information
				FGMC_ReqDownloadNetInfoHeader* const pInCmdPointer =
					reinterpret_cast<FGMC_ReqDownloadNetInfoHeader*>(reinterpret_cast<uint32_t>(inputBufferAddress) + static_cast<uint32_t>(sizeof(FGMC_GenericHeader)));
				cmdDnetinf(pInCmdPointer, packetId);
			}
			break;
		#endif

		case FGMCCommand::MCD_COMMAND_REBOOT:
			if (strncmp(pInGenericHeader->fgmc_destination_id_, unique_id_[iface_id_], SIZE_FGMC_DEST_ID) == 0)
			{
				// fgmc command: reboot
				cmdReboot(packetId);
			}
			break;

		case FGMCCommand::MCD_COMMAND_IDENTIFY:
			if (strncmp(pInGenericHeader->fgmc_destination_id_, unique_id_[iface_id_], SIZE_FGMC_DEST_ID) == 0)
			{
				// fgmc command: identify
				FGMC_ReqIdentify* pInCmdPointer =
					reinterpret_cast<FGMC_ReqIdentify*>(reinterpret_cast<uint32_t>(inputBufferAddress) + static_cast<uint32_t>(sizeof(FGMC_GenericHeader)));
				cmdIdentify(pInCmdPointer, packetId);
			}
			break;

		case FGMCCommand::MCD_COMMAND_GET_FIRMWARE_VERSION:
			if (strncmp(pInGenericHeader->fgmc_destination_id_, unique_id_[iface_id_], SIZE_FGMC_DEST_ID) == 0)
			{
				// fgmc command: get firmware version
				cmdGetFirmwareVersion(packetId);
			}
			break;

		case FGMCCommand::MCD_COMMAND_GET_SUPPORTED_COMMANDS:
			if (strncmp(pInGenericHeader->fgmc_destination_id_, unique_id_[iface_id_], SIZE_FGMC_DEST_ID) == 0)
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
	strncpy(pOutGenericHeader->fgmc_destination_id_, unique_id_[iface_id_], SIZE_FGMC_DEST_ID);
	pOutGenericHeader->fgmc_packet_id_ = packetId + 1;
	pOutGenericHeader->fgmc_segment_index_ = segmentIndex;
	pOutGenericHeader->fgmc_segment_count_ = segmentCount;
	pOutGenericHeader->fgmc_command_ = cmd;
	pOutGenericHeader->fgmc_command_version_ = FGMCCommandVersion::MCD_COMMAND_VERSION;
	pOutGenericHeader->fgmc_error_code_ = FGMCErrorCode::FGMC_ERROR_CODE_NORMAL;

	// Send Message
	insertPacketId(pOutGenericHeader->fgmc_packet_id_);

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
	const bool dhcpEnable = reprap.GetNetwork().UsingDhcp(iface_id_);
	if (dhcpEnable)
	{
		// dynamic
		pOutCmdHeader->fgmc_ip_address_type_ = 1;
	}
	else
	{
		// static
		pOutCmdHeader->fgmc_ip_address_type_ = 0;
	}

	// mac address
	const MacAddress& macAddress = reprap.GetNetwork().GetMacAddress(iface_id_);
	memcpy(pOutCmdHeader->fgmc_mac_address_, macAddress.bytes, 6);

	// IPv4
	const uint32_t ipaddress = reprap.GetNetwork().GetIPAddress(iface_id_).GetV4LittleEndian();
	(void)memcpy(&pOutCmdHeader->fgmc_ip_v4_static_address_[0], &ipaddress, SIZE_IP_V4);

	const uint32_t subnetmask = reprap.GetNetwork().GetNetmask(iface_id_).GetV4LittleEndian();
	(void)memcpy(&pOutCmdHeader->fgmc_ip_v4_static_netmask_[0], &subnetmask, SIZE_IP_V4);

	const uint32_t gateway = reprap.GetNetwork().GetGateway(iface_id_).GetV4LittleEndian();
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
	const uint32_t ipaddressActive = ipaddress;
	(void)memcpy(&pOutCmdHeader->fgmc_ip_v4_address_[0], &ipaddressActive, SIZE_IP_V4);

	const uint32_t subnetmaskActive = subnetmask;
	(void)memcpy(&pOutCmdHeader->fgmc_ip_v4_netmask_[0], &subnetmaskActive, SIZE_IP_V4);

	const uint32_t gatewayActive = gateway;
	(void)memcpy(&pOutCmdHeader->fgmc_ip_v4_gateway_[0], &gatewayActive, SIZE_IP_V4);

	pOutCmdHeader->fgmc_ip_v4_dns_[0] = 0;
	pOutCmdHeader->fgmc_ip_v4_dns_[1] = 0;
	pOutCmdHeader->fgmc_ip_v4_dns_[2] = 0;
	pOutCmdHeader->fgmc_ip_v4_dns_[3] = 0;

	// NOC-CODE
//	(void)strncpy(pOutCmdHeader->fgmc_noc_code_, &eeprom_noc_code_[0], SIZE_EEPROM_NOC_CODE);

	// iTTL
	pOutCmdHeader->fgmc_ttl_ = 255;

	// Connection State
	pOutCmdHeader->fgmc_connection_state_ = 0;

	// Device Type
	(void)strncpy(pOutCmdHeader->fgmc_device_type_, BOARD_NAME, SIZE_DEVICE_TYPE);

	// Device Serial Number
	(void)strncpy(pOutCmdHeader->fgmc_serial_number_, &eeprom_product_key_[0], SIZE_SERIAL_NUMBER);

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

#if 0	// not supported for now
void FGMCProtocol::cmdDnetinf(FGMC_ReqDownloadNetInfoHeader* pInCmdHeader, uint32_t inPacketId)
{
	FGMC_ResDownloadNetInfoHeader* pOutCmdHeader = reinterpret_cast<FGMC_ResDownloadNetInfoHeader*>(tx_netbuf_);
	(void)memset(pOutCmdHeader, 0x00, sizeof(FGMC_ResDownloadNetInfoHeader));

	//-----------------------------------------------------------------------------------
	// The Download Netinformation Structure
	//-----------------------------------------------------------------------------------

	bool dhcpEnable = false;
	uint32_t ipaddress = 0;
	uint32_t subnetmask = 0;
	uint32_t gateway = 0;

	// set new network settings
	dhcpEnable = static_cast<bool>(pInCmdHeader->fgmc_ip_address_type_);
	engp_router_[iface_id_]->SetDhcpenable(dhcpEnable);
	(void)memcpy(&ipaddress, &pInCmdHeader->fgmc_ip_v4_static_address_[0], SIZE_IP_V4);
	engp_router_[iface_id_]->SetIpaddress(ntohl(ipaddress));
	(void)memcpy(&subnetmask, &pInCmdHeader->fgmc_ip_v4_static_netmask_[0], SIZE_IP_V4);
	engp_router_[iface_id_]->SetSubnetmask(ntohl(subnetmask));
	(void)memcpy(&gateway, &pInCmdHeader->fgmc_ip_v4_static_gateway_[0], SIZE_IP_V4);
	engp_router_[iface_id_]->SetGateway(ntohl(gateway));

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
	app_info_->setDeviceName(&pInCmdHeader->fgmc_device_name_[0], SIZE_DEVICE_NAME);

	//-----------------------------------------------------------------------------------
	// Generic Multicast Header
	//-----------------------------------------------------------------------------------
	sendGenericHeader(tx_netbuf_, FGMCCommand::MCD_COMMAND_DNETINF, sizeof(FGMC_ResDownloadNetInfoHeader), inPacketId, 0, 1);
}
#endif

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
#if 0	// not supported for now
		FGMCCommand::MCD_COMMAND_DNETINF,
#endif
		FGMCCommand::MCD_COMMAND_IDENTIFY,
		FGMCCommand::MCD_COMMAND_GET_FIRMWARE_VERSION,
		FGMCCommand::MCD_COMMAND_GET_SUPPORTED_COMMANDS
	};

	for (uint8_t i = 0; i < (sizeof(supportedCommands) / sizeof(FGMCCommand)); i++)
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
		segmentCount = static_cast<uint32_t>(sizeof(supportedCommands)) / sizeof(FGMCCommand);
		sendGenericHeader(tx_netbuf_, FGMCCommand::MCD_COMMAND_GET_SUPPORTED_COMMANDS, sizeof(FGMC_ResGetSupportedCommands), inPacketId, segmentIndex, segmentCount);
	}
}

bool FGMCProtocol::isPacketInBuffer(uint32_t packetId) noexcept
{
	for (uint32_t i = 0; i < ringBufferSize; i++)
	{
		if (packetId == packetIdBuffer[iface_id_][i])
		{
			return true;
		}
	}
	return false;
}

void FGMCProtocol::insertPacketId(uint32_t packetId) noexcept
{
	if (isPacketInBuffer(packetId))
	{
		return;
	}
	packetIdBuffer[iface_id_][packetIdIndex[iface_id_]] = packetId;
	packetIdIndex[iface_id_]++;
	if (packetIdIndex[iface_id_] >= ringBufferSize)
	{
		packetIdIndex[iface_id_] = 0;
	}
}

#endif

// End
