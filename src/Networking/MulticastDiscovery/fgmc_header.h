// Multicast discovery protocol and associated constants

#ifndef FGMC_HEADER_H
#define FGMC_HEADER_H

/// fgmc device id for platform family cmmt
enum class FGMCHwTypeId : uint32_t { FGMC_DEVICE_ID_ZERO = 0, FGMC_DEVICE_ID_CMMT_AS = 23, FGMC_DEVICE_ID_CMMT_ST = 33, FGMC_DEVICE_ID_DUET3 = 41 };

/// fgmc error codes
enum class FGMCErrorCode : int16_t
{
  FGMC_ERROR_CODE_NORMAL = 0,
  FGMC_ERROR_CODE_CMD_NOT_KNOWN = -1,
  FGMC_ERROR_CODE_CMD_VERSION_NOT_SUPPORTED = -2,
  FGMC_ERROR_CODE_CMD_FAILED = -3,
  FGMC_ERROR_CODE_CMD_SEGMENT_INDEX_MISS = -4
};

/// fgmc commands
enum class FGMCCommand : int32_t
{
  MCD_COMMAND_UNKNOWN = 0,
  MCD_COMMAND_UNETINF = 1,
  MCD_COMMAND_DNETINF = 2,
  MCD_COMMAND_DNETINF_REBOOT = 3,
  MCD_COMMAND_REBOOT = 4,
  MCD_COMMAND_UPDATE_FIRMWARE = 5,
  MCD_COMMAND_IDENTIFY = 6,
  MCD_COMMAND_GET_VERSIONS = 7,
  MCD_COMMAND_READFILE = 8,
  MCD_COMMAND_GET_FIRMWARE_VERSION = 9,
  MCD_COMMAND_GET_SUPPORTED_COMMANDS = 10
};

/// fgmc command version
enum class FGMCCommandVersion : uint16_t { MCD_COMMAND_VERSION = 0 };

//
constexpr const char * INVALID_EEPROM_DATA_STRING = "invalid eeprom data";

/// array size definitions
constexpr const char *FGMC_NAME = "FESTOMULTICAST";
constexpr size_t SIZE_FGMC_NAME = strlen(FGMC_NAME);
constexpr size_t SIZE_FGMC_DEST_ID = 16;

/// fgmc protocol header
struct __attribute__((packed)) FGMC_GenericHeader
{
	char fgmc_name_[SIZE_FGMC_NAME];
	uint32_t fgmc_length_;
	FGMCHwTypeId fgmc_hw_type_id_;
	char fgmc_destination_id_[SIZE_FGMC_DEST_ID];
	uint32_t fgmc_packet_id_;
	uint32_t fgmc_segment_index_;
	uint32_t fgmc_segment_count_;
	FGMCCommand fgmc_command_;
	FGMCCommandVersion fgmc_command_version_;
	FGMCErrorCode fgmc_error_code_;
};

/// array size definitions
constexpr size_t SIZE_MAC_ADDRESS = 6;
constexpr size_t SIZE_IP_V4 = 4;
constexpr size_t SIZE_IP_V6 = 16;
constexpr size_t SIZE_DEVICE_TYPE = 12;
constexpr size_t SIZE_NOC_CODE = 64;
constexpr size_t SIZE_SERIAL_NUMBER = 64;
constexpr size_t SIZE_GENERIC_INFO = 12;
constexpr size_t SIZE_DEVICE_NAME = 128;

/// fgmc command response header upload network informations
struct __attribute__((packed)) FGMC_ResUploadNetInfoHeader
{
	FGMC_GenericHeader fgmc_header_;
	char fgmc_mac_address_[SIZE_MAC_ADDRESS];
	uint32_t fgmc_ip_address_type_;
	uint8_t fgmc_ip_v4_static_address_[SIZE_IP_V4];
	uint8_t fgmc_ip_v4_static_netmask_[SIZE_IP_V4];
	uint8_t fgmc_ip_v4_static_gateway_[SIZE_IP_V4];
	uint8_t fgmc_ip_v4_static_dns_[SIZE_IP_V4];
	uint8_t fgmc_ip_v6_static_6_addr_[SIZE_IP_V6];
	uint8_t fgmc_ip_v6_static_netmask_[SIZE_IP_V6];
	uint8_t fgmc_ip_v6_static_gateway_[SIZE_IP_V6];
	uint8_t fgmc_ip_v6_static_dns_[SIZE_IP_V6];
	uint8_t fgmc_ip_v4_address_[SIZE_IP_V4];
	uint8_t fgmc_ip_v4_netmask_[SIZE_IP_V4];
	uint8_t fgmc_ip_v4_gateway_[SIZE_IP_V4];
	uint8_t fgmc_ip_v4_dns_[SIZE_IP_V4];
	char fgmc_noc_code_[SIZE_NOC_CODE];
	int32_t fgmc_ttl_;
	uint8_t fgmc_connection_state_;
	char fgmc_device_type_[SIZE_DEVICE_TYPE];
	char fgmc_serial_number_[SIZE_SERIAL_NUMBER];
	uint32_t fgmc_application_type_;
	uint32_t fgmc_application_version_;
	uint32_t fgmc_application_version_revision_;
	uint8_t fgmc_generic_info_[SIZE_GENERIC_INFO];
	char fgmc_device_name_[SIZE_DEVICE_NAME];
};

/// array size definitions
constexpr size_t SIZE_PASSWORD = 12;

/// fgmc command request header "download network informations"
struct __attribute__((packed)) FGMC_ReqDownloadNetInfoHeader
{
	uint32_t fgmc_ip_address_type_;
	uint8_t fgmc_ip_v4_static_address_[SIZE_IP_V4];
	uint8_t fgmc_ip_v4_static_netmask_[SIZE_IP_V4];
	uint8_t fgmc_ip_v4_static_gateway_[SIZE_IP_V4];
	uint8_t fgmc_ip_v4_static_dns_[SIZE_IP_V4];
	uint8_t fgmc_ip_v6_static_6_addr_[SIZE_IP_V6];
	uint8_t fgmc_ip_v6_static_netmask_[SIZE_IP_V6];
	uint8_t fgmc_ip_v6_static_gateway_[SIZE_IP_V6];
	uint8_t fgmc_ip_v6_static_dns_[SIZE_IP_V6];
	int32_t fgmc_ttl_;
	char fgmc_device_name_[SIZE_DEVICE_NAME];
	char fgmc_password_[SIZE_PASSWORD];
};

/// fgmc command response header "download network informations"
struct __attribute__((packed)) FGMC_ResDownloadNetInfoHeader
{
  FGMC_GenericHeader fgmc_header_;
};

/// array size definitions
constexpr uint32_t IDENTIFY_DEFAULT = 0;
constexpr uint32_t IDENTIFY_ON = 1;
constexpr uint32_t IDENTIFY_OFF = 2;

/// fgmc command request header "identify"
struct __attribute__((packed)) FGMC_ReqIdentify
{
	int32_t identification_type_;
	int32_t reserved_;
};

/// fgmc command response header "identify"
struct __attribute__((packed)) FGMC_ResIdentify
{
  FGMC_GenericHeader fgmc_header_;
};

/// array size definitions
constexpr size_t SIZE_MODULE_NAME = 192;
constexpr size_t SIZE_MODULE_VERSION = 64;

/// fgmc command response header "get firmware version"
struct __attribute__((packed)) FGMC_ResGetFwVersion
{
	FGMC_GenericHeader fgmc_header_;
	char module_name_[SIZE_MODULE_NAME];
	char module_version_[SIZE_MODULE_VERSION];
};

/// fgmc command response header "get supported commands"
struct __attribute__((packed)) FGMC_ResGetSupportedCommands
{
	FGMC_GenericHeader fgmc_header_;
	uint32_t cmd_;
	uint16_t cmd_version_;
};

constexpr size_t SIZE_FGMC_RES_MAX = 461;		// 461 Bytes for FGMC_ResUploadNetInfoHeader

static_assert(SIZE_FGMC_RES_MAX >= sizeof(FGMC_ResIdentify));
static_assert(SIZE_FGMC_RES_MAX >= sizeof(FGMC_ResUploadNetInfoHeader));
static_assert(SIZE_FGMC_RES_MAX >= sizeof(FGMC_ResDownloadNetInfoHeader));
static_assert(SIZE_FGMC_RES_MAX >= sizeof(FGMC_ResGetFwVersion));
static_assert(SIZE_FGMC_RES_MAX >= sizeof(FGMC_ResGetSupportedCommands));

#endif // #ifndef FGMC_HEADER_H
