// FGMC protocol handler

#ifndef FGMC_PROTOCOL_H
#define FGMC_PROTOCOL_H

#include <RepRapFirmware.h>

#if SUPPORT_MULTICAST_DISCOVERY

#include "fgmc_header.h"

/// array size definitions
#define SIZE_EEPROM_MAC_ADDRESS 6
#define SIZE_EEPROM_PRODUCT_KEY 15
#define SIZE_EEPROM_SERIAL_NUMBER 5
#define SIZE_EEPROM_TEST_NUMBER 9
#define SIZE_EEPROM_NOC_CODE 50

/// fgmc port
//#define FGMC_PORT 10002
//#define FGMC_IP IP_BYTES2ADDR(239, 255, 2, 3)  // Broadcast-IP

constexpr size_t IP_MAX_IFACES = 1;

class FGMCProtocol
{
public:
	/// constructor
	FGMCProtocol() noexcept;

#if 0
	/// registers engp router component
	/// \param router reference to engp router component
	void registerEngpRouter(com::protocols::engp::ENGPRouter& router0, com::protocols::engp::ENGPRouter& router1)
	{
		this->engp_router_[router0.GetIFaceId()] = &router0;
		this->engp_router_[router0.GetVirtIFaceId()] = &router0;
		this->engp_router_[router1.GetIFaceId()] = &router1;
		this->engp_router_[router1.GetVirtIFaceId()] = &router1;
	}

	/// registers interface self identification
	/// \param ledManager reference to led manager component
	void registerLedManager(hal::ISelfIdentification& ledManager) { this->led_manager_ = &ledManager; }

	/// registers application information component
	/// \param appInfo reference to led manager component
	void registerApplikationInformation(bmc::services::ApplicationInformation& appInfo) { this->app_info_ = &appInfo; }

	/// registers parameter management
	/// \param parameterManagement reference to led parameter management component
	void registerParameterManagement(interfaces::IParameterManagement& parameterManagement) { this->parameter_management_ = &parameterManagement; }
#endif

	/// this function initializes fgmc protocoll class
	void init() noexcept;

	/// this handles a fgmc frame
	/// \param nConn connection
	/// \param inputBufferAddress fgmc request frame
	/// \param rxLength receive frame length
	void handleStream(unsigned int iFaceId, uint8_t* inputBufferAddress, uint32_t rxLength) noexcept;

	/// this functions sends fgmc frame
	/// \param pOutPointer sciopta network buffer
	/// \param cmd cmd
	/// \param length length
	/// \param packetId packedId
	/// \param segmentIndex segmentIndex
	/// \param segmentCount segmentCount
	void sendGenericHeader(char* tx_netbuf, FGMCCommand cmd, uint32_t length, uint32_t packetId, uint32_t segmentIndex, uint32_t segmentCount) noexcept;

	/// fgmc command "upload network informations"
	/// \param inPacketId packedId
	void cmdUnetinf(uint32_t inPacketId) noexcept;

	/// fgmc command "download network information"
	/// \param pInCmdHeader request header network information header
	/// \param inPacketId packedId
	void cmdDnetinf(FGMC_ReqDownloadNetInfoHeader* pInCmdHeader, uint32_t inPacketId) noexcept;

	/// fgmc command "reboot"
	/// \param inPacketId packedId
	void cmdReboot(uint32_t inPacketId) noexcept;

	/// fgmc command "identify"
	/// \param pInCmdHeader request header identify
	/// \param inPacketId packedId
	void cmdIdentify(FGMC_ReqIdentify* pInCmdHeader, uint32_t inPacketId) noexcept;

	/// fgmc command "get firmware version"
	/// \param inPacketId packedId
	void cmdGetFirmwareVersion(uint32_t inPacketId) noexcept;

	/// fgmc command "get supported commands"
	/// \param inPacketId packedId
	void cmdGetSupportedCommands(uint32_t inPacketId) noexcept;

	/// insert used packet-id
	/// \param packetId packedId
	void insertPacketId(uint32_t packetId) noexcept;

	/// check if packet id is already used
	/// \param packetId packedId
	bool isPacketInBuffer(uint32_t packetId) noexcept;

	private:
	void macToString(uint32_t interface) noexcept;

	// connection data pointer
	unsigned int iface_id_;

	// product key from eeprom
	char eeprom_product_key_[SIZE_EEPROM_PRODUCT_KEY];
	char unique_id_[IP_MAX_IFACES][SIZE_FGMC_DEST_ID];
	char eeprom_noc_code_[SIZE_EEPROM_NOC_CODE];

	// variables will be initialized at init phase
	FGMCHwTypeId fgmc_device_id_;
	uint32_t fgmc_application_type_;
	char fgmc_device_type_[SIZE_DEVICE_TYPE];

	char tx_netbuf_[SIZE_FGMC_RES_MAX];

	/// Object that holds communication related parameter.
#if 0
	bmc::services::ApplicationInformation* app_info_;
	hal::ISelfIdentification* led_manager_;
	com::protocols::engp::ENGPRouter* engp_router_[IP_MAX_IFACES];
	interfaces::IParameterManagement* parameter_management_;
#endif

	static constexpr uint32_t ringBufferSize = 10;
	uint32_t packetIdBuffer[IP_MAX_IFACES][ringBufferSize];
	uint32_t packetIdIndex[IP_MAX_IFACES];
};

#endif	// SUPPORT_MULTICAST_DISCOVERY

#endif // FGMC_PROTOCOL_H
