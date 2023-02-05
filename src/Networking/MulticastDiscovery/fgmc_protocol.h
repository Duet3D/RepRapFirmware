// FGMC protocol handler

#ifndef FGMC_PROTOCOL_H
#define FGMC_PROTOCOL_H

#include <RepRapFirmware.h>
#include <General/IPAddress.h>

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

	/// this function initializes fgmc protocoll class
	void init() noexcept;

	/// this handles a fgmc frame
	/// \param nConn connection
	/// \param inputBufferAddress fgmc request frame
	/// \param rxLength receive frame length
	void handleStream(unsigned int iFaceId, const uint8_t* inputBufferAddress, uint32_t rxLength) noexcept;

private:
	/// this functions sends fgmc frame
	/// \param pOutPointer sciopta network buffer
	/// \param cmd cmd
	/// \param length length
	/// \param packetId packedId
	/// \param segmentIndex segmentIndex
	/// \param segmentCount segmentCount
	void sendGenericHeader(uint8_t* tx_netbuf, FGMCCommand cmd, uint32_t length, uint32_t packetId, uint32_t segmentIndex, uint32_t segmentCount) noexcept;

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

	/// Build the unique ID
	void BuildUniqueId() noexcept;

	// connection data pointer
	unsigned int iface_id_;

	// variables will be initialized at init phase
	FGMCHwTypeId fgmc_device_id_;
	uint32_t fgmc_application_type_;

	// struct to hold per-interface data
	struct InterfaceData
	{
		IPAddress configuredIpAddress, configuredNetmask, configuredGateway;
		static constexpr uint32_t ringBufferSize = 10;
		uint32_t packetIdBuffer[ringBufferSize];
		uint32_t packetIdIndex;

		InterfaceData() noexcept;

		/// insert used packet-id
		/// \param packetId packedId
		/// \return true if inserted, false if it was already used
		bool insertPacketId(uint32_t packetId) noexcept;
	};


	char uniqueId[SIZE_FGMC_DEST_ID];
	InterfaceData ifaceData[IP_MAX_IFACES];
	uint8_t tx_netbuf_[SIZE_FGMC_RES_MAX];
};

#endif	// SUPPORT_MULTICAST_DISCOVERY

#endif // FGMC_PROTOCOL_H
