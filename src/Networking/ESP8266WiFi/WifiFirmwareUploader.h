/*
 * EspFirmwareUpload.h
 *
 *  Created on: 15 Apr 2016
 *      Author: David
 */

#ifndef SRC_SAME70_WIFIFIRMWAREUPLOADER_H_
#define SRC_SAME70_WIFIFIRMWAREUPLOADER_H_

#include "RepRapFirmware.h"

#if HAS_WIFI_NETWORKING

class WiFiInterface;

class WifiFirmwareUploader
{
public:
	WifiFirmwareUploader(AsyncSerial& port, WiFiInterface &iface) noexcept;
	bool IsReady() const noexcept;
	void SendUpdateFile(const char *file, uint32_t address) noexcept;
	void Spin() noexcept;

	// Offsets in flash memory
	static const uint32_t FirmwareAddress = 0x00000000;

private:
	static const uint32_t defaultTimeout = 500;					// default timeout in milliseconds
	static const uint32_t syncTimeout = 1000;
	static const unsigned int retriesPerBaudRate = 9;
	static const unsigned int retriesPerReset = 3;
	static const uint32_t connectAttemptInterval = 50;
	static const uint32_t resetDelay = 500;
	static const uint32_t blockWriteInterval = 15;				// 15ms is long enough, 10ms is mostly too short
	static const uint32_t blockWriteTimeout = 200;
	static const uint32_t eraseTimeout = 15000;					// increased from 12 to 15 seconds because Roland's board was timing out
	static const unsigned int percentToReportIncrement = 5;		// how often we report % complete

	static const uint32_t esp8266systemParametersAddress = 0x3FE000;	// the address of the system + user parameter area that needs to be cleared when changing SDK version
	static const uint32_t esp8266systemParametersSize = 0x2000;			// the size of the system + user parameter area

	// Return codes
	// *** This list must be kept in step with the corresponding messages! ***
	enum class EspUploadResult : uint8_t
	{
		success = 0,

		// The following are status codes return by the ESP
		invalidMessage = 0x05,
		failedToAct = 0x06,
		invalidCrc = 0x07,
		flashWriteError = 0x08,
		flashReadError = 0x09,
		deflateError = 0x0b,

		// The remainder are our own status codes
		timeout = 0x10,
		connect,
		badReply,
		fileRead,
		emptyFile,
		respHeader,
		slipFrame,
		slipState,
		slipData,
	};

	enum class UploadState
	{
		idle,
		resetting,
		connecting,
		erasing1,
		erasing2,
		uploading,
		done
	};

	// Type of ESP. We need to identify ESP8266, the original ESP32 and ESP3232S2 or later devices.
	// These three classes of device need slightly different flash commands.
	enum ESPType : uint8_t
	{
		unknown = 0,
		ESP8266,
		ESP32,
		ESP32_PLUS
	};

	void MessageF(const char *fmt, ...) noexcept;
	int ReadByte(uint8_t& data, bool slipDecode) noexcept;
	void WriteByteRaw(uint8_t b) noexcept;
	void WriteByteSlip(uint8_t b) noexcept;
	void flushInput() noexcept;
	EspUploadResult readPacket(uint8_t op, uint32_t *valp, size_t& bodyLen, uint32_t *status, uint32_t msTimeout) noexcept;
	void writePacket(const uint8_t *data, size_t len) noexcept;
	void writePacketRaw(const uint8_t *buf, size_t len) noexcept;
	void writePacket(const uint8_t *hdr, size_t hdrLen, const uint8_t *data, size_t dataLen) noexcept;
	void writePacketRaw(const uint8_t *hdr, size_t hdrLen, const uint8_t *data, size_t dataLen) noexcept;
	void sendCommand(uint8_t op, uint32_t checkVal, const uint8_t *data, size_t dataLen) noexcept;
	EspUploadResult doCommand(uint8_t op, const uint8_t *data, size_t dataLen, uint32_t checkVal, uint32_t *valp, uint32_t msTimeout) noexcept;
	EspUploadResult Sync(uint16_t timeout) noexcept;
	EspUploadResult flashBegin(uint32_t offset, uint32_t size) noexcept;
	EspUploadResult flashFinish(bool reboot) noexcept;
	EspUploadResult flashSpiSetParameters(uint32_t size) noexcept;
	EspUploadResult flashSpiAttach() noexcept;
	static uint16_t checksum(const uint8_t *data, uint16_t dataLen, uint16_t cksum) noexcept;
	EspUploadResult flashWriteBlock(uint16_t flashParmVal, uint16_t flashParmMask) noexcept;
	EspUploadResult DoErase(uint32_t address, uint32_t size) noexcept;
	void Identify() noexcept;

	AsyncSerial& uploadPort;
	WiFiInterface& interface;
	FileStore *uploadFile;
	FilePosition fileSize;
	uint32_t uploadAddress;
	uint32_t uploadBlockNumber;
	unsigned int uploadNextPercentToReport;
	unsigned int connectAttemptNumber;
	uint32_t lastAttemptTime;
	uint32_t lastResetTime;
	UploadState state;
	EspUploadResult uploadResult;
	int restartModeOnCompletion;
	ESPType espType;
#if STM32
	uint32_t *blkBuf32;
#endif
};

#endif	// HAS_WIFI_NETWORKING

#endif /* SRC_SAME70_WIFIFIRMWAREUPLOADER_H_ */
