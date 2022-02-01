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

	static const uint32_t FirmwareAddress = 0x00000000;
	static const uint32_t WebFilesAddress = 0x00100000;

private:
	static const uint32_t defaultTimeout = 500;				// default timeout in milliseconds
	static const uint32_t syncTimeout = 1000;
	static const unsigned int retriesPerBaudRate = 9;
	static const unsigned int retriesPerReset = 3;
	static const uint32_t connectAttemptInterval = 50;
	static const uint32_t resetDelay = 500;
	static const uint32_t blockWriteInterval = 15;			// 15ms is long enough, 10ms is mostly too short
	static const uint32_t blockWriteTimeout = 200;
	static const uint32_t eraseTimeout = 15000;				// increased from 12 to 15 seconds because Roland's board was timing out
	static const unsigned int percentToReportIncrement = 5;	// how often we report % complete
	static const uint32_t systemParametersAddress = 0x3FE000;	// the address of the system + user parameter area that needs to be cleared when changing SDK version
	static const uint32_t systemParametersSize = 0x2000;		// the size of the system + user parameter area

	// Return codes - this list must be kept in step with the corresponding messages
	enum class EspUploadResult
	{
		success = 0,
		timeout,
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

	void MessageF(const char *fmt, ...) noexcept;
	uint32_t getData(unsigned byteCnt, const uint8_t *buf, int ofst) noexcept;
	void putData(uint32_t val, unsigned byteCnt, uint8_t *buf, int ofst) noexcept;
	int ReadByte(uint8_t& data, bool slipDecode) noexcept;
	void WriteByteRaw(uint8_t b) noexcept;
	void WriteByteSlip(uint8_t b) noexcept;
	void flushInput() noexcept;
	EspUploadResult readPacket(uint8_t op, uint32_t *valp, size_t& bodyLen, uint32_t msTimeout) noexcept;
	void writePacket(const uint8_t *data, size_t len) noexcept;
	void writePacketRaw(const uint8_t *buf, size_t len) noexcept;
	void writePacket(const uint8_t *hdr, size_t hdrLen, const uint8_t *data, size_t dataLen) noexcept;
	void writePacketRaw(const uint8_t *hdr, size_t hdrLen, const uint8_t *data, size_t dataLen) noexcept;
	void sendCommand(uint8_t op, uint32_t checkVal, const uint8_t *data, size_t dataLen) noexcept;
	EspUploadResult doCommand(uint8_t op, const uint8_t *data, size_t dataLen, uint32_t checkVal, uint32_t *valp, uint32_t msTimeout) noexcept;
	EspUploadResult Sync(uint16_t timeout) noexcept;
	EspUploadResult flashBegin(uint32_t addr, uint32_t size) noexcept;
	EspUploadResult flashFinish(bool reboot) noexcept;
	static uint16_t checksum(const uint8_t *data, uint16_t dataLen, uint16_t cksum) noexcept;
	EspUploadResult flashWriteBlock(uint16_t flashParmVal, uint16_t flashParmMask) noexcept;
	EspUploadResult DoErase(uint32_t address, uint32_t size) noexcept;

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
};

#endif	// HAS_WIFI_NETWORKING

#endif /* SRC_SAME70_WIFIFIRMWAREUPLOADER_H_ */
