/*
 * WifiFirmwareUploader.cpp
 *
 *  Created on: 15 Apr 2016
 *      Author: David
 */

#include "WifiFirmwareUploader.h"

#if HAS_WIFI_NETWORKING && (HAS_MASS_STORAGE || HAS_EMBEDDED_FILES)

#include "WiFiInterface.h"

#include <Platform/Platform.h>
#include <Platform/RepRap.h>
#include <Storage/FileStore.h>

constexpr uint32_t Esp32FlashModuleSize = 4 * 1024 * 1024;		// assume at least 4Mbytes flash

// ESP8266 command codes
const uint8_t ESP_FLASH_BEGIN = 0x02;		// Four 32-bit words: size to erase, number of data packets, data size in one packet, flash offset.
const uint8_t ESP_FLASH_DATA = 0x03;		// Four 32-bit words: data size, sequence number, 0, 0, then data. Uses Checksum.
const uint8_t ESP_FLASH_END = 0x04;			// One 32-bit word: 0 to reboot, 1 “run to user code”. Not necessary to send this command if you wish to stay in the loader
const uint8_t ESP_MEM_BEGIN = 0x05;			// total size, number of data packets, data size in one packet, memory offset
const uint8_t ESP_MEM_END = 0x06;			// Two 32-bit words: execute flag, entry point address
const uint8_t ESP_MEM_DATA = 0x07;			// Four 32-bit words: data size, sequence number, 0, 0, then data. Uses Checksum.
const uint8_t ESP_SYNC = 0x08;				// 36 bytes: 0x07 0x07 0x12 0x20, followed by 32 x 0x55
const uint8_t ESP_WRITE_REG = 0x09;			// Four 32-bit words: address, value, mask and delay (in microseconds)
const uint8_t ESP_READ_REG = 0x0a;			// Address as 32-bit word. Returns read data as 32-bit word in value field.
// The following two commands are needed by the ESP32 ROM loader
const uint8_t ESP_SPI_SET_PARAMS = 0x0b;	// Six 32-bit words: id, total size in bytes, block size, sector size, page size, status mask.
const uint8_t ESP_SPI_ATTACH = 0x0d;		// 32-bit word: Zero for normal SPI flash. A second 32-bit word (should be 0) is passed to ROM loader only.

// MAC address storage locations
const uint32_t ESP_OTP_MAC0 = 0x3ff00050;
const uint32_t ESP_OTP_MAC1 = 0x3ff00054;
const uint32_t ESP_OTP_MAC2	= 0x3ff00058;
const uint32_t ESP_OTP_MAC3 = 0x3ff0005c;

const size_t EspFlashBlockSize = 0x0400;			// we send 1K byte blocks

const uint8_t ESP_IMAGE_MAGIC = 0xe9;
const uint8_t ESP_CHECKSUM_MAGIC = 0xef;

// Status body lengths, used to initially identify ESP8266 v ESP32 devices
const size_t ESP_BODY_LENGTHS[] = {0, 2, 4, 4};
const char * const ESP_NAMES[] = {"unknown", "ESP8266", "ESP32", "ESP32+"};

// Following address contains unique per device type id used to confirm ESP8266 and identify original ESP32
const uint32_t CHIP_DETECT_MAGIC_REG_ADDR = 0x40001000;

// value stored at the above address (values taken from the esptool.py source)
const uint32_t ESP_IDS[] = {0xffffffff, 0xfff0c101, 0x00f01d83, 0x0000000};

// Messages corresponding to result codes, should make sense when followed by " error"
const char * const resultMessages[] =
{
	"no",											// 0
	"unknown", "unknown", "unknown", "unknown",		// 0x01 to 0x04
	"invalidMessage",								// 0x05
	"failedToAct",									// 0x06
	"invalidCrc",									// 0x07
	"flashWriteError",								// 0x08
	"flashReadError",								// 0x09
	"unknown",										// 0x0a
	"deflateError",									// 0x0b
	"unknown", "unknown", "unknown", "unknown",		// 0x0c to 0x0f

	"timeout",										// 0x10
	"comm write",
	"connect",
	"bad reply",
	"file read",
	"empty file",
	"response header",
	"slip frame",
	"slip state",
	"slip data"
};

// A note on baud rates.
// The ESP8266 supports 921600, 460800, 230400, 115200, 74880 and some lower baud rates.
// 921600b is not reliable because even though it sometimes succeeds in connecting, we get a bad response during uploading after a few blocks.
// Probably our UART ISR cannot receive bytes fast enough, perhaps because of the latency of the system tick ISR.
// 460800b doesn't always manage to connect, but if it does then uploading appears to be reliable.
// 230400b always manages to connect.
static const uint32_t uploadBaudRates[] = { 230400, 115200, 74880, 9600 };

WifiFirmwareUploader::WifiFirmwareUploader(AsyncSerial& port, WiFiInterface& iface) noexcept
	: uploadPort(port), interface(iface), uploadFile(nullptr), state(UploadState::idle), espType(ESPType::unknown)
{
}

bool WifiFirmwareUploader::IsReady() const noexcept
{
	return state == UploadState::idle;
}

void WifiFirmwareUploader::MessageF(const char *fmt, ...) noexcept
{
	va_list vargs;
	va_start(vargs, fmt);
	reprap.GetPlatform().MessageV(FirmwareUpdateMessage, fmt, vargs);
	va_end(vargs);
}

void WifiFirmwareUploader::flushInput() noexcept
{
	while (uploadPort.available() != 0)
	{
		(void)uploadPort.read();
	}
}

// Read a byte optionally performing SLIP decoding.  The return values are:
//
//	2 - an escaped byte was read successfully
//	1 - a non-escaped byte was read successfully
//	0 - no data was available
//   -1 - the value 0xc0 was encountered (shouldn't happen)
//   -2 - a SLIP escape byte was found but the following byte wasn't available
//   -3 - a SLIP escape byte was followed by an invalid byte
int WifiFirmwareUploader::ReadByte(uint8_t& data, bool slipDecode) noexcept
{
	if (uploadPort.available() == 0)
	{
		return(0);
	}

	// at least one byte is available
	data = uploadPort.read();
	if (!slipDecode)
	{
		return(1);
	}

	if (data == 0xc0)
	{
		// this shouldn't happen
		return(-1);
	}

	// if not the SLIP escape, we're done
	if (data != 0xdb)
	{
		return(1);
	}

	// SLIP escape, check availability of subsequent byte
	if (uploadPort.available() == 0)
	{
		return(-2);
	}

	// process the escaped byte
	data = uploadPort.read();
	if (data == 0xdc)
	{
		data = 0xc0;
		return(2);
	}

	if (data == 0xdd)
	{
		data = 0xdb;
		return(2);
	}
	// invalid
	return(-3);
}

// When we write a sync packet, there must be no gaps between most of the characters.
// So use this function, which does a block write to the UART buffer in the latest CoreNG.
void WifiFirmwareUploader::writePacketRaw(const uint8_t *buf, size_t len) noexcept
{
	uploadPort.write(buf, len);
}

// Write a byte to the serial port optionally SLIP encoding. Return the number of bytes actually written.
inline void WifiFirmwareUploader::WriteByteRaw(uint8_t b) noexcept
{
	uploadPort.write(b);
}

// Write a byte to the serial port optionally SLIP encoding. Return the number of bytes actually written.
inline void WifiFirmwareUploader::WriteByteSlip(uint8_t b) noexcept
{
	if (b == 0xC0)
	{
		WriteByteRaw(0xDB);
		WriteByteRaw(0xDC);
	}
	else if (b == 0xDB)
	{
		WriteByteRaw(0xDB);
		WriteByteRaw(0xDD);
	}
	else
	{
		uploadPort.write(b);
	}
}

// Wait for a data packet to be returned.  If the body of the packet is
// non-zero length, return an allocated buffer indirectly containing the
// data and return the data length. Note that if the pointer for returning
// the data buffer is NULL, the response is expected to be two bytes of zero.
//
// If an error occurs, return a negative value.  Otherwise, return the number
// of bytes in the response (or zero if the response was not the standard "two bytes of zero").
WifiFirmwareUploader::EspUploadResult WifiFirmwareUploader::readPacket(uint8_t op, uint32_t *valp, size_t& bodyLen, uint32_t *status, uint32_t msTimeout) noexcept
{
	enum class PacketState
	{
		begin = 0,
		header,
		body,
		end,
		done
	};

	const size_t headerLength = 8;

	uint32_t startTime = millis();
	alignas(4) uint8_t hdr[headerLength];
	uint16_t hdrIdx = 0;
	bodyLen = 0;
	uint16_t bodyIdx = 0;
	alignas(4) uint8_t respBuf[4];

	// wait for the response
	uint16_t needBytes = 1;
	PacketState pState = PacketState::begin;
	while (pState != PacketState::done)
	{
		uint8_t c;
		EspUploadResult stat;

		if (millis() - startTime > msTimeout)
		{
			return(EspUploadResult::timeout);
		}

		if (uploadPort.available() < needBytes)
		{
			// insufficient data available
			// preferably, return to Spin() here
			continue;
		}

		// sufficient bytes have been received for the current state, process them
		switch(pState)
		{
		case PacketState::begin:	// expecting frame start
		case PacketState::end:		// expecting frame end
			c = uploadPort.read();
			if (c != 0xc0)
			{
				return EspUploadResult::slipFrame;
			}
			if (pState == PacketState::begin)
			{
				pState = PacketState::header;
				needBytes = 2;
			}
			else
			{
				pState = PacketState::done;
			}
			break;

		case PacketState::header:	// reading an 8-byte header
		case PacketState::body:		// reading the response body
			{
				int rslt;
				// retrieve a byte with SLIP decoding
				rslt = ReadByte(c, true);
				if (rslt != 1 && rslt != 2)
				{
					// some error occurred
					stat = (rslt == 0 || rslt == -2) ? EspUploadResult::slipData : EspUploadResult::slipFrame;
					return stat;
				}
				else if (pState == PacketState::header)
				{
					//store the header byte
					hdr[hdrIdx++] = c;
					if (hdrIdx >= headerLength)
					{
						// get the body length, prepare a buffer for it
						bodyLen = *(const uint16_t*)(hdr + 2);

						// extract the value, if requested
						if (valp != nullptr)
						{
							*valp = *(const uint32_t*)(hdr + 4);
						}

						if (bodyLen != 0)
						{
							pState = PacketState::body;
						}
						else
						{
							needBytes = 1;
							pState = PacketState::end;
						}
					}
				}
				else
				{
					// Store the response body byte, check for completion
					if (bodyIdx < ARRAY_SIZE(respBuf))
					{
						respBuf[bodyIdx] = c;
					}
					++bodyIdx;
					if (bodyIdx >= bodyLen)
					{
						needBytes = 1;
						pState = PacketState::end;
					}
				}
			}
			break;

		default:		// this shouldn't happen
			return EspUploadResult::slipState;
		}
	}

	// Extract elements from the header
	const uint8_t resp = hdr[0];
	const uint8_t opRet = hdr[1];

	// Sync packets often provoke a response with a zero opcode instead of ESP_SYNC
	if (resp != 0x01 || opRet != op)
	{
//debugPrintf("resp %02x %02x\n", resp, opRet);
		return EspUploadResult::respHeader;
	}

	if (status != nullptr)
	{
		*status = (bodyLen == 2) ? *(const uint16_t*)respBuf : *(const uint32_t*)respBuf;
	}

	return EspUploadResult::success;
}

// Send a block of data performing SLIP encoding of the content.
inline void WifiFirmwareUploader::writePacket(const uint8_t *data, size_t len) noexcept
{
	while (len != 0)
	{
		WriteByteSlip(*data++);
		--len;
	}
}

// Send a packet to the serial port while performing SLIP framing. The packet data comprises a header and an optional data block.
// A SLIP packet begins and ends with 0xc0.  The data encapsulated has the bytes
// 0xc0 and 0xdb replaced by the two-byte sequences {0xdb, 0xdc} and {0xdb, 0xdd} respectively.
void WifiFirmwareUploader::writePacket(const uint8_t *hdr, size_t hdrLen, const uint8_t *data, size_t dataLen) noexcept
{
	WriteByteRaw(0xc0);				// send the packet start character
	writePacket(hdr, hdrLen);		// send the header
	writePacket(data, dataLen);		// send the data block
	WriteByteRaw(0xc0);				// send the packet end character
}

// Send a packet to the serial port while performing SLIP framing. The packet data comprises a header and an optional data block.
// This is like writePacket except that it does a fast block write for both the header and the main data with no SLIP encoding. Used to send sync commands.
void WifiFirmwareUploader::writePacketRaw(const uint8_t *hdr, size_t hdrLen, const uint8_t *data, size_t dataLen) noexcept
{
	WriteByteRaw(0xc0);				// send the packet start character
	writePacketRaw(hdr, hdrLen);	// send the header
	writePacketRaw(data, dataLen);	// send the data block in raw mode
	WriteByteRaw(0xc0);				// send the packet end character
}

// Send a command to the attached device together with the supplied data, if any.
// The data is supplied via a list of one or more segments.
void WifiFirmwareUploader::sendCommand(uint8_t op, uint32_t checkVal, const uint8_t *data, size_t dataLen) noexcept
{
	struct __attribute__((packed)) CommandHeader
	{
		uint8_t zero;
		uint8_t op;
		uint16_t dataLen;
		uint32_t checkVal;
	};

	// populate the header
	CommandHeader cmdHdr = { 0, op, (uint16_t)dataLen, checkVal };

	// send the packet
	flushInput();
	if (op == ESP_SYNC)
	{
		writePacketRaw((const uint8_t*)&cmdHdr, sizeof(cmdHdr), data, dataLen);
	}
	else
	{
		writePacket((const uint8_t*)&cmdHdr, sizeof(cmdHdr), data, dataLen);
	}
}

// Send a command to the attached device together with the supplied data, if any, and get the response
WifiFirmwareUploader::EspUploadResult WifiFirmwareUploader::doCommand(uint8_t op, const uint8_t *data, size_t dataLen, uint32_t checkVal, uint32_t *valp, uint32_t msTimeout) noexcept
{
	sendCommand(op, checkVal, data, dataLen);
	size_t bodyLen;
	uint32_t status;
	EspUploadResult stat = readPacket(op, valp, bodyLen, &status, msTimeout);
	if (stat == EspUploadResult::success)
	{
		if (espType != ESPType::unknown && bodyLen != ESP_BODY_LENGTHS[(unsigned int)espType])
		{
			stat = EspUploadResult::badReply;
		}
		else if ((status & 0xFF) != 0)
		{
			stat = (EspUploadResult)((status >> 8) & 0xFF);
			if (reprap.Debug(Module::WiFi))
			{
				debugPrintf("opcode %u returned length %u status 0x%" PRIx32 "\n", op, bodyLen, status);
			}
		}
	}

	return stat;
}

// Send a synchronising packet to the serial port in an attempt to induce the ESP8266 to auto-baud lock on the baud rate.
WifiFirmwareUploader::EspUploadResult WifiFirmwareUploader::Sync(uint16_t timeout) noexcept
{
	uint8_t buf[36];

	// compose the data for the sync attempt
	memset(buf, 0x55, sizeof(buf));
	buf[0] = 0x07;
	buf[1] = 0x07;
	buf[2] = 0x12;
	buf[3] = 0x20;

	size_t bodyLen;
	sendCommand(ESP_SYNC, 0, buf, sizeof(buf));
	EspUploadResult stat = readPacket(ESP_SYNC, nullptr, bodyLen, nullptr, timeout);

	// If we got a response other than sync, discard it and wait for a sync response. This happens at higher baud rates.
	for (int i = 0; i < 10 && stat == EspUploadResult::respHeader; ++i)
	{
		stat = readPacket(ESP_SYNC, nullptr, bodyLen, nullptr, timeout);
	}

	if (stat == EspUploadResult::success)
	{
		// Set the initial type of ESP based on status reply length
		espType = (bodyLen == ESP_BODY_LENGTHS[ESPType::ESP32]) ? ESPType::ESP32 : ESPType::ESP8266;

		// Read and discard additional replies
		for (;;)
		{
			EspUploadResult rc = readPacket(ESP_SYNC, nullptr, bodyLen, nullptr, defaultTimeout);
			if (rc != EspUploadResult::success)
			{
				break;
			}
		}
	}
//DEBUG
//	else debugPrintf("stat=%d\n", (int)stat);
	return stat;
}

// Send a command to the device to begin the Flash process.
WifiFirmwareUploader::EspUploadResult WifiFirmwareUploader::flashBegin(uint32_t offset, uint32_t size) noexcept
{
	// Determine the number of blocks represented by the size
	const uint32_t blkCnt = (size + (EspFlashBlockSize - 1)) / EspFlashBlockSize;

	// Determine the erase size parameter to pass in the FLASH_BEGIN command
	uint32_t erase_size;
	if (espType == ESPType::ESP8266)
	{
		// Calculate the number of sectors to erase
		const uint32_t sector_size = 4 * 1024;
		uint32_t num_sectors = (size + (sector_size - 1))/sector_size;

		const uint32_t start_sector = offset / sector_size;
		const uint32_t sectors_per_block = 16;

		uint32_t head_sectors = sectors_per_block - (start_sector % sectors_per_block);
		if (num_sectors < head_sectors)
		{
			head_sectors = num_sectors;
		}

		// SPI EraseArea function in the esp8266 ROM has a bug which causes extra area to be erased.
		// If the address range to be erased crosses the block boundary then extra head_sector_count sectors are erased.
		// If the address range doesn't cross the block boundary, then extra total_sector_count sectors are erased.
		if (num_sectors < 2 * head_sectors)
		{
			num_sectors = ((num_sectors + 1) / 2);
		}
		else
		{
			num_sectors = (num_sectors - head_sectors);
		}
		erase_size = num_sectors * sector_size;
	}
	else
	{
		erase_size = size;
	}

	// begin the Flash process
	const uint32_t buf[5] = { erase_size, blkCnt, EspFlashBlockSize, offset, 0 };		// last word means not encrypted

	const uint32_t timeout = (erase_size != 0) ? eraseTimeout : defaultTimeout;
	return doCommand(ESP_FLASH_BEGIN, (const uint8_t*)buf, (espType == ESPType::ESP32_PLUS) ? sizeof(buf) : sizeof(buf) - sizeof(uint32_t), 0, nullptr, timeout);
}

// Send a command to the device to terminate the Flash process
WifiFirmwareUploader::EspUploadResult WifiFirmwareUploader::flashFinish(bool reboot) noexcept
{
	const uint32_t data = (reboot) ? 0 : 1;
	return doCommand(ESP_FLASH_END, (const uint8_t*)&data, sizeof(data), 0, nullptr, defaultTimeout);
}

WifiFirmwareUploader::EspUploadResult WifiFirmwareUploader::flashSpiSetParameters(uint32_t size) noexcept
{
	const uint32_t buf[6] = { 0, size, 64 * 1024, 4 * 1024, 256, 0x0000FFFF };	// id, size, block size, sector size, page size, status mask
	return doCommand(ESP_SPI_SET_PARAMS, (const uint8_t*)buf, sizeof(buf), 0, nullptr, defaultTimeout);
}

WifiFirmwareUploader::EspUploadResult WifiFirmwareUploader::flashSpiAttach() noexcept
{
	const uint32_t buf[2] = { 0, 0 };
	return doCommand(ESP_SPI_ATTACH, (const uint8_t*)buf, sizeof(buf), 0, nullptr, defaultTimeout);
}

// Compute the checksum of a block of data
uint16_t WifiFirmwareUploader::checksum(const uint8_t *data, uint16_t dataLen, uint16_t cksum) noexcept
{
	if (data != NULL)
	{
		while (dataLen--)
		{
			cksum ^= (uint16_t)*data++;
		}
	}
	return(cksum);
}

WifiFirmwareUploader::EspUploadResult WifiFirmwareUploader::flashWriteBlock(uint16_t flashParmVal, uint16_t flashParmMask) noexcept
{
	const uint32_t blkSize = EspFlashBlockSize;

	// Allocate a data buffer for the combined header and block data
	const uint16_t dataOfst = 16;
	const uint16_t blkBufSize = dataOfst + blkSize;
#if !STM32
	// On the STM32F4 our stack is not DMA capable, so we can't use the stack instead we allocate it when we open the file.
	uint32_t blkBuf32[blkBufSize/4];
#endif
	uint8_t * const blkBuf = reinterpret_cast<uint8_t*>(blkBuf32);

	// Prepare the header for the block
	blkBuf32[0] = blkSize;
	blkBuf32[1] = uploadBlockNumber;
	blkBuf32[2] = 0;
	blkBuf32[3] = 0;

	// Get the data for the block
	size_t cnt = uploadFile->Read(reinterpret_cast<char *>(blkBuf + dataOfst), blkSize);
	if (cnt != blkSize)
	{
		if (uploadFile->Position() == fileSize)
		{
			// partial last block, fill the remainder
			memset(blkBuf + dataOfst + cnt, 0xff, blkSize - cnt);
		}
		else
		{
			return EspUploadResult::fileRead;
		}
	}

	// Patch the flash parameters into the first block if it is loaded at address 0
	if (uploadBlockNumber == 0 && uploadAddress == 0 && blkBuf[dataOfst] == ESP_IMAGE_MAGIC && flashParmMask != 0)
	{
		// update the Flash parameters
		const uint32_t flashParm = (blkBuf32[dataOfst/4] >> 16) & ~(uint32_t)flashParmMask;
		blkBuf32[dataOfst/4] = (blkBuf32[dataOfst/4] & 0x0000FFFF) | ((flashParm | flashParmVal) << 16);
	}

	// Calculate the block checksum
	const uint16_t cksum = checksum(blkBuf + dataOfst, blkSize, ESP_CHECKSUM_MAGIC);
	EspUploadResult stat;
	for (int i = 0; i < 3; i++)
	{
		if ((stat = doCommand(ESP_FLASH_DATA, blkBuf, blkBufSize, cksum, nullptr, blockWriteTimeout)) == EspUploadResult::success)
		{
			break;
		}
	}

	return stat;
}

WifiFirmwareUploader::EspUploadResult WifiFirmwareUploader::DoErase(uint32_t address, uint32_t size) noexcept
{
	MessageF("Erasing %u bytes...\n", size);
	return flashBegin(address, size);
}

void WifiFirmwareUploader::Identify() noexcept
{
	if (espType != ESPType::unknown)
	{
		// sync will have done basics so we can safely use doCommand
		const uint32_t addr = CHIP_DETECT_MAGIC_REG_ADDR;
		uint32_t regVal;
		EspUploadResult stat;

		if ((stat = doCommand(ESP_READ_REG, (const uint8_t*)&addr, sizeof(addr), 0, &regVal, defaultTimeout)) == EspUploadResult::success)
		{
			//debugPrintf("read magic returns %x current type is %s\n", regVal, ESP_NAMES[espType]);
			if (regVal == ESP_IDS[ESPType::ESP8266])
			{
				espType = ESPType::ESP8266;
			}
			else if (regVal == ESP_IDS[ESPType::ESP32])
			{
				espType = ESPType::ESP32;
			}
			else if (espType == ESPType::ESP32)
			{
				// we don't recognise it as ESP8266 or ESP32, but we think it is an ESP32 device. Assume a newer ESP32
				espType = ESPType::ESP32_PLUS;
			}
			// if we don't have an explicit match use whatever sync decided
		}
	}
}

void WifiFirmwareUploader::Spin() noexcept
{
	switch (state)
	{
	case UploadState::resetting:
		if (connectAttemptNumber == ARRAY_SIZE(uploadBaudRates) * retriesPerBaudRate)
		{
			// Time to give up
			interface.ResetWiFi();
			uploadResult = EspUploadResult::connect;
			state = UploadState::done;
		}
		else
		{
			// Reset the serial port at the new baud rate. Also reset the ESP8266.
			const uint32_t baud = uploadBaudRates[connectAttemptNumber/retriesPerBaudRate];
			if (connectAttemptNumber % retriesPerBaudRate == 0)
			{
				// First attempt at this baud rate
				MessageF("Trying to connect at %u baud: ", baud);
			}
			interface.ResetWiFiForUpload(false);
			uploadPort.begin(baud);
			lastAttemptTime = lastResetTime = millis();
			state = UploadState::connecting;
		}
		break;

	case UploadState::connecting:
		if (millis() - lastAttemptTime >= connectAttemptInterval && millis() - lastResetTime >= resetDelay)
		{
			// Attempt to establish a connection to the ESP8266.
			EspUploadResult res = Sync(syncTimeout);
			lastAttemptTime = millis();
			if (res == EspUploadResult::success)
			{
				// Successful connection
				Identify();
				MessageF("success, found %s\n", ESP_NAMES[espType]);
				if (espType == ESPType::ESP8266)
				{
					state = UploadState::erasing1;
				}
				else
				{
					res = flashSpiAttach();
					if (res != EspUploadResult::success)
					{
						MessageF("Failed to attach SPI flash\n");
						state = UploadState::resetting;		// try a reset and a lower baud rate
						break;
					}
					res = flashSpiSetParameters(Esp32FlashModuleSize);
					if (res != EspUploadResult::success)
					{
						MessageF("Failed to set SPI parameters\n");
						state = UploadState::resetting;		// try a reset and a lower baud rate
						break;
					}
					MessageF("SPI flash parameters set\n");
					state = UploadState::erasing2;
				}
			}
			else
			{
				// This attempt failed
				++connectAttemptNumber;
				if (connectAttemptNumber % retriesPerReset == 0)
				{
					if (connectAttemptNumber % retriesPerBaudRate == 0)
					{
						MessageF("failed\n");
					}
					state = UploadState::resetting;		// try a reset and a lower baud rate
				}
			}
		}
		break;

	case UploadState::erasing1:
		if (millis() - lastAttemptTime >= blockWriteInterval)
		{
			uploadResult = DoErase(esp8266systemParametersAddress, esp8266systemParametersSize);
			if (uploadResult == EspUploadResult::success)
			{
				state = UploadState::erasing2;
			}
			else
			{
				MessageF("Erase failed\n");
				state = UploadState::done;
			}
		}
		break;

	case UploadState::erasing2:
		if (millis() - lastAttemptTime >= blockWriteInterval)
		{
			uploadResult = DoErase(uploadAddress, fileSize);
			if (uploadResult == EspUploadResult::success)
			{
				MessageF("Uploading file...\n");
				uploadBlockNumber = 0;
				uploadNextPercentToReport = percentToReportIncrement;
				lastAttemptTime = millis();
				state = UploadState::uploading;
			}
			else
			{
				MessageF("Erase failed\n");
				state = UploadState::done;
			}
		}
		break;

	case UploadState::uploading:
		// The ESP needs several milliseconds to recover from one packet before it will accept another
		if (millis() - lastAttemptTime >= blockWriteInterval)
		{
			const uint32_t blkCnt = (fileSize + EspFlashBlockSize - 1) / EspFlashBlockSize;
			if (uploadBlockNumber < blkCnt)
			{
				uploadResult = flashWriteBlock(0, 0);
				lastAttemptTime = millis();
				if (uploadResult != EspUploadResult::success)
				{
					MessageF("Flash block upload failed\n");
					state = UploadState::done;
				}
				const unsigned int percentComplete = (100 * uploadBlockNumber)/blkCnt;
				++uploadBlockNumber;
				if (percentComplete >= uploadNextPercentToReport)
				{
					MessageF("%u%% complete\n", percentComplete);
					uploadNextPercentToReport += percentToReportIncrement;
				}
			}
			else
			{
				state = UploadState::done;
			}
		}
		break;

	case UploadState::done:
		uploadFile->Close();
		uploadPort.end();					// disable the port, it has a high interrupt priority
#if STM32
		DeleteObject(blkBuf32);
#endif
		if (uploadResult == EspUploadResult::success)
		{
			MessageF("Upload successful\n");
			if (restartModeOnCompletion == 1)
			{
				interface.Start();
			}
			else
			{
				interface.ResetWiFi();
			}
		}
		else
		{
			MessageF("Error: Installation failed due to %s error\n", resultMessages[(size_t)uploadResult]);
			// Not safe to restart the network
			interface.ResetWiFi();
		}
		state = UploadState::idle;
		break;

	default:
		break;
	}
}

// Try to upload the given file at the given address
void WifiFirmwareUploader::SendUpdateFile(const char *file, uint32_t address) noexcept
{
	Platform& platform = reprap.GetPlatform();
	uploadFile = platform.OpenFile(FIRMWARE_DIRECTORY, file, OpenMode::read);
	if (uploadFile == nullptr)
	{
		// Fall back to /sys if the wifi file wasn't found in /firmware
		uploadFile = platform.OpenFile(DEFAULT_SYS_DIR, file, OpenMode::read);
		if (uploadFile == nullptr)
		{
			MessageF("Failed to open file %s%s\n", FIRMWARE_DIRECTORY, file);
			return;
		}
	}

	fileSize = uploadFile->Length();
	if (fileSize == 0)
	{
		uploadFile->Close();
		MessageF("Upload file is empty %s\n", file);
		return;
	}

#if STM32
	// we need a buffer that is DMA capable
	const uint32_t blkSize = EspFlashBlockSize;

	// Allocate a data buffer for the combined header and block data
	const uint16_t dataOfst = 16;
	const uint16_t blkBufSize = dataOfst + blkSize;
	blkBuf32 = new uint32_t[blkBufSize/4];
	if (blkBuf32 == nullptr)
	{
		uploadFile->Close();
		MessageF("Unable to allocate upload buffer\n");
		return;
	}
#endif

	// Stop the network
	restartModeOnCompletion = interface.EnableState();
	interface.Stop();

	// Set up the state so that subsequent calls to Spin() will attempt the upload
	uploadAddress = address;
	connectAttemptNumber = 0;
	state = UploadState::resetting;
}

#endif	// HAS_WIFI_NETWORKING

// End
