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

// ESP8266 command codes
const uint8_t ESP_FLASH_BEGIN = 0x02;
const uint8_t ESP_FLASH_DATA = 0x03;
const uint8_t ESP_FLASH_END = 0x04;
const uint8_t ESP_MEM_BEGIN = 0x05;
const uint8_t ESP_MEM_END = 0x06;
const uint8_t ESP_MEM_DATA = 0x07;
const uint8_t ESP_SYNC = 0x08;
const uint8_t ESP_WRITE_REG = 0x09;
const uint8_t ESP_READ_REG = 0x0a;

// MAC address storage locations
const uint32_t ESP_OTP_MAC0 = 0x3ff00050;
const uint32_t ESP_OTP_MAC1 = 0x3ff00054;
const uint32_t ESP_OTP_MAC2	= 0x3ff00058;
const uint32_t ESP_OTP_MAC3 = 0x3ff0005c;

const size_t EspFlashBlockSize = 0x0400;			// 1K byte blocks

const uint8_t ESP_IMAGE_MAGIC = 0xe9;
const uint8_t ESP_CHECKSUM_MAGIC = 0xef;

const uint32_t ESP_ERASE_CHIP_ADDR = 0x40004984;	// &SPIEraseChip
const uint32_t ESP_SEND_PACKET_ADDR = 0x40003c80;	// &send_packet
const uint32_t ESP_SPI_READ_ADDR = 0x40004b1c;		// &SPIRead
const uint32_t ESP_UNKNOWN_ADDR = 0x40001121;		// not used
const uint32_t ESP_USER_DATA_RAM_ADDR = 0x3ffe8000;	// &user data ram
const uint32_t ESP_IRAM_ADDR = 0x40100000;			// instruction RAM
const uint32_t ESP_FLASH_ADDR = 0x40200000;			// address of start of Flash

// Messages corresponding to result codes, should make sense when followed by " error"
const char * const resultMessages[] =
{
	"no",
	"timeout",
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
	: uploadPort(port), interface(iface), uploadFile(nullptr), state(UploadState::idle)
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

// Extract 1-4 bytes of a value in little-endian order from a buffer beginning at a specified offset
uint32_t WifiFirmwareUploader::getData(unsigned byteCnt, const uint8_t *buf, int ofst) noexcept
{
	uint32_t val = 0;

	if (buf && byteCnt)
	{
		unsigned int shiftCnt = 0;
		if (byteCnt > 4)
			byteCnt = 4;
		do
		{
			val |= (uint32_t)buf[ofst++] << shiftCnt;
			shiftCnt += 8;
		} while (--byteCnt);
	}
	return(val);
}

// Put 1-4 bytes of a value in little-endian order into a buffer beginning at a specified offset.
void WifiFirmwareUploader::putData(uint32_t val, unsigned byteCnt, uint8_t *buf, int ofst) noexcept
{
	if (buf && byteCnt)
	{
		if (byteCnt > 4)
		{
			byteCnt = 4;
		}
		do
		{
			buf[ofst++] = (uint8_t)(val & 0xff);
			val >>= 8;
		} while (--byteCnt);
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
WifiFirmwareUploader::EspUploadResult WifiFirmwareUploader::readPacket(uint8_t op, uint32_t *valp, size_t& bodyLen, uint32_t msTimeout) noexcept
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
	uint8_t hdr[headerLength];
	uint16_t hdrIdx = 0;
	bodyLen = 0;
	uint16_t bodyIdx = 0;
	uint8_t respBuf[2];

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
						bodyLen = (uint16_t)getData(2, hdr, 2);

						// extract the value, if requested
						if (valp != nullptr)
						{
							*valp = getData(4, hdr, 4);
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
	const uint8_t resp = (uint8_t)getData(1, hdr, 0);
	const uint8_t opRet = (uint8_t)getData(1, hdr, 1);
	// Sync packets often provoke a response with a zero opcode instead of ESP_SYNC
	if (resp != 0x01 || opRet != op)
	{
//debugPrintf("resp %02x %02x\n", resp, opRet);
		return EspUploadResult::respHeader;
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
	// populate the header
	uint8_t hdr[8];
	putData(0, 1, hdr, 0);
	putData(op, 1, hdr, 1);
	putData(dataLen, 2, hdr, 2);
	putData(checkVal, 4, hdr, 4);

	// send the packet
	flushInput();
	if (op == ESP_SYNC)
	{
		writePacketRaw(hdr, sizeof(hdr), data, dataLen);
	}
	else
	{
		writePacket(hdr, sizeof(hdr), data, dataLen);
	}
}

// Send a command to the attached device together with the supplied data, if any, and get the response
WifiFirmwareUploader::EspUploadResult WifiFirmwareUploader::doCommand(uint8_t op, const uint8_t *data, size_t dataLen, uint32_t checkVal, uint32_t *valp, uint32_t msTimeout) noexcept
{
	sendCommand(op, checkVal, data, dataLen);
	size_t bodyLen;
	EspUploadResult stat = readPacket(op, valp, bodyLen, msTimeout);
	if (stat == EspUploadResult::success && bodyLen != 2)
	{
		stat = EspUploadResult::badReply;
	}

	return stat;
}

// Send a synchronising packet to the serial port in an attempt to induce
// the ESP8266 to auto-baud lock on the baud rate.
WifiFirmwareUploader::EspUploadResult WifiFirmwareUploader::Sync(uint16_t timeout) noexcept
{
	uint8_t buf[36];

	// compose the data for the sync attempt
	memset(buf, 0x55, sizeof(buf));
	buf[0] = 0x07;
	buf[1] = 0x07;
	buf[2] = 0x12;
	buf[3] = 0x20;

	EspUploadResult stat = doCommand(ESP_SYNC, buf, sizeof(buf), 0, nullptr, timeout);

	// If we got a response other than sync, discard it and wait for a sync response. This happens at higher baud rates.
	for (int i = 0; i < 10 && stat == EspUploadResult::respHeader; ++i)
	{
		size_t bodyLen;
		stat = readPacket(ESP_SYNC, nullptr, bodyLen, timeout);
	}

	if (stat == EspUploadResult::success)
	{
		// Read and discard additional replies
		for (;;)
		{
			size_t bodyLen;
			EspUploadResult rc = readPacket(ESP_SYNC, nullptr, bodyLen, defaultTimeout);
			if (rc != EspUploadResult::success || bodyLen != 2)
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
WifiFirmwareUploader::EspUploadResult WifiFirmwareUploader::flashBegin(uint32_t addr, uint32_t size) noexcept
{
	// determine the number of blocks represented by the size
	const uint32_t blkCnt = (size + EspFlashBlockSize - 1) / EspFlashBlockSize;

	// ensure that the address is on a block boundary
	addr &= ~(EspFlashBlockSize - 1);

	// begin the Flash process
	uint8_t buf[16];
	putData(size, 4, buf, 0);
	putData(blkCnt, 4, buf, 4);
	putData(EspFlashBlockSize, 4, buf, 8);
	putData(addr, 4, buf, 12);

	uint32_t timeout = (size != 0) ? eraseTimeout : defaultTimeout;
	return doCommand(ESP_FLASH_BEGIN, buf, sizeof(buf), 0, nullptr, timeout);
}

// Send a command to the device to terminate the Flash process
WifiFirmwareUploader::EspUploadResult WifiFirmwareUploader::flashFinish(bool reboot) noexcept
{
	uint8_t buf[4];
	putData(reboot ? 0 : 1, 4, buf, 0);
	return doCommand(ESP_FLASH_END, buf, sizeof(buf), 0, nullptr, defaultTimeout);
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
	const uint16_t hdrOfst = 0;
	const uint16_t dataOfst = 16;
	const uint16_t blkBufSize = dataOfst + blkSize;
	uint32_t blkBuf32[blkBufSize/4];
	uint8_t * const blkBuf = reinterpret_cast<uint8_t*>(blkBuf32);

	// Prepare the header for the block
	putData(blkSize, 4, blkBuf, hdrOfst + 0);
	putData(uploadBlockNumber, 4, blkBuf, hdrOfst + 4);
	putData(0, 4, blkBuf, hdrOfst + 8);
	putData(0, 4, blkBuf, hdrOfst + 12);

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
		uint32_t flashParm = getData(2, blkBuf + dataOfst + 2, 0) & ~(uint32_t)flashParmMask;
		putData(flashParm | flashParmVal, 2, blkBuf + dataOfst + 2, 0);
	}

	// Calculate the block checksum
	uint16_t cksum = checksum(blkBuf + dataOfst, blkSize, ESP_CHECKSUM_MAGIC);
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
	const uint32_t sectorsPerBlock = 16;
	const uint32_t sectorSize = 4096;
	const uint32_t numSectors = (size + sectorSize - 1)/sectorSize;
	const uint32_t startSector = address/sectorSize;
	uint32_t headSectors = sectorsPerBlock - (startSector % sectorsPerBlock);

	if (numSectors < headSectors)
	{
		headSectors = numSectors;
	}
    const uint32_t eraseSize = (numSectors < 2 * headSectors)
    									? (numSectors + 1) / 2 * sectorSize
    									: (numSectors - headSectors) * sectorSize;

	MessageF("Erasing %u bytes...\n", eraseSize);
	return flashBegin(uploadAddress, eraseSize);
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
			uploadPort.begin(baud);
			interface.ResetWiFiForUpload(false);
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
//				MessageF(" success on attempt %d\n", (connectAttemptNumber % retriesPerBaudRate) + 1);
				MessageF(" success\n");
				state = UploadState::erasing1;
			}
			else
			{
				// This attempt failed
				++connectAttemptNumber;
				if (connectAttemptNumber % retriesPerReset == 0)
				{
					if (connectAttemptNumber % retriesPerBaudRate == 0)
					{
						MessageF(" failed\n");
					}
					state = UploadState::resetting;		// try a reset and a lower baud rate
				}
			}
		}
		break;

	case UploadState::erasing1:
		if (millis() - lastAttemptTime >= blockWriteInterval)
		{
			uploadResult = DoErase(systemParametersAddress, systemParametersSize);
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
