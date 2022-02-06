/*
 * SbcMessageFormats.h
 *
 *  Created on: 29 Mar 2019
 *      Author: Christian
 */

#ifndef SRC_SBC_MESSAGEFORMATS_H_
#define SRC_SBC_MESSAGEFORMATS_H_

#if HAS_SBC_INTERFACE

#include <cstddef>
#include <cstdint>
#include <ctime>

#include <RepRapFirmware.h>
#include <Platform/PrintPausedReason.h>

constexpr uint8_t SbcFormatCode = 0x5F;				// standard format code for RRF SPI protocol
constexpr uint8_t SbcFormatCodeStandalone = 0x60;	// used to indicate that RRF is running in stand-alone mode
constexpr uint8_t InvalidFormatCode = 0xC9;			// must be different from any other format code

constexpr uint16_t SbcProtocolVersion = 6;

constexpr size_t SbcTransferBufferSize = 8192;		// maximum length of a data transfer. Must be a multiple of 4 and kept in sync with Duet Control Server!
static_assert(SbcTransferBufferSize % sizeof(uint32_t) == 0, "SbcTransferBufferSize must be a whole number of dwords");

constexpr size_t MaxCodeBufferSize = 256;			// maximum length of a G/M/T-code in binary encoding
static_assert(MaxCodeBufferSize % sizeof(uint32_t) == 0, "MaxCodeBufferSize must be a whole number of dwords");

constexpr uint32_t SpiTransferDelay = 25;			// default time to wait after a transfer before another one is started (in ms)
constexpr uint32_t SpiFileOpenDelay = 5;			// same as above but when a file is open
constexpr uint32_t SpiEventsRequired = 4;			// number of events required to happen in RRF before the delay is skipped

constexpr uint32_t SpiMaxRequestTime = 3000;		// maximum time to wait a blocking request (like macros or file requests, in ms)
constexpr uint32_t SpiTransferTimeout = 500;		// maximum allowed delay between data exchanges during a full transfer (in ms)
constexpr uint32_t SpiMaxTransferTime = 50;			// maximum allowed time for a single SPI transfer
constexpr uint32_t SpiConnectionTimeout = 4000;		// maximum time to wait for the next transfer (in ms)
constexpr uint16_t SpiCodeBufferSize = 4096;		// number of bytes available for G-code caching

// Shared structures
enum class DataType : uint8_t
{
    Int = 0,				// int32_t
    UInt = 1,				// uint32_t
    Float = 2,				// float
    IntArray = 3,			// int32_t[]
    UIntArray = 4,			// uint32_t[]
    FloatArray = 5,			// float[]
    String = 6,				// char[]
    Expression = 7,			// char[] but containing '{'...'}'
	DriverId_dt = 8,		// two sequential uint16_t representing board and port of a driver
	DriverIdArray = 9,		// array of driver ids
	Bool = 10,				// bool (int32_t)
	BoolArray = 11,			// bool[] (uint8_t[])
	ULong = 12,				// uint64_t
	DateTime = 13,			// datetime string in ISO-conform formatting
	Null = 14				// null value
};

struct CodeChannelHeader
{
	uint8_t channel;
	uint8_t paddingA;
	uint16_t paddingB;
};

struct HeightMapHeader
{
	float xMin;
	float xMax;
	float xSpacing;
	float yMin;
	float yMax;
	float ySpacing;
	float radius;
	uint16_t numX;
	uint16_t numY;
};

struct MessageHeader
{
	MessageType messageType;
	uint16_t length;
	uint16_t padding;
};

struct PacketHeader
{
	uint16_t request;
	uint16_t id;
	uint16_t length;
	uint16_t resendPacketId;
};

struct StringHeader
{
	uint16_t length;
	uint16_t padding;
};

struct TransferHeader
{
	uint8_t formatCode;
	uint8_t numPackets;
	uint16_t protocolVersion;
	uint16_t sequenceNumber;
	uint16_t dataLength;
	uint32_t crcData;
	uint32_t crcHeader;
};

enum TransferResponse : uint32_t
{
	Success = 1,
	BadFormat = 2,
	BadProtocolVersion = 3,
	BadDataLength = 4,
	BadHeaderChecksum = 5,
	BadDataChecksum = 6,

	BadResponse = 0xFEFEFEFE
};

// RepRapFirmware to Sbc
struct AbortFileHeader
{
	uint8_t channel;
	bool abortAll;
	uint16_t padding;
};

struct FileHandleHeader
{
	FileHandle handle;
};

struct CodeBufferUpdateHeader
{
	uint16_t bufferSpace;
	uint16_t padding;
};

struct DoCodeHeader
{
	uint8_t channel;
	uint8_t padding;
	uint16_t length;
};

struct EvaluationResultHeader
{
	DataType dataType;
	uint8_t padding;
	uint16_t expressionLength;
	union
	{
		int32_t intValue;
		uint32_t uintValue;
		float floatValue;
	};
};

struct ExecuteMacroHeader
{
	uint8_t channel;
	uint8_t dummy;
	bool fromCode;
	uint8_t length;
};

struct FileChunkHeader
{
	uint32_t offset;
	uint32_t maxLength;
	uint32_t filenameLength;
};

struct OpenFileHeader
{
	bool forWriting;
	bool append;
	uint8_t filenameLength;
	uint8_t padding;
	uint32_t preAllocSize;
};

struct ReadFileHeader
{
	FileHandle handle;
	uint32_t maxLength;
};

struct SeekFileHeader
{
	FileHandle handle;
	uint32_t offset;
};

enum class FirmwareRequest : uint16_t
{
	ResendPacket = 0,					// Request the retransmission of the given packet
	ObjectModel = 1,					// Response to an object model request
	CodeBufferUpdate = 2,				// Update about the available code buffer size
	Message = 3,						// Message from the firmware
	ExecuteMacro = 4,					// Request execution of a macro file
	AbortFile = 5,						// Request the current file to be closed
	StackEvent_Obsolete = 6,			// Stack has been changed (unused)
	PrintPaused = 7,					// Print has been paused
	HeightMap_Obsolete = 8,				// Response to a heightmap request (no longer used)
	Locked = 9,							// Movement has been locked and machine is in standstill
	FileChunk = 10,            			// Request another chunk of a file (only used by CAN expansion board updates)
	EvaluationResult = 11,				// Response to an expression evaluation request
	DoCode = 12,						// Perform a G/M/T-code from a code input
	WaitForMessageAcknowledgment = 13,	// Wait for a message to be acknowledged
	MacroFileClosed = 14,				// Last macro file has been closed
	MessageAcknowledged = 15,			// Pending message prompt has been acknowledged
	VariableResult = 16,				// Result of a variable get or set request
	CheckFileExists = 17,				// Check if a file exists
	DeleteFileOrDirectory = 18,			// Delete a file or directory
	OpenFile = 19,						// Open a file on the SBC
	ReadFile = 20,						// Read from a file
	WriteFile = 21,						// Write to a file
	SeekFile = 22,						// Seek in a file
	TruncateFile = 23,					// Truncate a file
	CloseFile = 24						// Close a file again
};

struct PrintPausedHeader
{
	uint32_t filePosition;
	PrintPausedReason pauseReason;
	uint8_t paddingA;
	uint16_t paddingB;
};

// Sbc to RepRapFirmware
enum class SbcRequest : uint16_t
{
    EmergencyStop = 0,							// Perform immediate emergency stop
    Reset = 1,									// Reset the controller
    Code = 2,									// Request execution of a G/M/T-code
    GetObjectModel = 3,							// Request a part of the machine's object model
    SetObjectModel = 4,							// Set a value in the machine's object model
	SetPrintFileInfo = 5,						// Print is about to be started, set file print information
    PrintStopped = 6,							// Print has been stopped, reset file print information
    MacroCompleted = 7,							// Notification that a macro file has been fully processed
    GetHeightMap_deprecated = 8,				// Request the heightmap coordinates as generated by G29 S0 (no longer used)
    SetHeightMap_deprecated = 9,				// Set the heightmap coordinates via G29 S1 (no longer used)
    LockMovementAndWaitForStandstill = 10,		// Lock movement and wait for standstill
	Unlock = 11,								// Unlock occupied resources
	WriteIap = 12,								// Write another chunk of the IAP binary
	StartIap = 13,								// Launch the IAP binary
	AssignFilament_deprecated = 14,				// Assign filament to an extruder (no longer used)
	FileChunk = 15,								// Response to a file chunk request from a CAN-connected board
	EvaluateExpression = 16,					// Evaluate an arbitrary expression
	Message = 17,								// Send an arbitrary message
	MacroStarted = 18,							// Macro file has been started
	InvalidateChannel = 19,						// Invalidate all files and codes on a given channel
	SetVariable = 20,							// Assign a variable (global, set, var)
	DeleteLocalVariable = 21,					// Delete an existing local variable at the end of a code block
	CheckFileExistsResult = 22,					// Result of a request to check if a given file exists
	FileDeleteResult = 23,						// Result of a file deletion request
	OpenFileResult = 24,						// Result of an attempt to open a file
	FileReadResult = 25,						// Result of a file read request
	FileWriteResult = 26,						// Result of a file write request
	FileSeekResult = 27,						// Result of a file seek request
	FileTruncateResult = 28,					// Result of a file truncate request

	InvalidRequest = 29
};

struct BooleanHeader
{
	bool value;
	uint8_t paddingA;
	uint16_t paddingB;
};

enum CodeFlags : uint8_t
{
	HasMajorCommandNumber = 1,
	HasMinorCommandNumber = 2,
	HasFilePosition = 4,
	EnforceAbsolutePosition = 8
};

// Not used during data transfers
struct BufferedCodeHeader
{
	bool isPending;
	uint8_t padding;
	uint16_t length;
};

struct CodeHeader
{
	uint8_t channel;
	CodeFlags flags;
	uint8_t numParameters;
	char letter;
	int32_t majorCode;
	int32_t minorCode;
	uint32_t filePosition;
	int32_t lineNumber;
};

struct CodeParameter
{
	char letter;
	DataType type;
	uint16_t padding;
	union
	{
		int32_t intValue;
		uint32_t uintValue;
		float floatValue;
	};
};

struct DeleteLocalVariableHeader
{
	uint8_t channel;
	uint8_t variableLength;
	uint16_t padding;
};

struct FileChunk
{
	int32_t dataLength;
	uint32_t fileLength;
};

struct FileDataHeader
{
	int32_t bytesRead;
};

struct GetObjectModelHeader
{
	uint16_t keyLength;
	uint16_t flagsLength;
};

struct MacroCompleteHeader
{
	uint8_t channel;
	bool error;
	uint16_t padding;
};

struct OpenFileResult
{
	uint32_t handle;
	uint32_t fileSize;
};

struct PrintStartedHeader
{
	uint16_t filenameLength;
	uint16_t generatedByLength;
	uint32_t numFilaments;
	time_t lastModifiedTime;
	uint32_t fileSize;
	uint32_t numLayers;
	float layerHeight;
	float objectHeight;
	uint32_t printTime;
	uint32_t simulatedTime;
};

// Keep this in sync with StopPrintReason in GCodes/GCodes.h
enum class PrintStoppedReason : uint8_t
{
    normalCompletion = 0,
    userCancelled = 1,
    abort = 2
};

struct PrintStoppedHeader
{
	PrintStoppedReason reason;
	uint8_t paddingA;
	uint16_t paddingB;
};

#if false
struct SetObjectModelHeader		// unused
{
	DataType type;
	uint8_t fieldLength;
	union
	{
		int32_t intValue;
		uint32_t uintValue;
		float floatValue;
	};
};
#endif

struct SetVariableHeader
{
	uint8_t channel;
	bool createVariable;
	uint8_t variableLength;
	uint8_t expressionLength;
};

#endif

#endif /* SRC_SBC_MESSAGEFORMATS_H_ */
