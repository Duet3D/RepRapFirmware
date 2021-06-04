/*
 * MessageFormats.h
 *
 *  Created on: 29 Mar 2019
 *      Author: Christian
 */

#ifndef SRC_LINUX_MESSAGEFORMATS_H_
#define SRC_LINUX_MESSAGEFORMATS_H_

#include <cstddef>
#include <cstdint>
#include <ctime>

#include <RepRapFirmware.h>

constexpr uint8_t LinuxFormatCode = 0x5F;			// standard format code for RRF SPI protocol
constexpr uint8_t LiunxFormatCodeStandalone = 0x60;	// used to indicate that RRF is running in stand-alone mode
constexpr uint8_t InvalidFormatCode = 0xC9;			// must be different from any other format code

constexpr uint16_t LinuxProtocolVersion = 5;

constexpr size_t LinuxTransferBufferSize = 8192;	// maximum length of a data transfer. Must be a multiple of 4 and kept in sync with Duet Control Server!
static_assert(LinuxTransferBufferSize % sizeof(uint32_t) == 0, "LinuxTransferBufferSize must be a whole number of dwords");

constexpr size_t MaxCodeBufferSize = 256;			// maximum length of a G/M/T-code in binary encoding
static_assert(MaxCodeBufferSize % sizeof(uint32_t) == 0, "MaxCodeBufferSize must be a whole number of dwords");
static_assert(MaxCodeBufferSize >= GCODE_LENGTH, "MaxCodeBufferSize must be at least as big as GCODE_LENGTH");

constexpr uint32_t SpiMacroRequestTimeout = 3000;	// maximum time to wait a macro file
constexpr uint32_t SpiTransferTimeout = 500;		// maximum allowed delay between data exchanges during a full transfer (in ms)
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
	DriverId = 8,			// two sequential uint16_t representing board and port of a driver
	DriverIdArray = 9,		// array of driver ids
	Bool = 10,				// bool (int32_t)
	BoolArray = 11			// bool[] (uint8_t[])
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

// RepRapFirmware to Linux
struct AbortFileHeader
{
	uint8_t channel;
	bool abortAll;
	uint16_t padding;
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
	HeightMap = 8,						// Response to a heightmap request
	Locked = 9,							// Movement has been locked and machine is in standstill
	FileChunk = 10,						// Request another chunk of a file
	EvaluationResult = 11,				// Response to an expression evaluation request
	DoCode = 12,						// Perform a G/M/T-code from a code input
	WaitForMessageAcknowledgment = 13,	// Wait for a message to be acknowledged
	MacroFileClosed = 14,				// Last macro file has been closed
	MessageAcknowledged = 15,			// Pending message prompt has been acknowledged
	VariableResult = 16					// Result of a variable get or set request
};

enum class PrintPausedReason : uint8_t
{
	user = 1,
	gcode = 2,
	filamentChange = 3,
	trigger = 4,
	heaterFault = 5,
	filamentError = 6,
	stall = 7,
	lowVoltage = 8
};

struct PrintPausedHeader
{
	uint32_t filePosition;
	PrintPausedReason pauseReason;
	uint8_t paddingA;
	uint16_t paddingB;
};

// Linux to RepRapFirmware
enum class LinuxRequest : uint16_t
{
    EmergencyStop = 0,							// Perform immediate emergency stop
    Reset = 1,									// Reset the controller
    Code = 2,									// Request execution of a G/M/T-code
    GetObjectModel = 3,							// Request a part of the machine's object model
    SetObjectModel = 4,							// Set a value in the machine's object model
    PrintStarted = 5,							// Print has been started, set file print information
    PrintStopped = 6,							// Print has been stopped, reset file print information
    MacroCompleted = 7,							// Notification that a macro file has been fully processed
    GetHeightMap = 8,							// Request the heightmap coordinates as generated by G29 S0
    SetHeightMap = 9,							// Set the heightmap coordinates via G29 S1
    LockMovementAndWaitForStandstill = 10,		// Lock movement and wait for standstill
	Unlock = 11,								// Unlock occupied resources
	WriteIap = 12,								// Write another chunk of the IAP binary
	StartIap = 13,								// Launch the IAP binary
	AssignFilament = 14,						// Assign filament to an extruder
	FileChunk = 15,								// Response to a file chunk request from a CAN-connected board
	EvaluateExpression = 16,					// Evaluate an arbitrary expression
	Message = 17,								// Send an arbitrary message
	MacroStarted = 18,							// Macro file has been started
	FilesAborted = 19,							// All files on the given channel have been aborted by DSF
	SetVariable = 20,							// Assign a variable (global, set, var)
	DeleteLocalVariable = 21,					// Delete an existing local variable at the end of a code block

	InvalidRequest = 22
};

struct AssignFilamentHeader
{
	int32_t extruder;
	uint32_t filamentLength;
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

struct FileChunk
{
	int32_t dataLength;
	uint32_t fileLength;
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

struct PrintStartedHeader
{
	uint16_t filenameLength;
	uint16_t generatedByLength;
	uint32_t numFilaments;
	time_t lastModifiedTime;
	uint32_t fileSize;
	float firstLayerHeight;
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

struct DeleteLocalVariableHeader
{
	uint8_t channel;
	uint8_t variableLength;
	uint16_t padding;
};

#endif /* SRC_LINUX_MESSAGEFORMATS_H_ */
