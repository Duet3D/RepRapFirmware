/*
 * GCodeInput.cpp
 *
 *  Created on: 16 Sep 2016
 *      Author: Christian
 */

#include "GCodeInput.h"

#include <Platform/RepRap.h>
#include <Movement/Move.h>
#include "GCodes.h"
#include "GCodeBuffer/GCodeBuffer.h"
#include <Storage/MassStorage.h>

#include <Devices.h>

const size_t GCodeInputFileReadThreshold = 128;		// How many free bytes must be available before data is read from the file
const size_t GCodeInputUSBReadThreshold = 128;		// How many free bytes must be available before we read more data from USB

// Read some input bytes into the GCode buffer. Return true if there is a line of GCode waiting to be processed.
// This needs to be efficient
bool StandardGCodeInput::FillBuffer(GCodeBuffer *gb) noexcept
{
	const size_t bytesToPass = BytesCached();

#if HAS_MASS_STORAGE
	// To save calling IsWritingBinary on every character when we are not uploading a file in binary mode, use a separate loop for uploading
	if (gb->IsWritingBinary())
	{
		for (size_t i = 0; i < bytesToPass; i++)
		{
			const char c = ReadByte();
			if (gb->WriteBinaryToFile(c))
			{
				return false;				// finished binary upload, so we may as well stop here
			}
		}
	}
	else
#endif
	{
		for (size_t i = 0; i < bytesToPass; i++)
		{
			const char c = ReadByte();
			if (gb->Put(c))					// process a character, returns true if a line of GCode is complete
			{
#if HAS_MASS_STORAGE
				if (gb->IsWritingFile())
				{
					gb->WriteToFile();
				}
				else
#endif
				{
					return true;			// a line of GCode is complete, so stop here
				}
			}
		}
	}

	return false;
}

// G-code input class for wrapping around Stream-based hardware ports

void StreamGCodeInput::Reset() noexcept
{
	while (device.available() > 0)
	{
		device.read();
	}
}

char StreamGCodeInput::ReadByte() noexcept
{
	return static_cast<char>(device.read());
}

size_t StreamGCodeInput::BytesCached() const noexcept
{
	return device.available();
}

// Dynamic G-code input class for caching codes from software-defined sources

RegularGCodeInput::RegularGCodeInput(MessageType mt) noexcept
	: mtype(mt), state(GCodeInputState::idle), writingFile(false), writingPointer(0), readingPointer(0)
{
}

void RegularGCodeInput::Reset() noexcept
{
	state = GCodeInputState::idle;
	writingPointer = readingPointer = 0;
}

char RegularGCodeInput::ReadByte() noexcept
{
	char c = buffer[readingPointer++];
	if (readingPointer == GCodeInputBufferSize)
	{
		readingPointer = 0;
	}
	return c;
}

size_t RegularGCodeInput::BytesCached() const noexcept
{
	return (writingPointer - readingPointer) % GCodeInputBufferSize;
}

size_t RegularGCodeInput::BufferSpaceLeft() const noexcept
{
	return (readingPointer - writingPointer - 1u) % GCodeInputBufferSize;
}

// Check a character for urgent commands: M112 (emergency stop), M122 (diagnostics), M108 (cancel wait for temperature), M25.1 (feed hold) and M24 (feed hold release).
// Return true if an urgent command was handled and the buffer was reset
bool RegularGCodeInput::CheckForUrgentCommand(char c) noexcept
{
	if (writingFile)
	{
		return false;
	}

	switch (state)
	{
		case GCodeInputState::idle:
			if (c > ' ')
			{
				state = (c == 'M' || c == 'm') ? GCodeInputState::doingMCode : GCodeInputState::doingCode;
			}
			break;

		case GCodeInputState::doingCode:
			break;

		case GCodeInputState::doingMCode:
			state = (c == '1') ? GCodeInputState::doingMCode1
				  : (c == '2') ? GCodeInputState::doingMCode2
				  : GCodeInputState::doingCode;
			break;

		case GCodeInputState::doingMCode1:
			state = (c == '0') ? GCodeInputState::doingMCode10
				  : (c == '1') ? GCodeInputState::doingMCode11
				  : (c == '2') ? GCodeInputState::doingMCode12
				  : GCodeInputState::doingCode;
			break;

		case GCodeInputState::doingMCode2:
			state = (c == '4') ? GCodeInputState::doingMCode24
				  : (c == '5') ? GCodeInputState::doingMCode25
				  : GCodeInputState::doingCode;
			break;

		case GCodeInputState::doingMCode24:
			// A plain M24 while a feed hold is active and no print is paused only has to release the hold. Do it here and swallow the command,
			// because the channel it arrived on may be the one whose macro the hold has stopped, in which case the queued copy would never run
			if ((c < ' ' || c == ';') && reprap.GetMove().IsFeedHoldActive() && reprap.GetGCodes().GetPauseState() != PauseState::paused)
			{
				reprap.GetMove().ReleaseFeedHold();
				Reset();
				return true;
			}
			state = GCodeInputState::doingCode;
			break;

		case GCodeInputState::doingMCode25:
			state = (c == '.') ? GCodeInputState::doingMCode25dot : GCodeInputState::doingCode;
			break;

		case GCodeInputState::doingMCode25dot:
			state = (c == '1') ? GCodeInputState::doingMCode251 : GCodeInputState::doingCode;
			break;

		case GCodeInputState::doingMCode251:
			if (c <= ' ' || c == ';')
			{
				// Feed hold requested, with or without parameters. Stop the motion now because the GCodeBuffer may be blocked by a move,
				// and let the command execute again from the queue, which repeats the hold harmlessly or performs the S1 stop-and-abort
				reprap.GetMove().FeedHold();
			}
			state = (c == ' ') ? GCodeInputState::doingMCode251space : GCodeInputState::doingCode;
			break;

		case GCodeInputState::doingMCode251space:
			state = (c == 'S') ? GCodeInputState::doingMCode251S : GCodeInputState::doingCode;
			break;

		case GCodeInputState::doingMCode251S:
			state = (c == '1') ? GCodeInputState::doingMCode251S1 : GCodeInputState::doingCode;
			break;

		case GCodeInputState::doingMCode251S1:
			if (c < ' ' || c == ';')
			{
				// Stop-and-abort requested. The channel may be the one whose macro is to be aborted, so the queued copy would never run; hand it to the main task instead
				reprap.GetGCodes().RequestStopAndAbort();
				Reset();
				return true;
			}
			state = GCodeInputState::doingCode;
			break;

		case GCodeInputState::doingMCode10:
			state = (c == '8') ? GCodeInputState::doingMCode108 : GCodeInputState::doingCode;
			break;

		case GCodeInputState::doingMCode11:
			state = (c == '2') ? GCodeInputState::doingMCode112 : GCodeInputState::doingCode;
			break;

		case GCodeInputState::doingMCode12:
			state = (c == '2') ? GCodeInputState::doingMCode122 : GCodeInputState::doingCode;
			break;

		case GCodeInputState::doingMCode108:
			if (c < ' ' || c == ';')
			{
				reprap.GetGCodes().CancelWaitForTemperatures(false);
				// Normally we would call Reset and then return here to prevent the command being executed twice.
				// However, the host may be expecting an empty response, which we can't always send from here.
				// So we allow the command to be queued and executed again, which is harmless.
			}
			state = GCodeInputState::doingCode;
			break;

		case GCodeInputState::doingMCode112:
			if (c <= ' ' || c == ';')
			{
				// Emergency stop requested - perform it now
				reprap.EmergencyStop();
				reprap.GetGCodes().Reset();

				// But don't run it twice
				Reset();
				return true;
			}
			state = GCodeInputState::doingCode;
			break;

		case GCodeInputState::doingMCode122:
			// Only execute M122 here if there is no parameter
			if (c < ' ' || c == ';')
			{
				// Diagnostics requested - defer to the main task to avoid race conditions if called from another task
				reprap.DeferredDiagnostics(mtype);

				// But don't report them twice
				Reset();
				return true;
			}
			state = GCodeInputState::doingCode;
			break;
	}

	// A line end finishes the command in every state, including the ones that matched part of an urgent command, or the scanner would miss the next command
	if (c == 0 || c == '\r' || c == '\n')
	{
		state = GCodeInputState::idle;
	}
	return false;
}

// BufferedStreamGCodeInput methods

// How many bytes are ready to be passed to the GCodeBuffer. Any trailing characters that might still be
// completing an urgent command are held back, and how many that is follows directly from the scanner state
size_t BufferedStreamGCodeInput::BytesCached() const noexcept
{
	const size_t cached = RegularGCodeInput::BytesCached();
	if (writingFile)
	{
		return cached;					// raw file data is never an urgent command, so none of it is held back
	}

	// The scanner state encodes how far a potential M112/M122/M108/M25.1 has been matched, which is exactly how many
	// trailing characters must stay hidden until we know whether they form an urgent command
	size_t held;
	switch (state)
	{
	case GCodeInputState::doingMCode:
		held = 1;
		break;
	case GCodeInputState::doingMCode1:
	case GCodeInputState::doingMCode2:
		held = 2;
		break;
	case GCodeInputState::doingMCode10:
	case GCodeInputState::doingMCode11:
	case GCodeInputState::doingMCode12:
	case GCodeInputState::doingMCode24:
	case GCodeInputState::doingMCode25:
		held = 3;
		break;
	case GCodeInputState::doingMCode108:
	case GCodeInputState::doingMCode112:
	case GCodeInputState::doingMCode122:
	case GCodeInputState::doingMCode25dot:
		held = 4;
		break;
	case GCodeInputState::doingMCode251:
		held = 5;
		break;
	case GCodeInputState::doingMCode251space:
		held = 6;
		break;
	case GCodeInputState::doingMCode251S:
		held = 7;
		break;
	case GCodeInputState::doingMCode251S1:
		held = 8;
		break;
	default:
		held = 0;
		break;
	}
	return (cached > held) ? cached - held : 0;
}

// Read from the device into our ring buffer and scan for urgent commands (M112/M108/M122).
// Whenever we drop buffered data we must also drop any partly-received line in the GCodeBuffer, because the two together
// form one stream: dropping only our half would splice the remains of a truncated line onto the start of the next one
void BufferedStreamGCodeInput::Spin(GCodeBuffer& gb) noexcept
{
#if HAS_MASS_STORAGE
	// A binary upload cannot complete once its host has gone, and left alone it would write the next connection's commands into the file
	if (gb.IsWritingBinary() && UploadHostDisconnected())
	{
		AbortBinaryUpload(gb);
	}
# if CORE_USES_TINYUSB
	if (directReadAbandoned)
	{
		if (!device.IsConnected())
		{
			NoteAbandonedReadProgress();
		}
		ServiceAbandonedRead();
	}
# endif
#endif

	if (!device.IsConnected())
	{
		// The USB host has closed the port. Discard anything we have buffered as well as anything the USB driver has
		// received since, because it keeps filling its receive FIFO regardless of the DTR state. This way the first
		// command after the host reconnects starts cleanly instead of being appended to a leftover fragment
		while (device.available() != 0)
		{
			(void)device.read();
		}
#if HAS_MASS_STORAGE && CORE_USES_TINYUSB
		directTailLength = 0;
#endif
		if (RegularGCodeInput::BytesCached() != 0)
		{
			Reset();
		}
		gb.DiscardPartialLine();
		return;
	}

#if HAS_MASS_STORAGE && CORE_USES_TINYUSB
	// During a binary upload of known size, FillBinary takes the data straight from the device
	if (gb.IsWritingBinary() && gb.GetBinaryBytesRemaining() != 0)
	{
		return;
	}

	if (directTailLength != 0 || device.available() != 0)
#else
	if (device.available() != 0)
#endif
	{
		const size_t spaceLeft = BufferSpaceLeft();
		if (spaceLeft >= GCodeInputUSBReadThreshold)
		{
			const size_t maxToTransfer = min<size_t>(spaceLeft, GCodeInputBufferSize - writingPointer);
#if HAS_MASS_STORAGE && CORE_USES_TINYUSB
			const size_t endPointer = writingPointer + ReadInput(buffer + writingPointer, maxToTransfer);
#else
			const size_t endPointer = writingPointer + device.readBytes(buffer + writingPointer, maxToTransfer);
#endif
			while (writingPointer != endPointer)
			{
				const size_t pos = writingPointer;
				if (CheckForUrgentCommand(buffer[pos]))
				{
					// Urgent command handled and buffer was reset (writingPointer is now 0). The GCodeBuffer may already
					// hold the start of the same line (e.g. the line number in "N12 M122"), so throw that away too
					gb.DiscardPartialLine();

					// Copy any remaining bytes after the trigger into the buffer so that
					// following commands (e.g. M999 after M112) can still be processed
					const size_t remaining = endPointer - pos - 1;
					if (remaining != 0)
					{
						memmove(buffer, buffer + pos + 1, remaining);
						writingPointer = remaining;
					}
					return;
				}
				writingPointer++;
			}
			if (writingPointer == GCodeInputBufferSize)
			{
				writingPointer = 0;
			}
		}
	}
}

bool BufferedStreamGCodeInput::FillBuffer(GCodeBuffer *gb) noexcept
{
#if HAS_MASS_STORAGE && CORE_USES_TINYUSB
	// An upload of unknown size ends with the end-of-file string, so it has to go through the byte-by-byte path below
	if (gb->IsWritingBinary() && gb->GetBinaryBytesRemaining() != 0)
	{
		FillBinary(*gb);
		return false;
	}
#endif
	return StandardGCodeInput::FillBuffer(gb);
}

#if HAS_MASS_STORAGE

void BufferedStreamGCodeInput::NoteBinaryUpload(const char *_ecv_array directory, const char *_ecv_array fileName) noexcept
{
	(void)MassStorage::CombineName(uploadFilePath.GetRef(), directory, fileName);
# if CORE_USES_TINYUSB
	uploadDisconnectCount = device.GetDisconnectCount();
# endif
}

// The disconnect count also catches a host that went away and came back between two calls, which a slow SD card write makes possible
bool BufferedStreamGCodeInput::UploadHostDisconnected() const noexcept
{
# if CORE_USES_TINYUSB
	return !device.IsConnected() || device.GetDisconnectCount() != uploadDisconnectCount;
# else
	return !device.IsConnected();
# endif
}

// Abandon a binary upload whose host has gone. The partial file is deleted, as it is for a failed HTTP upload
void BufferedStreamGCodeInput::AbortBinaryUpload(GCodeBuffer& gb) noexcept
{
	gb.AbortBinaryUpload();
	Reset();
	gb.DiscardPartialLine();
# if CORE_USES_TINYUSB
	directTailLength = 0;
	if (device.IsDirectReadPending())
	{
		directReadAbandoned = true;
		NoteAbandonedReadProgress();
	}
	else
	{
		device.ReturnReceiveEndpoint();
	}
# endif
	(void)MassStorage::Delete(uploadFilePath.GetRef(), ErrorMessageMode::messageUnlessMissing);
	reprap.GetPlatform().MessageF(ErrorMessage, "USB upload of %s aborted because the host disconnected, partial file deleted\n", uploadFilePath.c_str());
	uploadFilePath.Clear();
}

# if CORE_USES_TINYUSB

// Take input for the ring buffer, from what followed the upload data in the last direct read before anything newer in the device
size_t BufferedStreamGCodeInput::ReadInput(char *_ecv_array dest, size_t maxBytes) noexcept
{
	if (directTailLength == 0)
	{
		return device.readBytes(dest, maxBytes);
	}

	const size_t bytesToCopy = min<size_t>(maxBytes, directTailLength);
	memcpy(dest, directTail, bytesToCopy);
	directTail += bytesToCopy;
	directTailLength -= bytesToCopy;
	return bytesToCopy;
}

// Note how much of the abandoned read is upload data. Whatever arrives while the host is disconnected still belongs to the
// upload, so this is called until the host is back. If the hardware cannot tell, the whole read counts as upload data
void BufferedStreamGCodeInput::NoteAbandonedReadProgress() noexcept
{
	if (!device.GetDirectReadProgress(abandonedUploadBytes))
	{
		abandonedUploadBytes = DirectBufferSize;
	}
}

// A direct read cannot be cancelled, so one that was pending when its upload was aborted is left to finish, which takes the
// next data the host sends or a bus reset. Its buffer starts with any upload data that arrived before the host went away.
// That part is dropped, because it could be the text of a G-code file, and the rest is the input of the next connection.
void BufferedStreamGCodeInput::ServiceAbandonedRead() noexcept
{
	size_t bytesRead;
	if (!device.IsReceiveEndpointTaken())
	{
		directReadAbandoned = false;
	}
	else if (device.IsDirectReadComplete(bytesRead))
	{
		if (bytesRead > abandonedUploadBytes)
		{
			directTail = directBuffers[directBufferInUse] + abandonedUploadBytes;
			directTailLength = bytesRead - abandonedUploadBytes;
		}
		device.ReturnReceiveEndpoint();
		directReadAbandoned = false;
	}
}

// Start receiving into the buffer selected by directBufferInUse. The length is rounded up to whole packets, because a read
// that ends inside a packet would overrun the buffer, so the last read of an upload can bring some of the G-code that follows it
bool BufferedStreamGCodeInput::StartDirectRead(FilePosition bytesExpected) noexcept
{
	const size_t packetSize = device.GetMaxPacketSize();
	return device.StartDirectRead(directBuffers[directBufferInUse], (bytesExpected >= DirectBufferSize) ? DirectBufferSize : (((size_t)bytesExpected + packetSize - 1) / packetSize) * packetSize);
}

// Pass the data of a binary upload of known size to the file. What is in the ring buffer and in the CDC FIFO arrived first and
// is written first. The rest is received straight into two alternating buffers, one of which keeps receiving while the other
// is written, so the data is neither copied through the FIFO nor held up by the time a file write takes.
void BufferedStreamGCodeInput::FillBinary(GCodeBuffer& gb) noexcept
{
	while (RegularGCodeInput::BytesCached() != 0)
	{
		const size_t contiguous = min<size_t>(RegularGCodeInput::BytesCached(), GCodeInputBufferSize - readingPointer);
		const size_t used = gb.WriteBinaryBlock(buffer + readingPointer, contiguous);
		readingPointer = (readingPointer + used) % GCodeInputBufferSize;
		if (!gb.IsWritingBinary() || used == 0)
		{
			return;								// upload finished, so any bytes left in the ring buffer are G-code
		}
	}

	if (directReadAbandoned)
	{
		ServiceAbandonedRead();
		if (directReadAbandoned)
		{
			return;
		}
	}

	if (directTailLength != 0)
	{
		const size_t used = gb.WriteBinaryBlock(reinterpret_cast<const char *>(directTail), directTailLength);
		directTail += used;
		directTailLength -= used;
		if (!gb.IsWritingBinary())
		{
			return;
		}
	}

	if (!device.IsReceiveEndpointTaken())
	{
		// The endpoint can only be taken once the CDC stream has filled its FIFO and stopped, so leave the FIFO alone until
		// then. If the FIFO already holds the rest of the upload there is nothing left to receive directly
		if ((FilePosition)device.available() < gb.GetBinaryBytesRemaining() && !device.TakeReceiveEndpoint())
		{
			return;
		}

		if (directBuffers[0] == nullptr)
		{
			for (uint8_t *_ecv_array _ecv_null& directBuffer : directBuffers)
			{
				directBuffer = static_cast<uint8_t *>(Tasks::AllocPermanent(DirectBufferSize, std::align_val_t(DirectBufferAlignment)));
			}
		}

		while (gb.IsWritingBinary())
		{
			const size_t bytesRead = device.readBytes(reinterpret_cast<char *>(directBuffers[0]), (size_t)min<FilePosition>(gb.GetBinaryBytesRemaining(), DirectBufferSize));
			if (bytesRead == 0)
			{
				break;
			}
			(void)gb.WriteBinaryBlock(reinterpret_cast<const char *>(directBuffers[0]), bytesRead);
		}

		if (device.IsReceiveEndpointTaken())
		{
			directBufferInUse = 0;
			(void)StartDirectRead(gb.GetBinaryBytesRemaining());
		}
		return;
	}

	const uint32_t startTime = millis();
	size_t bytesRead;
	while (device.IsDirectReadComplete(bytesRead))
	{
		const uint8_t *_ecv_array const data = directBuffers[directBufferInUse];
		const FilePosition remaining = gb.GetBinaryBytesRemaining();
		const size_t bytesToWrite = (size_t)min<FilePosition>(bytesRead, remaining);
		if (remaining > bytesToWrite)
		{
			// Start the next read before writing this one, so that the endpoint keeps receiving during the file write
			directBufferInUse ^= 1u;
			(void)StartDirectRead(remaining - bytesToWrite);
		}
		else
		{
			directTail = data + bytesToWrite;
			directTailLength = bytesRead - bytesToWrite;
			device.ReturnReceiveEndpoint();
		}

		(void)gb.WriteBinaryBlock(reinterpret_cast<const char *>(data), bytesToWrite);
		if (millis() - startTime >= MaxDirectMillisPerCall)
		{
			break;
		}
	}
}

# endif

#endif

#if defined(SERIAL_USB_DEVICE)

#include <Devices.h>

UsbGCodeInput::UsbGCodeInput(SerialCDC &_ecv_from dev, MessageType mt) noexcept
	: BufferedStreamGCodeInput(dev, mt), usbDevice(dev)
{
}

void UsbGCodeInput::Spin(GCodeBuffer& gb) noexcept
{
	// If the host disconnected, discard any buffered partial data
	if (!usbDevice.IsConnected() && BytesCached() != 0)
	{
		Reset();
	}
	BufferedStreamGCodeInput::Spin(gb);
}

#endif

// NetworkGCodeInput methods
void NetworkGCodeInput::Put(char c) noexcept
{
	if (BufferSpaceLeft() == 0)
	{
		// Don't let the buffer overflow if we run out of space
		return;
	}

	// Check for urgent commands (M112, M122, M108) while receiving new characters
	if (CheckForUrgentCommand(c))
	{
		return;
	}

	// Feed another character into the buffer
	buffer[writingPointer++] = c;
	if (writingPointer == GCodeInputBufferSize)
	{
		writingPointer = 0;
	}
}

bool NetworkGCodeInput::Put(const char *_ecv_array buf) noexcept
{
	const size_t len = strlen(buf) + 1;
	MutexLocker lock(bufMutex, 200);
	if (lock.IsAcquired())
	{
		// Only cache this if we have enough space left
		if (len <= BufferSpaceLeft())
		{
			for (size_t i = 0; i < len; i++)
			{
				Put(buf[i]);
			}
			return true;
		}
	}
	return false;
}

NetworkGCodeInput::NetworkGCodeInput(MessageType mt) noexcept : RegularGCodeInput(mt)
{
	bufMutex.Create("NetworkGCodeInput");
}

// Fill a GCodeBuffer with the last available G-code
bool NetworkGCodeInput::FillBuffer(GCodeBuffer *gb) noexcept /*override*/
{
	MutexLocker lock(bufMutex, 10);
	return lock.IsAcquired() && RegularGCodeInput::FillBuffer(gb);
}

#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES

// File-based G-code input source

// Reset this input. Should be called when the associated file is being closed
void FileGCodeInput::Reset() noexcept
{
	lastFileRead.Close();
	RegularGCodeInput::Reset();
}

// Reset this input. Should be called when a specific G-code or macro file is closed outside of the reading context
void FileGCodeInput::Reset(const FileData &file) noexcept
{
	if (lastFileRead == file)
	{
		Reset();
	}
}

// How many bytes have been cached for the given file?
size_t FileGCodeInput::FileBytesCached(const FileData &file) const noexcept
{
	return (lastFileRead == file) ? RegularGCodeInput::BytesCached() : 0;
}

// Read another chunk of G-codes from the file and return true if more data is available
GCodeInputReadResult FileGCodeInput::ReadFromFile(FileData &file) noexcept
{
	size_t bytesCached = RegularGCodeInput::BytesCached();

	// Keep track of the last file we read from
	if (lastFileRead != file)
	{
		if (lastFileRead.IsLive() && bytesCached > 0)
		{
			// Rewind back to the right position so we can resume at the right position later.
			// This may be necessary when nested macros are executed.
			lastFileRead.Seek(lastFileRead.GetPosition() - bytesCached);
		}

		RegularGCodeInput::Reset();
		bytesCached = 0;

		lastFileRead.CopyFrom(file);
	}

	// Read more from the file
	if (bytesCached < GCodeInputFileReadThreshold)
	{
		// Reset the read+write pointers for better performance if possible
		if (readingPointer == writingPointer)
		{
			readingPointer = writingPointer = 0;
		}

		// The code here used to read into a local buffer in blocks that are multiples of 4 bytes.
		// However, unless we can use a buffer of at least 512 bytes then that is redundant,
		// because the data will be copied via the sector buffer in FatFS anyway. So we don't do that any more.
		const int bytesRead = file.Read(buffer + writingPointer, min<size_t>(BufferSpaceLeft(), GCodeInputBufferSize - writingPointer));
		if (bytesRead < 0)
		{
			return GCodeInputReadResult::error;
		}
		if (bytesRead > 0)
		{
			writingPointer = (writingPointer + (size_t)bytesRead) % GCodeInputBufferSize;
			return GCodeInputReadResult::haveData;
		}
	}

	return (bytesCached > 0) ? GCodeInputReadResult::haveData : GCodeInputReadResult::noData;
}

#endif

// End
