/*
 * ClosedLoop.cpp
 *
 *  Created on: 19 Mar 2021
 *      Author: Louis
 */

#include "ClosedLoop.h"

#if SUPPORT_CAN_EXPANSION && (HAS_MASS_STORAGE || HAS_SBC_INTERFACE)

# include <Platform/RepRap.h>
# include <CAN/CanInterface.h>
# include <Platform/Platform.h>
# include <CanMessageFormats.h>
# include <Storage/MassStorage.h>
# include <CAN/ExpansionManager.h>
# include <GCodes/GCodeBuffer/GCodeBuffer.h>
# include <CAN/CanMessageGenericConstructor.h>
# include <General/Portability.h>
# include <atomic>

constexpr unsigned int MaxSamples = 65535;				// This comes from the fact CanMessageClosedLoopData->firstSampleNumber has a max value of 65535
constexpr uint32_t DataReceiveTimeout = 5000;			// Data receive timeout in milliseconds

static uint16_t rateRequested;							// The sampling rate
static uint8_t modeRequested;							// The sampling mode(immediate or on next move)
static uint32_t filterRequested;						// A filter for what data is collected
static size_t dataBytesPerSample;						// How many bytes there will be in each sample
static DriverId deviceRequested;						// The driver being sampled
static uint8_t movementRequested;						// The movement to be made whilst recording
static std::atomic<uint32_t> whenDataLastReceived;
static std::atomic<uint32_t> numSamplesRequested;			// The number of samples to collect
static std::atomic<FileStore*> closedLoopFile = nullptr;	// This is non-null when the data collection is running, null otherwise

static unsigned int expectedRemoteSampleNumber = 0;
static CanAddress expectedRemoteBoardAddress = CanId::NoAddress;

static bool OpenDataCollectionFile(String<MaxFilenameLength> filename, unsigned int size) noexcept
{
	// Create the file
	FileStore * const f = MassStorage::OpenFile(filename.c_str(), OpenMode::write, size);
	if (f == nullptr) { return false; }

	// Write the header line
	{
		static constexpr const char *headings[16] =
		{
			",Raw Encoder Reading",
			",Measured Motor Steps",
			",Target Motor Steps",
			",Current Error",
			",PID Control Signal",
			",PID P Term",
			",PID I Term",
			",PID D Term",

			// The next two are out of order in the filter bits, they come later on
			",PID V Term",
			",PID A Term",

			",Measured Step Phase",
			",Desired Step Phase",
			",Phase Shift",
			",Coil A Current",
			",Coil B Current",
			",Unknown",
		};
		String<StringLength500> temp;
		temp.copy("Sample,Timestamp");
		uint16_t filter = (filterRequested & (CL_RECORD_CURRENT_STEP_PHASE - 1))
						| ((filterRequested & (CL_RECORD_CURRENT_STEP_PHASE | CL_RECORD_DESIRED_STEP_PHASE | CL_RECORD_PHASE_SHIFT | CL_RECORD_COIL_A_CURRENT | CL_RECORD_COIL_B_CURRENT)) << 2)
						| ((filterRequested & (CL_RECORD_PID_V_TERM | CL_RECORD_PID_A_TERM)) >> 5);
		for (unsigned int i = 0; filter != 0; ++i)
		{
			if (filter & 1u)
			{
				temp.cat(headings[i]);
			}
			filter >>= 1;
		}
		temp.cat("\n");
		f->Write(temp.c_str());							// this call could result in the file becoming invalidated
	}

	whenDataLastReceived = millis();					// prevent another request closing the file
	closedLoopFile.store(f);
	return true;
}

// Close the data collection file. Avoid a race between the two tasks that access it.
static void CloseDataCollectionFile() noexcept
{
	FileStore *const f = closedLoopFile.exchange(nullptr);
	if (f != nullptr)
	{
		f->Truncate();				// truncate the file in case we didn't write all the preallocated space
		f->Close();
		reprap.GetExpansion().AddClosedLoopRun(expectedRemoteBoardAddress, expectedRemoteSampleNumber);
	}
}

// Handle M569.5 - Collect closed loop data
GCodeResult ClosedLoop::StartDataCollection(DriverId driverId, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	// Check what the user wants - record or report
	// Record - A number of samples (Snn) is given to record
	// Report - Samples (Snn) is not given
	bool recording = false;
	uint32_t parsedS;
	gb.TryGetLimitedUIValue('S', parsedS, recording, MaxSamples + 1);
	if (!recording)
	{
		// No S parameter was given so this is a request for the recording status
		if (closedLoopFile.load() == nullptr)
		{
			reply.copy("Closed loop data is not being collected");
			return GCodeResult::warning;							// looks like the closed loop plugin relies on this being a warning
		}
		else
		{
			reply.printf("Collecting sample: %u/%u", expectedRemoteSampleNumber, (unsigned int)numSamplesRequested);
			return GCodeResult::ok;
		}
	}

	// If we get here then the user is requesting a recording
	if (closedLoopFile.load() != nullptr)							// check if one is already happening
	{
		const uint32_t wlr = whenDataLastReceived;					// load this volatile variable before calling millis()
		if (millis() - wlr >= DataReceiveTimeout)
		{
			CloseDataCollectionFile();								// this case is to allow us to reset if data collection stalls
			reply.copy("Closed loop data collection timed out, closing file");
		}
		else
		{
			reply.copy("Closed loop data is already being collected");
		}
		return GCodeResult::error;
	}

	// Parse the additional parameters
	bool seen = false;
	uint32_t parsedA = 0, parsedD = 0, parsedR = 0, parsedV = 0;

	gb.TryGetLimitedUIValue('A', parsedA, seen, 2);					// valid collection modes are 0 and 1
	gb.TryGetUIValue('D', parsedD, seen);
	gb.TryGetLimitedUIValue('R', parsedR, seen, std::numeric_limits<uint16_t>::max() + 1);
	gb.TryGetUIValue('V', parsedV, seen);

	// Validation passed - store the values
	modeRequested = parsedA;
	rateRequested = parsedR;
	filterRequested = parsedD;
	dataBytesPerSample = ClosedLoopSampleLength(filterRequested);
	deviceRequested = driverId;
	movementRequested = parsedV;
	numSamplesRequested = parsedS;

	// Estimate how large the file will be
	const unsigned int numVariables = Bitmap<uint32_t>(filterRequested).CountSetBits() + 1;		// 1 extra for time stamp
	const uint32_t preallocSize = numSamplesRequested * ((numVariables * 8) + 4);				// assume format "xxx.xxx," for most samples

	// Create the file
	String<StringLength50> tempFilename;
	if (gb.Seen('F'))
	{
		gb.GetQuotedString(tempFilename.GetRef(), false);
	}
	else
	{
		// Create default filename as none was provided
		const time_t time = reprap.GetPlatform().GetDateTime();
		tm timeInfo;
		gmtime_r(&time, &timeInfo);
		tempFilename.printf("0:/sys/closed-loop/%u_%04u-%02u-%02u_%02u.%02u.%02u.csv",
						(unsigned int) deviceRequested.boardAddress,
						timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday, timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);
	}

	String<MaxFilenameLength> closedLoopFileName;
	MassStorage::CombineName(closedLoopFileName.GetRef(), "0:/sys/closed-loop/", tempFilename.c_str());
	if (!OpenDataCollectionFile(closedLoopFileName, preallocSize))
	{
		reply.copy("failed to create data collection file");
		return GCodeResult::error;
	}

	// Set up the expected CAN address and next point number before we do anything that might call CloseDataFile
	expectedRemoteSampleNumber = 0;
	expectedRemoteBoardAddress = deviceRequested.boardAddress;

	// If no samples have been requested, return with an info message
	if (numSamplesRequested == 0)
	{
		CloseDataCollectionFile();
		reply.copy("no samples recorded");
		return GCodeResult::warning;
	}

	// Set up & start the CAN data transfer
	const GCodeResult rslt = CanInterface::StartClosedLoopDataCollection(deviceRequested, filterRequested, numSamplesRequested, rateRequested, movementRequested, modeRequested, gb, reply);
	if (rslt > GCodeResult::warning)
	{
		CloseDataCollectionFile();
		(void)MassStorage::Delete(closedLoopFileName.GetRef(), ErrorMessageMode::messageAlways);
	}
	return rslt;
}

// Process closed loop data received over CAN
void ClosedLoop::ProcessReceivedData(CanAddress src, const CanMessageClosedLoopData& msg, size_t msgLen) noexcept
{
	FileStore * const f = closedLoopFile.load();
	if (f != nullptr)
	{
		whenDataLastReceived = millis();
		if (msg.firstSampleNumber != expectedRemoteSampleNumber)
		{
			f->Write("Data lost\n");
			CloseDataCollectionFile();
		}
		else if (dataBytesPerSample * msg.numSamples > CanMessageClosedLoopData::GetNumDataBytes(msgLen))
		{
			f->Write("Bad data received\n");
			CloseDataCollectionFile();
		}
		else
		{
			const uint8_t *dataPtr = msg.data;
			for (unsigned int sampleIndex = 0; sampleIndex < msg.numSamples; ++sampleIndex)
			{
				// Compile the data
				String<StringLength256> currentLine;
				currentLine.printf("%u,%.2f", msg.firstSampleNumber + sampleIndex, (double)FetchLEF32(dataPtr));	// sample number and time stamp
				if (filterRequested & CL_RECORD_RAW_ENCODER_READING)	{ currentLine.catf(",%" PRIi32,	FetchLEI32(dataPtr)); }
				if (filterRequested & CL_RECORD_CURRENT_MOTOR_STEPS)  	{ currentLine.catf(",%.2f", (double)FetchLEF32(dataPtr)); }
				if (filterRequested & CL_RECORD_TARGET_MOTOR_STEPS)  	{ currentLine.catf(",%.2f", (double)FetchLEF32(dataPtr)); }
				if (filterRequested & CL_RECORD_CURRENT_ERROR) 			{ currentLine.catf(",%.2f", (double)FetchLEF32(dataPtr)); }
				if (filterRequested & CL_RECORD_PID_CONTROL_SIGNAL)  	{ currentLine.catf(",%.1f", (double)FetchLEF16(dataPtr)); }
				if (filterRequested & CL_RECORD_PID_P_TERM)  			{ currentLine.catf(",%.1f", (double)FetchLEF16(dataPtr)); }
				if (filterRequested & CL_RECORD_PID_I_TERM)  			{ currentLine.catf(",%.1f", (double)FetchLEF16(dataPtr)); }
				if (filterRequested & CL_RECORD_PID_D_TERM)  			{ currentLine.catf(",%.1f", (double)FetchLEF16(dataPtr)); }
				if (filterRequested & CL_RECORD_PID_V_TERM)  			{ currentLine.catf(",%.1f", (double)FetchLEF16(dataPtr)); }
				if (filterRequested & CL_RECORD_PID_A_TERM)  			{ currentLine.catf(",%.1f", (double)FetchLEF16(dataPtr)); }
				if (filterRequested & CL_RECORD_CURRENT_STEP_PHASE)  	{ currentLine.catf(",%u",	FetchLEU16(dataPtr)); }
				if (filterRequested & CL_RECORD_DESIRED_STEP_PHASE)  	{ currentLine.catf(",%u",	FetchLEU16(dataPtr)); }
				if (filterRequested & CL_RECORD_PHASE_SHIFT)  			{ currentLine.catf(",%u",	FetchLEU16(dataPtr)); }
				if (filterRequested & CL_RECORD_COIL_A_CURRENT) 		{ currentLine.catf(",%d",	FetchLEI16(dataPtr)); }
				if (filterRequested & CL_RECORD_COIL_B_CURRENT) 		{ currentLine.catf(",%d",	FetchLEI16(dataPtr)); }
				currentLine.cat("\n");

				// Write the data
				f->Write(currentLine.c_str());

				// Increment the working variables
				expectedRemoteSampleNumber++;
			}

			if (msg.lastPacket)
			{
				if (msg.overflowed)
				{
					f->Write("Buffer overflowed\n");
				}
				if (msg.badSample)
				{
					f->Write("Data contains bad sample(s)\n");
				}
				CloseDataCollectionFile();
			}
		}
	}
}

#endif

// End
