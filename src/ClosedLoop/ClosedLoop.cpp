/*
 * ClosedLoop.cpp
 *
 *  Created on: 19 Mar 2021
 *      Author: Louis
 */

#include "ClosedLoop.h"

#if SUPPORT_CAN_EXPANSION

# include <Platform/RepRap.h>
# include <CAN/CanInterface.h>
# include <Platform/Platform.h>
# include <CanMessageFormats.h>
# include <Storage/MassStorage.h>
# include <CAN/ExpansionManager.h>
# include <GCodes/GCodeBuffer/GCodeBuffer.h>
# include <CAN/CanMessageGenericConstructor.h>

constexpr unsigned int MaxSamples = 65535;				// This comes from the fact CanMessageClosedLoopData->firstSampleNumber has a max value of 65535

static uint16_t rateRequested;							// The sampling rate
static uint8_t modeRequested;							// The sampling mode(immediate or on next move)
static uint32_t filterRequested;						// A filter for what data is collected
static DriverId deviceRequested;						// The driver being sampled
static uint8_t movementRequested;						// The movement to be made whilst recording
static volatile uint32_t numSamplesRequested;			// The number of samples to collect
static FileStore* volatile closedLoopFile = nullptr;	// This is non-null when the data collection is running, null otherwise

static unsigned int expectedRemoteSampleNumber = 0;
static CanAddress expectedRemoteBoardAddress = CanId::NoAddress;

static bool OpenDataCollectionFile(String<MaxFilenameLength> filename, unsigned int size) noexcept
{
	// Create the file
	FileStore * const f = MassStorage::OpenFile(filename.c_str(), OpenMode::write, size);
	if (f == nullptr) { return false; }

	// Write the header line
	{
		String<StringLength500> temp;
		temp.copy("Sample,Timestamp");
		if (filterRequested & CL_RECORD_RAW_ENCODER_READING)	{temp.cat(",Raw Encoder Reading");}
		if (filterRequested & CL_RECORD_CURRENT_MOTOR_STEPS)  	{temp.cat(",Measured Motor Steps");}
		if (filterRequested & CL_RECORD_TARGET_MOTOR_STEPS)  	{temp.cat(",Target Motor Steps");}
		if (filterRequested & CL_RECORD_CURRENT_ERROR) 			{temp.cat(",Current Error");}
		if (filterRequested & CL_RECORD_PID_CONTROL_SIGNAL)  	{temp.cat(",PID Control Signal");}
		if (filterRequested & CL_RECORD_PID_P_TERM)  			{temp.cat(",PID P Term");}
		if (filterRequested & CL_RECORD_PID_I_TERM)  			{temp.cat(",PID I Term");}
		if (filterRequested & CL_RECORD_PID_D_TERM)  			{temp.cat(",PID D Term");}
		if (filterRequested & CL_RECORD_STEP_PHASE)  			{temp.cat(",Measured Step Phase");}
		if (filterRequested & CL_RECORD_DESIRED_STEP_PHASE)  	{temp.cat(",Desired Step Phase");}
		if (filterRequested & CL_RECORD_PHASE_SHIFT)  			{temp.cat(",Phase Shift");}
		if (filterRequested & CL_RECORD_COIL_A_CURRENT) 		{temp.cat(",Coil A Current");}
		if (filterRequested & CL_RECORD_COIL_B_CURRENT) 		{temp.cat(",Coil B Current");}

		temp.Erase(temp.strlen(), 1);
		temp.cat("\n");
		f->Write(temp.c_str());
	}

	closedLoopFile = f;
	return true;
}

static void CloseDataCollectionFile() noexcept
{
	closedLoopFile->Truncate();				// truncate the file in case we didn't write all the preallocated space
	closedLoopFile->Close();
	closedLoopFile = nullptr;
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
		if (closedLoopFile == nullptr)
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
	if (closedLoopFile != nullptr)									// check if one is already happening
	{
		CloseDataCollectionFile();
		reply.copy("Closed loop data is already being collected; the file will now be closed");
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
	OpenDataCollectionFile(closedLoopFileName, preallocSize);

	// If no samples have been requested, return with an info message
	if (numSamplesRequested == 0)
	{
		CloseDataCollectionFile();
		reply.copy("no samples recorded");
		return GCodeResult::warning;
	}

	// Set up & start the CAN data transfer
	expectedRemoteSampleNumber = 0;
	expectedRemoteBoardAddress = deviceRequested.boardAddress;
	const GCodeResult rslt = CanInterface::StartClosedLoopDataCollection(deviceRequested, filterRequested, numSamplesRequested, rateRequested, movementRequested, modeRequested, gb, reply);
	if (rslt > GCodeResult::warning)
	{
		CloseDataCollectionFile();
		MassStorage::Delete(closedLoopFileName.c_str(), false);
	}
	return rslt;
}

// Process closed loop data received over CAN
void ClosedLoop::ProcessReceivedData(CanAddress src, const CanMessageClosedLoopData& msg, size_t msgLen) noexcept
{
	FileStore * const f = closedLoopFile;
	if (f != nullptr)
	{
		if (msg.firstSampleNumber != expectedRemoteSampleNumber)
		{
			f->Write("Data lost\n");
			CloseDataCollectionFile();
		}
		else
		{
			unsigned int numSamples = msg.numSamples;
			const size_t variableCount = msg.GetVariableCount();

			while (numSamples != 0)
			{
				// Compile the data
				String<StringLength256> currentLine;
				size_t sampleIndex = msg.numSamples - numSamples;
				//TODO use more intelligent formatting depending on the data type, to reduce the amount of data written
				currentLine.printf("%u", msg.firstSampleNumber + sampleIndex);
				for (size_t i = 0; i < variableCount; i++)
				{
					currentLine.catf(",%.2f", (double)msg.data[sampleIndex*variableCount + i]);
				}
				currentLine.cat("\n");

				// Write the data
				f->Write(currentLine.c_str());

				// Increment the working variables
				expectedRemoteSampleNumber++;
				numSamples--;
			}

			if (msg.lastPacket)
			{
				if (msg.overflowed)
				{
					f->Write("Buffer overflowed\n");
				}
				CloseDataCollectionFile();
			}
		}
	}
}

#endif

// End
