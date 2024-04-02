/*
 * Accelerometers.cpp
 *
 *  Created on: 19 Mar 2021
 *      Author: David
 */

#include "Accelerometers.h"

#if SUPPORT_ACCELEROMETERS

#include <Storage/MassStorage.h>
#include <Platform/Platform.h>
#include <Platform/RepRap.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <RTOSIface/RTOSIface.h>
#include <Platform/TaskPriorities.h>
#include <Hardware/Spi/SharedSpiDevice.h>
#include <AppNotifyIndices.h>

#if SUPPORT_CAN_EXPANSION
# include <CanMessageFormats.h>
# include <CAN/CanInterface.h>
# include <CAN/ExpansionManager.h>
# include <CAN/CanMessageGenericConstructor.h>
# include <CanMessageGenericTables.h>
#endif

#ifdef DUET3_ATE
# include <Duet3Ate.h>
#endif

constexpr uint32_t DefaultAccelerometerSpiFrequency = 2000000;

#if SUPPORT_CAN_EXPANSION

static unsigned int expectedRemoteSampleNumber = 0;
static CanAddress expectedRemoteBoardAddress = CanId::NoAddress;
static uint8_t expectedRemoteAxes;
static unsigned int numRemoteOverflows;

#endif

// Get the number of binary digits after the decimal point
static inline unsigned int GetBitsAfterPoint(uint8_t dataResolution) noexcept
{
	return dataResolution - 2;							// assumes the range is +/- 2g
}

// Get the number of decimal places that we should use when we print each acceleration value
static unsigned int GetDecimalPlaces(uint8_t dataResolution) noexcept
{
	return (GetBitsAfterPoint(dataResolution) >= 11) ? 4 : (GetBitsAfterPoint(dataResolution) >= 8) ? 3 : 2;
}

// Local accelerometer handling

#include "LISAccelerometer.h"

constexpr size_t AccelerometerTaskStackWords = 400;			// big enough to handle printf and file writes
static Task<AccelerometerTaskStackWords> *accelerometerTask;

static LISAccelerometer *accelerometer = nullptr;

static uint16_t samplingRate = 0;							// 0 means use the default
static volatile uint32_t numSamplesRequested;
static uint8_t resolution = DefaultAccelerometerResolution;
static uint8_t orientation = DefaultAccelerometerOrientation;
static volatile uint8_t axesRequested;
static FileStore* volatile accelerometerFile = nullptr;		// this is non-null when the accelerometer is running, null otherwise
static unsigned int numLocalRunsCompleted = 0;
static unsigned int lastRunNumSamplesReceived = 0;
static uint8_t axisLookup[3];
static bool axisInverted[3];
static volatile bool successfulStart = false;
static volatile bool failedStart = false;

static IoPort spiCsPort;
static IoPort irqPort;

// Add a local accelerometer run
static void AddLocalAccelerometerRun(unsigned int numDataPoints) noexcept
{
	lastRunNumSamplesReceived = numDataPoints;
	++numLocalRunsCompleted;
	reprap.BoardsUpdated();
}

static uint8_t TranslateAxes(uint8_t axes) noexcept
{
	uint8_t rslt = 0;
	for (unsigned int i = 0; i < 3; ++i)
	{
		if (axes & (1u << i))
		{
			rslt |= 1u << axisLookup[i];
		}
	}
	return rslt;
}

[[noreturn]] void AccelerometerTaskCode(void*) noexcept
{
	for (;;)
	{
		TaskBase::TakeIndexed(NotifyIndices::AccelerometerDataCollector);
		FileStore * f = accelerometerFile;			// capture volatile variable
		if (f != nullptr)
		{
			// Collect and write the samples
			unsigned int samplesWritten = 0;
			unsigned int samplesWanted = numSamplesRequested;
			unsigned int numOverflows = 0;
			const uint16_t mask = (1u << resolution) - 1;
			const int decimalPlaces = GetDecimalPlaces(resolution);
			bool recordFailedStart = false;

			if (accelerometer->StartCollecting(TranslateAxes(axesRequested)))
			{
				successfulStart = true;
				uint16_t dataRate = 0;
				do
				{
					const uint16_t *data;
					bool overflowed;
					unsigned int samplesRead = accelerometer->CollectData(&data, dataRate, overflowed);
					if (samplesRead == 0)
					{
						// samplesRead == 0 indicates an error, e.g. no interrupt
						samplesWanted = 0;
						f->Write("Failed to collect data from accelerometer\n");
						f->Truncate();				// truncate the file in case we didn't write all the preallocated space
						f->Close();
						f = nullptr;
						AddLocalAccelerometerRun(0);
					}
					else
					{
						if (overflowed)
						{
							++numOverflows;
						}
						if (samplesWritten == 0)
						{
							// The first sample taken after waking up is inaccurate, so discard it
							--samplesRead;
							data += 3;
						}
						if (samplesRead >= samplesWanted)
						{
							samplesRead = samplesWanted;
						}

						while (samplesRead != 0)
						{
							// Write a row of data
							String<StringLength50> temp;
							temp.printf("%u", samplesWritten);

							for (unsigned int axis = 0; axis < 3; ++axis)
							{
								if (axesRequested & (1u << axis))
								{
									uint16_t dataVal = data[axisLookup[axis]];
									if (axisInverted[axis])
									{
										dataVal = (dataVal == 0x8000) ? ~dataVal : ~dataVal + 1;
									}
									dataVal >>= (16u - resolution);					// data from LIS3DH is left justified

									// Sign-extend it
									if (dataVal & (1u << (resolution - 1)))
									{
										dataVal |= ~mask;
									}

									// Convert it to a float number of g
									const float fVal = (float)(int16_t)dataVal/(float)(1u << GetBitsAfterPoint(resolution));

									// Append it to the buffer
									temp.catf(",%.*f", decimalPlaces, (double)fVal);
								}
							}

							data += 3;

							temp.cat('\n');
							f->Write(temp.c_str());

							--samplesRead;
							--samplesWanted;
							++samplesWritten;
						}
					}
				} while (samplesWanted != 0);

				if (f != nullptr)
				{
					String<StringLength50> temp;
					temp.printf("Rate %u, overflows %u\n", dataRate, numOverflows);
					f->Write(temp.c_str());
				}
			}
			else
			{
				recordFailedStart = true;
				if (f != nullptr)
				{
					f->Write("Failed to start accelerometer\n");
				}
			}

			if (f != nullptr)
			{
				f->Truncate();				// truncate the file in case we didn't write all the preallocated space
				f->Close();
				AddLocalAccelerometerRun(samplesWritten);
			}

			accelerometer->StopCollecting();

			// Wait for another command
			accelerometerFile = nullptr;
			if (recordFailedStart)
			{
				failedStart = true;
			}
		}
	}
}

// Translate the orientation returning true if successful
static bool TranslateOrientation(uint32_t input) noexcept
{
	if (input >= 70u) { return false; }
	const uint8_t xDigit = input % 10u;
	if (xDigit >= 7u) { return false; }
	const uint8_t zDigit = input / 10u;
	const uint8_t xOrientation = xDigit & 0x03;
	const uint8_t zOrientation = zDigit & 0x03;
	if (xOrientation == 3u || zOrientation == 3u || xOrientation == zOrientation) { return false; }
	const uint8_t xInverted = xDigit & 0x04;
	const uint8_t zInverted = zDigit & 0x04;
	uint8_t yInverted = xInverted ^ zInverted;

	// Calculate the Y orientation. We must have axes 0, 1 and 2 so they must add up to 3.
	const uint8_t yOrientation = 3u - xOrientation - zOrientation;

	// The total number of inversions must be even if the cyclic order of the axes is 012, odd if it is 210 (can we prove this?)
	if ((xOrientation + 1) % 3 != yOrientation)
	{
		yInverted ^= 0x04;									// we need an odd number of axis inversions
	}

	// Now fill in the axis table
	axisLookup[xOrientation] = 0;
	axisInverted[xOrientation] = xInverted;
	axisLookup[yOrientation] = 1;
	axisInverted[yOrientation] = yInverted;
	axisLookup[zOrientation] = 2;
	axisInverted[zOrientation] = zInverted;
	return true;
}

// Deal with M955
GCodeResult Accelerometers::ConfigureAccelerometer(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	gb.MustSee('P');
	DriverId device = gb.GetDriverId();

# if SUPPORT_CAN_EXPANSION
	if (device.IsRemote())
	{
		CanMessageGenericConstructor cons(M955Params);
		cons.PopulateFromCommand(gb);
		const GCodeResult rslt = cons.SendAndGetResponse(CanMessageType::accelerometerConfig, device.boardAddress, reply);
		if (rslt <= GCodeResult::warning && gb.Seen('I'))
		{
			const uint8_t remoteOrientation = (uint8_t)gb.GetUIValue();
			reprap.GetExpansion().SaveAccelerometerOrientation(device.GetBoardAddress(), (uint8_t)remoteOrientation);
		}
		return rslt;
	}
# endif

	if (device.localDriver != 0)
	{
		reply.copy("Only one local accelerometer is supported");
		return GCodeResult::error;
	}

	// No need for task lock here because this function and the M956 function are called only by the MAIN task
	if (accelerometerFile != nullptr)
	{
		reply.copy("Cannot reconfigure accelerometer while it is collecting data");
		return GCodeResult::error;
	}

	bool seen = false;
	if (gb.Seen('C'))
	{
		seen = true;

		// Creating a new accelerometer. First delete any existing accelerometer.
		DeleteObject(accelerometer);
		spiCsPort.Release();
		irqPort.Release();

		IoPort * const ports[2] = { &spiCsPort, &irqPort };
		const PinAccess access[2] = { PinAccess::write1, PinAccess::read };
		if (IoPort::AssignPorts(gb, reply, PinUsedBy::sensor, 2, ports, access) != 2)
		{
			spiCsPort.Release();				// in case it was allocated
			return GCodeResult::error;
		}

		const uint32_t spiFrequency = (gb.Seen('Q')) ? gb.GetLimitedUIValue('Q', 500000, 10000001) : DefaultAccelerometerSpiFrequency;
		auto temp = new LISAccelerometer(SharedSpiDevice::GetMainSharedSpiDevice(), spiFrequency, spiCsPort.GetPin(), irqPort.GetPin());
		if (temp->CheckPresent())
		{
			accelerometer = temp;
			if (accelerometerTask == nullptr)
			{
				accelerometerTask = new Task<AccelerometerTaskStackWords>;
				accelerometerTask->Create(AccelerometerTaskCode, "ACCEL", nullptr, TaskPriority::Accelerometer);
			}
		}
		else
		{
			delete temp;
			reply.copy("Accelerometer not found on specified port");
			return GCodeResult::error;
		}
	}
	else if (accelerometer == nullptr)
	{
		reply.copy("Accelerometer not found");
		return GCodeResult::error;
	}

	uint32_t temp32;
	if (gb.TryGetLimitedUIValue('R', temp32, seen, 17))
	{
		resolution = temp32;
	}

	if (gb.TryGetLimitedUIValue('S', temp32, seen, 10000))
	{
		samplingRate = temp32;
	}

	if (seen)
	{
		if (!accelerometer->Configure(samplingRate, resolution))
		{
			reply.copy("Failed to configure accelerometer");
			return GCodeResult::error;
		}
		(void)TranslateOrientation(orientation);
	}

	if (gb.Seen('I'))
	{
		seen = true;
		const uint32_t localOrientation = gb.GetUIValue();
		if (TranslateOrientation(localOrientation))
		{
			orientation = localOrientation;
		}
		else
		{
			reply.copy("Bad orientation parameter");
			return GCodeResult::error;
		}
	}

	if (!seen)
	{
# if SUPPORT_CAN_EXPANSION
		reply.printf("Accelerometer %u:%u type %s with orientation %u samples at %uHz with %u-bit resolution, SPI frequency %" PRIu32,
						CanInterface::GetCanAddress(), 0, accelerometer->GetTypeName(), orientation, samplingRate, resolution, accelerometer->GetFrequency());
# else
		reply.printf("Accelerometer %u type %s with orientation %u samples at %uHz with %u-bit resolution, SPI frequency %" PRIu32,
						0, accelerometer->GetTypeName(), orientation, samplingRate, resolution, accelerometer->GetFrequency());
# endif
	}
	return GCodeResult::ok;
}

// Deal with M956
GCodeResult Accelerometers::StartAccelerometer(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	gb.MustSee('P');
	const DriverId device = gb.GetDriverId();
	gb.MustSee('S');
	const uint32_t numSamples = gb.GetUIValue();
	gb.MustSee('A');
	const uint8_t mode = gb.GetUIValue();

	uint8_t axes = 0;
	if (gb.Seen('X')) { axes |= 1u << 0; }
	if (gb.Seen('Y')) { axes |= 1u << 1; }
	if (gb.Seen('Z')) { axes |= 1u << 2; }

	if (axes == 0)
	{
		axes = 0x07;						// default to all three axes
	}

	// Check that we have an accelerometer
	if (
# if SUPPORT_CAN_EXPANSION
		!device.IsRemote() &&
# endif
		(device.localDriver != 0 || accelerometer == nullptr)
	   )
	{
		reply.copy("Accelerometer not found");
		return GCodeResult::error;
	}

	// No need for task lock here because this function and the M955 function are called only by the MAIN task
	if (accelerometerFile != nullptr)
	{
		reply.copy("Accelerometer is already collecting data");
		return GCodeResult::error;
	}

	// Set up the collection parameters in case the accelerometer task wakes up early
	axesRequested = axes;
	numSamplesRequested = numSamples;
	(void)mode;									// TODO implement mode

	// Create the file for saving the data. First calculate the approximate file size so that we can preallocate storage to reduce the risk of overflow.
	const unsigned int numAxes = (axesRequested & 1u) + ((axesRequested >> 1) & 1u) + ((axesRequested >> 2) & 1u);
	const uint32_t preallocSize = numSamplesRequested * ((numAxes * (3 + GetDecimalPlaces(resolution))) + 4);

	String<MaxFilenameLength> accelerometerFileName;
	if (gb.Seen('F'))
	{
		String<StringLength50> temp;
		gb.GetQuotedString(temp.GetRef(), false);
		MassStorage::CombineName(accelerometerFileName.GetRef(), "0:/sys/accelerometer/", temp.c_str());
	}
	else
	{
		const time_t time = reprap.GetPlatform().GetDateTime();
		tm timeInfo;
		gmtime_r(&time, &timeInfo);
		accelerometerFileName.printf("0:/sys/accelerometer/%u_%04u-%02u-%02u_%02u.%02u.%02u.csv",
# if SUPPORT_CAN_EXPANSION
										(unsigned int)device.boardAddress,
# else
										0,
# endif
										timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday, timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);
	}
	FileStore * const f = MassStorage::OpenFile(accelerometerFileName.c_str(), OpenMode::write, preallocSize);
	if (f == nullptr)
	{
		reply.copy("Failed to create accelerometer data file");
# if SUPPORT_CAN_EXPANSION
		if (device.IsRemote())
		{
			reprap.GetExpansion().AddAccelerometerRun(device.boardAddress, 0);
		}
		else
# endif
		{
			AddLocalAccelerometerRun(0);
		}
		return GCodeResult::error;
	}

	// Write the header line to the file
	{
		String<StringLength50> temp;
		temp.printf("Sample");
		if (axes & 1u) { temp.cat(",X"); }
		if (axes & 2u) { temp.cat(",Y"); }
		if (axes & 4u) { temp.cat(",Z"); }
		temp.cat('\n');
		f->Write(temp.c_str());
	}

# if SUPPORT_CAN_EXPANSION
	if (device.IsRemote())
	{
		expectedRemoteSampleNumber = 0;
		expectedRemoteBoardAddress = device.boardAddress;
		expectedRemoteAxes = axes;
		numRemoteOverflows = 0;

		accelerometerFile = f;
		const GCodeResult rslt = CanInterface::StartAccelerometer(device, axes, numSamples, mode, gb, reply);
		if (rslt > GCodeResult::warning)
		{
			accelerometerFile->Close();
			accelerometerFile = nullptr;
			(void)MassStorage::Delete(accelerometerFileName.GetRef(), ErrorMessageMode::messageAlways);
			reprap.GetExpansion().AddAccelerometerRun(device.boardAddress, 0);
		}
		return rslt;
	}
# endif

	successfulStart = false;
	failedStart = false;
	accelerometerFile = f;
	accelerometerTask->Give(NotifyIndices::AccelerometerDataCollector);
	const uint32_t startTime = millis();
	do
	{
		delay(5);
		if (successfulStart)
		{
			return GCodeResult::ok;
		}
	} while (!failedStart && millis() - startTime < 1000);

	reply.copy("Failed to start accelerometer data collection");
	if (accelerometer->HasInterruptError())
	{
		reply.cat(": INT1 error");
	}
	if (accelerometerFile != nullptr)
	{
		accelerometerFile->Close();
		accelerometerFile = nullptr;
		(void)MassStorage::Delete(accelerometerFileName.GetRef(), ErrorMessageMode::messageAlways);
	}
	return GCodeResult::error;
}

bool Accelerometers::HasLocalAccelerometer() noexcept
{
	return accelerometer != nullptr;
}

unsigned int Accelerometers::GetLocalAccelerometerDataPoints() noexcept
{
	return lastRunNumSamplesReceived;
}

unsigned int Accelerometers::GetLocalAccelerometerRuns() noexcept
{
	return numLocalRunsCompleted;
}

uint8_t Accelerometers::GetLocalAccelerometerOrientation() noexcept
{
	return orientation;
}

void Accelerometers::Exit() noexcept
{
	if (accelerometerTask != nullptr)
	{
		accelerometerTask->TerminateAndUnlink();
		accelerometerTask = nullptr;
	}
}

#if SUPPORT_CAN_EXPANSION

// Process accelerometer data received over CAN
void Accelerometers::ProcessReceivedData(CanAddress src, const CanMessageAccelerometerData& msg, size_t msgLen) noexcept
{
# ifdef DUET3_ATE
	if (Duet3Ate::ProcessAccelerometerData(src, msg, msgLen))
	{
		return;								// ATE has processed the data
	}
# endif

	FileStore * const f = accelerometerFile;
	if (f != nullptr)
	{
		if (msgLen < msg.GetActualDataLength())
		{
			f->Write("Received bad data\n");
			f->Truncate();				// truncate the file in case we didn't write all the preallocated space
			f->Close();
			accelerometerFile = nullptr;
			reprap.GetExpansion().AddAccelerometerRun(src, 0);
		}
		else if (msg.axes != expectedRemoteAxes || msg.firstSampleNumber != expectedRemoteSampleNumber || src != expectedRemoteBoardAddress)
		{
			f->Write("Received mismatched data\n");
			f->Truncate();				// truncate the file in case we didn't write all the preallocated space
			f->Close();
			accelerometerFile = nullptr;
			reprap.GetExpansion().AddAccelerometerRun(src, 0);
		}
		else
		{
			unsigned int numSamples = msg.numSamples;
			const unsigned int numAxes = (expectedRemoteAxes & 1u) + ((expectedRemoteAxes >> 1) & 1u) + ((expectedRemoteAxes >> 2) & 1u);
			size_t dataIndex = 0;
			uint16_t currentBits = 0;
			unsigned int bitsLeft = 0;
			const unsigned int receivedResolution = msg.bitsPerSampleMinusOne + 1;
			const uint16_t mask = (1u << receivedResolution) - 1;
			const int decimalPlaces = GetDecimalPlaces(receivedResolution);
			if (msg.overflowed)
			{
				++numRemoteOverflows;
			}

			while (numSamples != 0)
			{
				String<StringLength50> temp;
				temp.printf("%u", expectedRemoteSampleNumber);
				++expectedRemoteSampleNumber;

				for (unsigned int axis = 0; axis < numAxes; ++axis)
				{
					// Extract one value from the message. A value spans at most two words in the buffer.
					uint16_t val = currentBits;
					if (bitsLeft >= receivedResolution)
					{
						bitsLeft -= receivedResolution;
						currentBits >>= receivedResolution;
					}
					else
					{
						currentBits = msg.data[dataIndex++];
						val |= currentBits << bitsLeft;
						currentBits >>= receivedResolution - bitsLeft;
						bitsLeft += 16 - receivedResolution;
					}
					val &= mask;

					// Sign-extend it
					if (val & (1u << (receivedResolution - 1)))
					{
						val |= ~mask;
					}

					// Convert it to a float number of g
					const float fVal = (float)(int16_t)val/(float)(1u << GetBitsAfterPoint(receivedResolution));

					// Append it to the buffer
					temp.catf(",%.*f", decimalPlaces, (double)fVal);
				}

				temp.cat('\n');
				f->Write(temp.c_str());
				--numSamples;
			}

			if (msg.lastPacket)
			{
				String<StringLength50> temp;
				temp.printf("Rate %u, overflows %u\n", (unsigned int)msg.actualSampleRate, numRemoteOverflows);
				f->Write(temp.c_str());
				f->Truncate();				// truncate the file in case we didn't write all the preallocated space
				f->Close();
				accelerometerFile = nullptr;
				reprap.GetExpansion().AddAccelerometerRun(src, expectedRemoteSampleNumber);
			}
		}
	}
}

#endif

#endif

// End
