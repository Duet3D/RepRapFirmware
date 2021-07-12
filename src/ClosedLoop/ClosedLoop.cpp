/*
 * ClosedLoop.cpp
 *
 *  Created on: 19 Mar 2021
 *      Author: Louis
 */

#include "ClosedLoop.h"

#if SUPPORT_CAN_EXPANSION

#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include <Storage/MassStorage.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
//#include <RTOSIface/RTOSIface.h>
//#include <Platform/TaskPriorities.h>
//#include <Hardware/SharedSpi/SharedSpiDevice.h>

# include <CanMessageFormats.h>
# include <CAN/CanInterface.h>
# include <CAN/ExpansionManager.h>
# include <CAN/CanMessageGenericConstructor.h>

//constexpr uint8_t DefaultResolution = 10;
constexpr uint16_t DefaultSamplingRate = 1000;

static uint8_t modeRequested;							// The sampling mode
static DriverId deviceRequested;						// The driver being sampled
static volatile uint16_t numSamplesRequested;			// The number of samples to collect
//static uint8_t resolution = DefaultResolution;		// The resolution at which to collect the samples
static FileStore* volatile closedLoopFile = nullptr;	// This is non-null when the data collection is running, null otherwise

static unsigned int numRemoteOverflows;
static unsigned int expectedRemoteSampleNumber = 0;
static CanAddress expectedRemoteBoardAddress = CanId::NoAddress;

// Handle M569.5 - Collect closed loop data
GCodeResult ClosedLoop::StartDataCollection(DriverId driverId, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	deviceRequested = driverId;

	// Parse the GCODE params
	gb.MustSee('S');
	numSamplesRequested = min<uint32_t>(gb.GetUIValue(), 65535);
	gb.MustSee('A');
	modeRequested = gb.GetUIValue();

	// Check the chosen drive is a closed loop drive
	// TODO: Think of how we can do this. For now, just check it is remote
	if (!deviceRequested.IsRemote())
	{
		reply.copy("Drive is not in closed loop mode");
		return GCodeResult::error;
	}

	// Check if we are already collecting data
	if (closedLoopFile != nullptr)
	{
		reply.copy("Closed loop data is already being collected");
		return GCodeResult::error;
	}

	// Estimate how large the file will be
	const uint32_t preallocSize = 1000;	//TODO! numSamplesRequested * ((numAxes * (3 + GetDecimalPlaces(resolution))) + 4);

	// Create the filename
	String<MaxFilenameLength> closedLoopFileName;
	if (gb.Seen('F'))
	{
		String<StringLength50> temp;
		gb.GetQuotedString(temp.GetRef(), false);
		MassStorage::CombineName(closedLoopFileName.GetRef(), "0:/sys/closed-loop/", temp.c_str());
	}
	else
	{
		const time_t time = reprap.GetPlatform().GetDateTime();
		tm timeInfo;
		gmtime_r(&time, &timeInfo);
		closedLoopFileName.printf("0:/sys/closed-loop/%u_%04u-%02u-%02u_%02u.%02u.%02u.csv",
										(unsigned int) deviceRequested.boardAddress,
										timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday, timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);
	}

	// Create the file
	FileStore * const f = MassStorage::OpenFile(closedLoopFileName.c_str(), OpenMode::write, preallocSize);
	if (f == nullptr)
	{
		reply.copy("Failed to create closed loop data file");
		return GCodeResult::error;
	}

	// Write the header line
	{
		// TODO!
		String<StringLength50> temp;
		temp.printf("Sample,Encoder,Target,Control\n");
//		if (axes & 1u) { temp.cat(",X"); }
//		if (axes & 2u) { temp.cat(",Y"); }
//		if (axes & 4u) { temp.cat(",Z"); }
//		temp.cat('\n');
		f->Write(temp.c_str());
	}
	closedLoopFile = f;

	// If no samples have been requested, return with an info message
	if (numSamplesRequested == 0)
	{
		closedLoopFile->Truncate();
		closedLoopFile->Close();
		closedLoopFile = nullptr;
		reply.copy("no samples recorded");
		return GCodeResult::warning;
	}

	// Set up & start the CAN data transfer
	expectedRemoteSampleNumber = 0;
	expectedRemoteBoardAddress = deviceRequested.boardAddress;
	numRemoteOverflows = 0;
	const GCodeResult rslt = CanInterface::StartClosedLoopDataCollection(deviceRequested, numSamplesRequested, modeRequested, gb, reply);
	if (rslt > GCodeResult::warning)
	{
		closedLoopFile->Close();
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
//		if (msgLen < msg.GetActualDataLength())
//		{
//			f->Write("Received bad data\n");
//			f->Truncate();				// truncate the file in case we didn't write all the preallocated space
//			f->Close();
//			closedLoopFile = nullptr;
//		} else if
		if (msg.firstSampleNumber != expectedRemoteSampleNumber || src != expectedRemoteBoardAddress)
		{
			f->Write("Received mismatched data\n");
			String<StringLength50> temp;
			temp.printf("%d instead of %d\n", msg.firstSampleNumber, expectedRemoteSampleNumber);
			f->Write(temp.c_str());
			f->Truncate();				// truncate the file in case we didn't write all the preallocated space
			f->Close();
			closedLoopFile = nullptr;
		}
		else
		{
			unsigned int numSamples = msg.numSamples;
//			const unsigned int numAxes = (expectedRemoteAxes & 1u) + ((expectedRemoteAxes >> 1) & 1u) + ((expectedRemoteAxes >> 2) & 1u);
			size_t dataIndex = 0;
			uint16_t currentBits = 0;
			unsigned int bitsLeft = 0;
//			const unsigned int receivedResolution = msg.bitsPerSampleMinusOne + 1;
//			const uint16_t mask = (1u << receivedResolution) - 1;
//			const int decimalPlaces = GetDecimalPlaces(receivedResolution);
//			if (msg.overflowed)/
//			{
//				++numRemoteOverflows;
//			}

			while (numSamples != 0)
			{
				String<StringLength50> temp;
				temp.printf("%u", expectedRemoteSampleNumber);
				++expectedRemoteSampleNumber;

				// TODO: Parse data!
//				for (unsigned int axis = 0; axis < numAxes; ++axis)
//				{
//					// Extract one value from the message. A value spans at most two words in the buffer.
//					uint16_t val = currentBits;
//					if (bitsLeft >= receivedResolution)
//					{
//						bitsLeft -= receivedResolution;
//						currentBits >>= receivedResolution;
//					}
//					else
//					{
//						currentBits = msg.data[dataIndex++];
//						val |= currentBits << bitsLeft;
//						currentBits >>= receivedResolution - bitsLeft;
//						bitsLeft += 16 - receivedResolution;
//					}
//					val &= mask;
//
//					// Sign-extend it
//					if (val & (1u << (receivedResolution - 1)))
//					{
//						val |= ~mask;
//					}
//
//					// Convert it to a float number of g
//					const float fVal = (float)(int16_t)val/(float)(1u << GetBitsAfterPoint(receivedResolution));
//
//					// Append it to the buffer
//					temp.catf(",%.*f", decimalPlaces, (double)fVal);
//				}

				temp.catf(",%d", msg.encoderData[0]);
				temp.catf(",%d", msg.targetData[0]);
				temp.catf(",%d\n", msg.controlData[0]);

				f->Write(temp.c_str());
				--numSamples;
			}

			if (msg.lastPacket)
			{
//				String<StringLength50> temp;
//				temp.printf("Rate %u, overflows %u\n", msg.actualSampleRate, numRemoteOverflows);
//				f->Write(temp.c_str());
				f->Truncate();				// truncate the file in case we didn't write all the preallocated space
				f->Close();
				closedLoopFile = nullptr;
			}
		}
	}
}

#endif



















//static unsigned int expectedRemoteSampleNumber = 0;
//static CanAddress expectedRemoteBoardAddress = CanId::NoAddress;
//static uint8_t expectedRemoteAxes;
//static unsigned int numRemoteOverflows;
//
//
//// Get the number of binary digits after the decimal point
//static inline unsigned int GetBitsAfterPoint(uint8_t dataResolution) noexcept
//{
//	return dataResolution - 2;							// assumes the range is +/- 2g
//}
//
//// Get the number of decimal places that we should use when we print each acceleration value
//static unsigned int GetDecimalPlaces(uint8_t dataResolution) noexcept
//{
//	return (GetBitsAfterPoint(dataResolution) >= 11) ? 4 : (GetBitsAfterPoint(dataResolution) >= 8) ? 3 : 2;
//}
//
//// Local accelerometer handling
//
//#include "LIS3DH.h"
//
//constexpr uint16_t DefaultSamplingRate = 1000;
//constexpr uint8_t DefaultResolution = 10;
//
//constexpr size_t AccelerometerTaskStackWords = 400;			// big enough to handle printf and file writes
//static Task<AccelerometerTaskStackWords> *accelerometerTask;
//
//static LIS3DH *accelerometer = nullptr;
//
//static uint16_t samplingRate = DefaultSamplingRate;
//static volatile uint16_t numSamplesRequested;
//static uint8_t resolution = DefaultResolution;
//static uint8_t orientation = 20;							// +Z -> +Z, +X -> +X
//static volatile uint8_t axesRequested;
//static FileStore* volatile accelerometerFile = nullptr;		// this is non-null when the accelerometer is running, null otherwise
//static uint8_t axisLookup[3];
//static bool axisInverted[3];
//
//static IoPort spiCsPort;
//static IoPort irqPort;
//
//[[noreturn]] void AccelerometerTaskCode(void*) noexcept
//{
//	for (;;)
//	{
//		TaskBase::Take();
//		FileStore * const f = accelerometerFile;			// capture volatile variable
//		if (f != nullptr)
//		{
//			// Collect and write the samples
//			unsigned int samplesWritten = 0;
//			unsigned int samplesWanted = numSamplesRequested;
//			unsigned int numOverflows = 0;
//			const uint16_t mask = (1u << resolution) - 1;
//			const int decimalPlaces = GetDecimalPlaces(resolution);
//
//			if (accelerometer->StartCollecting(axesRequested))
//			{
//				uint16_t dataRate = 0;
//				do
//				{
//					const uint16_t *data;
//					bool overflowed;
//					unsigned int samplesRead = accelerometer->CollectData(&data, dataRate, overflowed);
//					if (samplesRead == 0)
//					{
//						// samplesRead == 0 indicates an error, e.g. no interrupt
//						samplesWanted = 0;
//						if (f != nullptr)
//						{
//							f->Write("Failed to collect data from accelerometer\n");
//							f->Truncate();				// truncate the file in case we didn't write all the preallocated space
//							f->Close();
//							accelerometerFile = nullptr;
//						}
//						break;
//					}
//					else
//					{
//						if (overflowed)
//						{
//							++numOverflows;
//						}
//						if (samplesWritten == 0)
//						{
//							// The first sample taken after waking up is inaccurate, so discard it
//							--samplesRead;
//							data += 3;
//						}
//						if (samplesRead >= samplesWanted)
//						{
//							samplesRead = samplesWanted;
//						}
//
//						while (samplesRead != 0)
//						{
//							// Write a row of data
//							String<StringLength50> temp;
//							temp.printf("%u", samplesWritten);
//
//							for (unsigned int axis = 0; axis < 3; ++axis)
//							{
//								if (axesRequested & (1u << axis))
//								{
//									uint16_t dataVal = data[axisLookup[axis]];
//									if (axisInverted[axis])
//									{
//										dataVal = (dataVal == 0x8000) ? ~dataVal : ~dataVal + 1;
//									}
//									dataVal >>= (16u - resolution);					// data from LIS3DH is left justified
//
//									// Sign-extend it
//									if (dataVal & (1u << (resolution - 1)))
//									{
//										dataVal |= ~mask;
//									}
//
//									// Convert it to a float number of g
//									const float fVal = (float)(int16_t)dataVal/(float)(1u << GetBitsAfterPoint(resolution));
//
//									// Append it to the buffer
//									temp.catf(",%.*f", decimalPlaces, (double)fVal);
//								}
//							}
//
//							data += 3;
//
//							temp.cat('\n');
//							f->Write(temp.c_str());
//
//							--samplesRead;
//							--samplesWanted;
//							++samplesWritten;
//						}
//					}
//				} while (samplesWanted != 0);
//
//				if (f != nullptr)
//				{
//					String<StringLength50> temp;
//					temp.printf("Rate %u, overflows %u\n", dataRate, numOverflows);
//					f->Write(temp.c_str());
//				}
//			}
//			else if (f != nullptr)
//			{
//				f->Write("Failed to start accelerometer\n");
//			}
//
//			if (f != nullptr)
//			{
//				f->Truncate();				// truncate the file in case we didn't write all the preallocated space
//				f->Close();
//				accelerometerFile = nullptr;
//			}
//
//			accelerometer->StopCollecting();
//
//			// Wait for another command
//			accelerometerFile = nullptr;
//		}
//	}
//}
//
//// Translate the orientation returning true if successful
//static bool TranslateOrientation(uint32_t input) noexcept
//{
//	if (input >= 70u) { return false; }
//	const uint8_t xDigit = input % 10u;
//	if (xDigit >= 7u) { return false; }
//	const uint8_t zDigit = input / 10u;
//	const uint8_t xOrientation = xDigit & 0x03;
//	const uint8_t zOrientation = zDigit & 0x03;
//	if (xOrientation == 3u || zOrientation == 3u || xOrientation == zOrientation) { return false; }
//	const uint8_t xInverted = xDigit & 0x04;
//	const uint8_t zInverted = zDigit & 0x04;
//	uint8_t yInverted = xInverted ^ zInverted;
//
//	// Calculate the Y orientation. We must have axes 0, 1 and 2 so they must add up to 3.
//	const uint8_t yOrientation = 3u - xOrientation - zOrientation;
//
//	// The total number of inversions must be even if the cyclic order of the axes is 012, odd if it is 210 (can we prove this?)
//	if ((xOrientation + 1) % 3 != yOrientation)
//	{
//		yInverted ^= 0x04;									// we need an odd number of axis inversions
//	}
//
//	// Now fill in the axis table
//	axisLookup[xOrientation] = 0;
//	axisInverted[xOrientation] = xInverted;
//	axisLookup[yOrientation] = 1;
//	axisInverted[yOrientation] = yInverted;
//	axisLookup[zOrientation] = 2;
//	axisInverted[zOrientation] = zInverted;
//	return true;
//}
//
//// Deal with M955
//GCodeResult Accelerometers::ConfigureAccelerometer(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
//{
//	gb.MustSee('P');
//	DriverId device = gb.GetDriverId();
//
//# if SUPPORT_CAN_EXPANSION
//	if (device.IsRemote())
//	{
//		CanMessageGenericConstructor cons(M955Params);
//		cons.PopulateFromCommand(gb);
//		return cons.SendAndGetResponse(CanMessageType::accelerometerConfig, device.boardAddress, reply);
//	}
//# endif
//
//	if (device.localDriver != 0)
//	{
//		reply.copy("Only one local accelerometer is supported");
//		return GCodeResult::error;
//	}
//
//	// No need for task lock here because this function and the M956 function are called only by the MAIN task
//	if (accelerometerFile != nullptr)
//	{
//		reply.copy("Cannot reconfigure accelerometer while it is collecting data");
//		return GCodeResult::error;
//	}
//
//	bool seen = false;
//	if (gb.Seen('C'))
//	{
//		seen = true;
//
//		// Creating a new accelerometer. First delete any existing accelerometer.
//		DeleteObject(accelerometer);
//		spiCsPort.Release();
//		irqPort.Release();
//
//		IoPort * const ports[2] = { &spiCsPort, &irqPort };
//		const PinAccess access[2] = { PinAccess::write1, PinAccess::read };
//		if (IoPort::AssignPorts(gb, reply, PinUsedBy::sensor, 2, ports, access) != 2)
//		{
//			spiCsPort.Release();				// in case it was allocated
//			return GCodeResult::error;
//		}
//
//		const uint32_t spiFrequency = (gb.Seen('Q')) ? gb.GetLimitedUIValue('Q', 500000, 10000001) : DefaultAccelerometerSpiFrequency;
//		auto temp = new LIS3DH(SharedSpiDevice::GetMainSharedSpiDevice(), spiFrequency, spiCsPort.GetPin(), irqPort.GetPin());
//		if (temp->CheckPresent())
//		{
//			accelerometer = temp;
//			if (accelerometerTask == nullptr)
//			{
//				accelerometerTask = new Task<AccelerometerTaskStackWords>;
//				accelerometerTask->Create(AccelerometerTaskCode, "ACCEL", nullptr, TaskPriority::Accelerometer);
//			}
//		}
//		else
//		{
//			delete temp;
//			reply.copy("Accelerometer not found on specified port");
//			return GCodeResult::error;
//		}
//	}
//	else if (accelerometer == nullptr)
//	{
//		reply.copy("Accelerometer not found");
//		return GCodeResult::error;
//	}
//
//	uint32_t temp32;
//	if (gb.TryGetLimitedUIValue('R', temp32, seen, 17))
//	{
//		resolution = temp32;
//	}
//
//	if (gb.TryGetLimitedUIValue('S', temp32, seen, 10000))
//	{
//		samplingRate = temp32;
//	}
//
//	if (seen)
//	{
//		if (!accelerometer->Configure(samplingRate, resolution))
//		{
//			reply.copy("Failed to configure accelerometer");
//			return GCodeResult::error;
//		}
//		(void)TranslateOrientation(orientation);
//	}
//
//	if (gb.Seen('I'))
//	{
//		seen = true;
//		if (!TranslateOrientation(gb.GetUIValue()))
//		{
//			reply.copy("Bad orientation parameter");
//			return GCodeResult::error;
//		}
//	}
//
//# if SUPPORT_CAN_EXPANSION
//	reply.printf("Accelerometer %u:%u with orientation %u samples at %uHz with %u-bit resolution, SPI frequency %" PRIu32,
//					CanInterface::GetCanAddress(), 0, orientation, samplingRate, resolution, accelerometer->GetFrequency());
//# else
//	reply.printf("Accelerometer %u with orientation %u samples at %uHz with %u-bit resolution, SPI frequency %" PRIu32,
//					0, orientation, samplingRate, resolution, accelerometer->GetFrequency());
//# endif
//	return GCodeResult::ok;
//}
//
//
//
//

// End
