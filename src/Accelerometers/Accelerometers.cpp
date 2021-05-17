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
#include <Hardware/SharedSpi/SharedSpiDevice.h>

#if SUPPORT_CAN_EXPANSION
# include <CanMessageFormats.h>
# include <CAN/CanInterface.h>
# include <CAN/ExpansionManager.h>
# include <CAN/CanMessageGenericConstructor.h>
#endif

#ifdef DUET3_ATE
# include <Duet3Ate.h>
#endif

constexpr uint32_t DefaultAccelerometerSpiFrequency = 2000000;

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

static FileStore *CreateFile(CanAddress src, uint8_t axesToWrite, uint32_t preallocSize) noexcept
{
	const time_t time = reprap.GetPlatform().GetDateTime();
	tm timeInfo;
	gmtime_r(&time, &timeInfo);
	String<StringLength50> temp;
	temp.printf("0:/sys/accelerometer/%u_%04u-%02u-%02u_%02u.%02u.%02u.csv",
					(unsigned int)src,
					timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday, timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);
	FileStore * const f = MassStorage::OpenFile(temp.c_str(), OpenMode::write, preallocSize);
	if (f != nullptr)
	{
		temp.printf("Sample");
		if (axesToWrite & 1u) { temp.cat(",X"); }
		if (axesToWrite & 2u) { temp.cat(",Y"); }
		if (axesToWrite & 4u) { temp.cat(",Z"); }
		temp.cat('\n');
		f->Write(temp.c_str());
	}
	return f;
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

	static FileStore *f = nullptr;
	static unsigned int expectedSampleNumber = 0;
	static CanAddress currentBoard = CanId::NoAddress;
	static uint8_t axesReceived;
	static unsigned int numOverflows;

	if (msg.firstSampleNumber == 0)
	{
		// Close any existing file
		if (f != nullptr)
		{
			f->Write("Data incomplete\n");
			f->Truncate();				// truncate the file in case we didn't write all the preallocated space
			f->Close();
			f = nullptr;
		}

		currentBoard = src;
		axesReceived = msg.axes;
		expectedSampleNumber = 0;
		numOverflows = 0;
		f = CreateFile(src, msg.axes, 0);
	}

	if (f != nullptr)
	{
		if (msgLen < msg.GetActualDataLength())
		{
			f->Write("Received bad data\n");
			f->Truncate();				// truncate the file in case we didn't write all the preallocated space
			f->Close();
			f = nullptr;
		}
		else if (msg.axes != axesReceived || msg.firstSampleNumber != expectedSampleNumber || src != currentBoard)
		{
			f->Write("Received mismatched data\n");
			f->Truncate();				// truncate the file in case we didn't write all the preallocated space
			f->Close();
			f = nullptr;
		}
		else
		{
			unsigned int numSamples = msg.numSamples;
			const unsigned int numAxes = (axesReceived & 1u) + ((axesReceived >> 1) & 1u) + ((axesReceived >> 2) & 1u);
			size_t dataIndex = 0;
			uint16_t currentBits = 0;
			unsigned int bitsLeft = 0;
			const unsigned int receivedResolution = msg.bitsPerSampleMinusOne + 1;
			const uint16_t mask = (1u << receivedResolution) - 1;
			const int decimalPlaces = GetDecimalPlaces(receivedResolution);
			if (msg.overflowed)
			{
				++numOverflows;
			}

			while (numSamples != 0)
			{
				String<StringLength50> temp;
				temp.printf("%u", expectedSampleNumber);
				++expectedSampleNumber;

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
				temp.printf("Rate %u, overflows %u\n", msg.actualSampleRate, numOverflows);
				f->Write(temp.c_str());
				f->Truncate();				// truncate the file in case we didn't write all the preallocated space
				f->Close();
				f = nullptr;
				expectedSampleNumber = 0;
			}
		}
	}
}

#endif

// Local accelerometer handling

#include "LIS3DH.h"

constexpr uint16_t DefaultSamplingRate = 1000;
constexpr uint8_t DefaultResolution = 10;

constexpr size_t AccelerometerTaskStackWords = 400;			// big enough to handle printf and file writes
static Task<AccelerometerTaskStackWords> *accelerometerTask;

static LIS3DH *accelerometer = nullptr;

static uint16_t samplingRate = DefaultSamplingRate;
static volatile uint16_t numSamplesRequested;
static uint8_t resolution = DefaultResolution;
static uint8_t orientation = 20;							// +Z -> +Z, +X -> +X
static volatile uint8_t axesRequested;
static volatile bool running = false;
static uint8_t axisLookup[3];
static bool axisInverted[3];

static IoPort spiCsPort;
static IoPort irqPort;

[[noreturn]] void AccelerometerTaskCode(void*) noexcept
{
	for (;;)
	{
		TaskBase::Take();
		if (running)
		{
			// Calculate the approximate file size so that we can preallocate storage to reduce the risk of overflow
			const unsigned int numAxes = (axesRequested & 1u) + ((axesRequested >> 1) & 1u) + ((axesRequested >> 2) & 1u);
			const uint32_t preallocSize = numSamplesRequested * ((numAxes * (3 + GetDecimalPlaces(resolution))) + 4);
			FileStore *f = CreateFile(CanInterface::GetCanAddress(), axesRequested, preallocSize);
			if (f != nullptr)
			{
				// Collect and write the samples
				unsigned int samplesWritten = 0;
				unsigned int samplesWanted = numSamplesRequested;
				unsigned int numOverflows = 0;
				const uint16_t mask = (1u << resolution) - 1;
				const int decimalPlaces = GetDecimalPlaces(resolution);

				if (accelerometer->StartCollecting(axesRequested))
				{
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
							if (f != nullptr)
							{
								f->Write("Failed to collect data from accelerometer\n");
								f->Truncate();				// truncate the file in case we didn't write all the preallocated space
								f->Close();
								f = nullptr;
							}
							break;
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
				else if (f != nullptr)
				{
					f->Write("Failed to start accelerometer\n");
				}

				if (f != nullptr)
				{
					f->Truncate();				// truncate the file in case we didn't write all the preallocated space
					f->Close();
					f = nullptr;
				}
			}

			accelerometer->StopCollecting();

			// Wait for another command
			running = false;
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
		return cons.SendAndGetResponse(CanMessageType::accelerometerConfig, device.boardAddress, reply);
	}
# endif

	if (device.localDriver != 0)
	{
		reply.copy("Only one local accelerometer is supported");
		return GCodeResult::error;
	}

	// No need for task lock here because this function and the M956 function are called only by the MAIN task
	if (running)
	{
		reply.copy("Cannot reconfigure accelerometer while it is collecting data");
		return GCodeResult::error;
	}

	bool seen = false;
	if (gb.Seen('C'))
	{
		seen = true;

		// Creating a new accelerometer. First delete any existing accelerometer.
		LIS3DH *temp = nullptr;
		std::swap(temp, accelerometer);
		delete temp;
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
		temp = new LIS3DH(SharedSpiDevice::GetMainSharedSpiDevice(), spiFrequency, spiCsPort.GetPin(), irqPort.GetPin());
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
		if (!TranslateOrientation(gb.GetUIValue()))
		{
			reply.copy("Bad orientation parameter");
			return GCodeResult::error;
		}
	}

#if SUPPORT_CAN_EXPANSION
	reply.printf("Accelerometer %u:%u with orientation %u samples at %uHz with %u-bit resolution", CanInterface::GetCanAddress(), 0, orientation, samplingRate, resolution);
#else
	reply.printf("Accelerometer %u with orientation %u samples at %uHz with %u-bit resolution", 0, orientation, samplingRate, resolution);
#endif
	return GCodeResult::ok;
}

// Deal with M956
GCodeResult Accelerometers::StartAccelerometer(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	gb.MustSee('P');
	const DriverId device = gb.GetDriverId();
	gb.MustSee('S');
	const uint16_t numSamples = min<uint32_t>(gb.GetUIValue(), 65535);
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

# if SUPPORT_CAN_EXPANSION
	if (device.IsRemote())
	{
		return CanInterface::StartAccelerometer(device, axes, numSamples, mode, gb, reply);
	}
# endif

	// No need for task lock here because this function and the M955 function are called only by the MAIN task
	if (device.localDriver != 0 || accelerometer == nullptr)
	{
		reply.copy("Accelerometer not found");
		return GCodeResult::error;
	}

	if (running)
	{
		reply.copy("Accelerometer is already collecting data");
		return GCodeResult::error;
	}

	axesRequested = axes;
	numSamplesRequested = numSamples;
	running = true;
	(void)mode;									// TODO implement mode
	accelerometerTask->Give();
	return GCodeResult::ok;
}

#endif

// End
