#include "TemperatureSensor.h"
#include "Thermistor.h"
#include "ThermocoupleSensor31855.h"
#include "ThermocoupleSensor31856.h"
#include "RtdSensor31865.h"
#include "CurrentLoopTemperatureSensor.h"
#include "LinearAnalogSensor.h"
#include "RemoteSensor.h"
#include "BME280.h"
#include "GCodes/GCodeBuffer/GCodeBuffer.h"

#if SUPPORT_REMOTE_COMMANDS
# include <CanMessageGenericParser.h>
#endif

#if HAS_CPU_TEMP_SENSOR
# include "CpuTemperatureSensor.h"
#endif

#if SUPPORT_DHT_SENSOR
# include "DhtSensor.h"
#endif

#if HAS_SMART_DRIVERS
# include "TmcDriverTemperatureSensor.h"
#endif

#if SUPPORT_CAN_EXPANSION
# include "CAN/CanInterface.h"
#endif

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(TemperatureSensor, __VA_ARGS__)

constexpr ObjectModelTableEntry TemperatureSensor::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. TemperatureSensor members
	{ "lastReading",	OBJECT_MODEL_FUNC(self->lastTemperature, 2), 						ObjectModelEntryFlags::live },
	{ "name",			OBJECT_MODEL_FUNC(self->sensorName), 								ObjectModelEntryFlags::none },
	{ "offsetAdj",		OBJECT_MODEL_FUNC(self->offsetAdjustment, 1), 						ObjectModelEntryFlags::none },
	{ "slopeAdj",		OBJECT_MODEL_FUNC(self->slopeAdjustment, 3), 						ObjectModelEntryFlags::none },
	{ "state",			OBJECT_MODEL_FUNC(((TemperatureError)self->lastResult).ToString()),	ObjectModelEntryFlags::none },
	{ "type",			OBJECT_MODEL_FUNC(self->GetShortSensorType()), 						ObjectModelEntryFlags::none },
};

constexpr uint8_t TemperatureSensor::objectModelTableDescriptor[] = { 1, 6 };

DEFINE_GET_OBJECT_MODEL_TABLE(TemperatureSensor)

#endif

// Constructor
TemperatureSensor::TemperatureSensor(unsigned int sensorNum, const char *_ecv_array t) noexcept
	: next(nullptr), sensorNumber(sensorNum), sensorType(t), sensorName(nullptr),
	  lastTemperature(0.0), whenLastRead(0), lastResult(TemperatureError::notReady), lastRealError(TemperatureError::ok) {}

// Virtual destructor
TemperatureSensor::~TemperatureSensor() noexcept
{
	delete sensorName;
}

// Return the latest temperature reading
TemperatureError TemperatureSensor::GetLatestTemperature(float& t) noexcept
{
	// We must read whenLastRead *before* we call millis(). Otherwise, a task switch to the heater task could occur after we call millis and before we read whenLastRead,
	// so that when we read whenLastRead its value is greater than the result from millis().
	const uint32_t wlr = whenLastRead;
	if (millis() - wlr > GetTemperatureReadingTimeout())
	{
		lastTemperature = BadErrorTemperature;
		lastResult = TemperatureError::timeout;
	}
	t = lastTemperature;
	return lastResult;
}

// Return the value of an additional output. Default implementation for sensors with no additional output.
TemperatureError TemperatureSensor::GetAdditionalOutput(float& t, uint8_t outputNumber) noexcept
{
	t = BadErrorTemperature;
	return TemperatureError::invalidOutputNumber;
}

// Set the name - normally called only once, so we allow heap memory to be allocated
void TemperatureSensor::SetSensorName(const char *_ecv_array _ecv_null newName) noexcept
{
	// Change the heater name in a thread-safe manner
	const char *_ecv_array _ecv_null oldName = sensorName;
	sensorName = nullptr;
	delete oldName;

	if (newName != nullptr && strlen(newName) != 0)
	{
		char *_ecv_array const temp = new char[strlen(newName) + 1];
		strcpy(temp, newName);
		sensorName = temp;
	}
}

// Default implementation of Configure, for sensors that have no configurable parameters
GCodeResult TemperatureSensor::Configure(GCodeBuffer& gb, const StringRef& reply, bool& changed) THROWS(GCodeException)
{
	ConfigureCommonParameters(gb, changed);
	if (!changed && !gb.Seen('Y'))
	{
		// No parameters were provided, so report the current configuration
		CopyBasicDetails(reply);
	}
	return GCodeResult::ok;
}

#if SUPPORT_REMOTE_COMMANDS

// Default implementation of Configure, for sensors that have no configurable parameters
GCodeResult TemperatureSensor::Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept
{
	bool seen = parser.HasParameter('Y');
	ConfigureCommonParameters(parser, seen);
	if (!seen)
	{
		// No parameters were provided, so report the current configuration
		CopyBasicDetails(reply);
	}
	return GCodeResult::ok;
}

#endif

void TemperatureSensor::CopyBasicDetails(const StringRef& reply) const noexcept
{
	reply.printf("Sensor %u", sensorNumber);
	if (sensorName != nullptr)
	{
		reply.catf(" (%s)", sensorName);
	}
	reply.catf(" type %s", sensorType);
	AppendPinDetails(reply);
	reply.catf(", last error %s, ", ((TemperatureError)lastRealError).ToString());
	if (slopeAdjustment != 0.0 || offsetAdjustment != 0.0)
	{
		reply.catf("offset adjustment %.1f, slope adjustment %.3f, adjusted reading %.1f", (double)offsetAdjustment, (double)slopeAdjustment, (double)lastTemperature);
	}
	else
	{
		reply.catf("reading %.1f", (double)lastTemperature);
	}
}

// Configure parameters that are common to all sensors and stored in the base class
void TemperatureSensor::ConfigureCommonParameters(GCodeBuffer& gb, bool& seen) THROWS(GCodeException)
{
	String<MaxHeaterNameLength> buf;
	if (gb.TryGetQuotedString('A', buf.GetRef(), seen))
	{
		SetSensorName(buf.c_str());
	}
	gb.TryGetLimitedFValue('U', offsetAdjustment, seen, -20.0, 20.0);
	gb.TryGetLimitedFValue('V', slopeAdjustment, seen, -0.2, 0.2);
}

#if SUPPORT_REMOTE_COMMANDS

// Configure the reading adjustment parameters
void TemperatureSensor::ConfigureCommonParameters(const CanMessageGenericParser& parser, bool& seen) noexcept
{
	if (parser.GetFloatParam('U', offsetAdjustment)) { seen = true; }
	if (parser.GetFloatParam('V', slopeAdjustment)) { seen = true; }
}

#endif

void TemperatureSensor::SetResult(float t, TemperatureError rslt) noexcept
{
	lastResult = rslt;
	lastTemperature = (t * (1.0 + slopeAdjustment)) + offsetAdjustment;
	whenLastRead = millis();
	if (rslt != TemperatureError::ok)
	{
		lastRealError = rslt;
	}
}

// This version is used for unsuccessful readings only
void TemperatureSensor::SetResult(TemperatureError rslt) noexcept
{
	lastResult = rslt;
	lastRealError = rslt;
	lastTemperature = BadErrorTemperature;
	whenLastRead = millis();
}

#if SUPPORT_CAN_EXPANSION

// Get the expansion board address. Overridden for remote sensors.
CanAddress TemperatureSensor::GetBoardAddress() const noexcept
{
	return CanInterface::GetCanAddress();
}

// Update the temperature, if it is a remote sensor. Overridden in class RemoteSensor.
void TemperatureSensor::UpdateRemoteTemperature(CanAddress src, const CanSensorReport& report) noexcept
{
	// Nothing to do here. This function is overridden in class RemoteSensor.
}

#endif

// Factory method
#if SUPPORT_CAN_EXPANSION
TemperatureSensor *_ecv_from TemperatureSensor::Create(unsigned int sensorNum, CanAddress boardAddress, const char *_ecv_array typeName, const StringRef& reply) noexcept
#else
TemperatureSensor *_ecv_from TemperatureSensor::Create(unsigned int sensorNum, const char *_ecv_array typeName, const StringRef& reply) noexcept
#endif
{
	TemperatureSensor *_ecv_from ts;
#if SUPPORT_CAN_EXPANSION
	if (boardAddress != CanInterface::GetCanAddress())
	{
		ts = new RemoteSensor(sensorNum, boardAddress);
	}
	else
#endif
	if (ReducedStringEquals(typeName, Thermistor::TypeNameThermistor))
	{
		ts = new Thermistor(sensorNum, false);
	}
	else if (ReducedStringEquals(typeName, Thermistor::TypeNamePT1000))
	{
		ts = new Thermistor(sensorNum, true);
	}
	else if (ReducedStringEquals(typeName, LinearAnalogSensor::TypeName))
	{
		ts = new LinearAnalogSensor(sensorNum);
	}
#if SUPPORT_SPI_SENSORS
	else if (ReducedStringEquals(typeName, ThermocoupleSensor31855::TypeName))
	{
		ts = new ThermocoupleSensor31855(sensorNum);
	}
	else if (ReducedStringEquals(typeName, ThermocoupleSensor31856::TypeName))
	{
		ts = new ThermocoupleSensor31856(sensorNum);
	}
	else if (ReducedStringEquals(typeName, RtdSensor31865::TypeName))
	{
		ts = new RtdSensor31865(sensorNum);
	}
	else if (ReducedStringEquals(typeName, CurrentLoopTemperatureSensor::TypeName))
	{
		ts = new CurrentLoopTemperatureSensor(sensorNum);
	}
#endif
#if SUPPORT_DHT_SENSOR
	else if (ReducedStringEquals(typeName, DhtTemperatureSensor::TypeNameDht21))
	{
		ts = new DhtTemperatureSensor(sensorNum, DhtSensorType::Dht21);
	}
	else if (ReducedStringEquals(typeName, DhtTemperatureSensor::TypeNameDht22))
	{
		ts = new DhtTemperatureSensor(sensorNum, DhtSensorType::Dht22);
	}
	else if (ReducedStringEquals(typeName, DhtHumiditySensor::TypeName))
	{
		ts = new DhtHumiditySensor(sensorNum);
	}
#endif
#if SUPPORT_BME280
	else if (ReducedStringEquals(typeName, BME280TemperatureSensor::TypeName))
	{
		ts = new BME280TemperatureSensor(sensorNum);
	}
	else if (ReducedStringEquals(typeName, BME280PressureSensor::TypeName))
	{
		ts = new BME280PressureSensor(sensorNum);
	}
	else if (ReducedStringEquals(typeName, BME280HumiditySensor::TypeName))
	{
		ts = new BME280HumiditySensor(sensorNum);
	}
#endif
#if HAS_CPU_TEMP_SENSOR
	else if (ReducedStringEquals(typeName, CpuTemperatureSensor::TypeName))
	{
		ts = new CpuTemperatureSensor(sensorNum);
	}
#endif
#if HAS_SMART_DRIVERS
	else if (ReducedStringEquals(typeName, TmcDriverTemperatureSensor::PrimaryTypeName))
	{
		ts = new TmcDriverTemperatureSensor(sensorNum, 0);
	}
# if defined(DUET_NG) || defined(PCCB_10)
	else if (ReducedStringEquals(typeName, TmcDriverTemperatureSensor::DuexTypeName))
	{
		ts = new TmcDriverTemperatureSensor(sensorNum, 1);
	}
# endif
#endif
	else
	{
		ts = nullptr;
		reply.printf("Unknown sensor type name \"%s\"", typeName);
	}
	return ts;
}

// Shared function used by two derived classes
//	 pt100rtd list of resistances v temperature
//
//	DIN 43760 / IEC 751 resistance values (ohms) were multiplied by 100 and
//	converted to 16 bit unsigned integers with no loss of accuracy.
//
//	Examples:
//	1852 represents 18.52 ohms and corresponds to a temperature of -200C.
//	10000 ==> 100.00 ohms @   0C
//	13851 ==> 138.51 ohms @ 100C

const float CelsiusMin = -100.0;					// starting temperature of the temp table below
const float CelsiusInterval = 10.0;

static const uint16_t tempTable[] =
{
	6026,  6430,  6833,  7233,  7633,  8031,  8427,  8822,  9216,  9609,
	10000, 10390, 10779, 11167, 11554, 11940, 12324, 12708, 13090, 13471,
	13851, 14229, 14607, 14983, 15358, 15733, 16105, 16477, 16848, 17217,
	17586, 17953, 18319, 18684, 19047, 19410, 19771, 20131, 20490, 20848,
	21205, 21561, 21915, 22268, 22621, 22972, 23321, 23670, 24018, 24364,
	24709, 25053, 25396, 25738, 26078, 26418, 26756, 27093, 27429, 27764,
	28098, 28430, 28762, 29092, 29421, 29749, 30075, 30401, 30725, 31048,
	31371, 31692, 32012, 32330, 32648, 32964, 33279, 33593, 33906, 34218,
	34528, 34838, 35146, 35453, 35759, 36064, 36367, 36670, 36971, 37271,
	37570, 37868, 38165, 38460, 38755, 39048
};

const size_t NumTempTableEntries = sizeof(tempTable)/sizeof(tempTable[0]);

/*static*/ TemperatureError TemperatureSensor::GetPT100Temperature(float& t, uint16_t ohmsx100) noexcept
{

	// Formally-verified binary search routine, adapted from one of the eCv examples
	size_t low = 0u, high = NumTempTableEntries;
	while (high > low)
	keep(low <= high; high <= NumTempTableEntries)
	keep(low == 0u || tempTable[low - 1u] < ohmsx100)
	keep(high == NumTempTableEntries || ohmsx100 <= tempTable[high])
	decrease(high - low)
	{
		size_t mid = (high - low)/2u + low;			// get the mid point, avoiding arithmetic overflow
		if (ohmsx100 <= tempTable[mid])
		{
			high = mid;
		}
		else
		{
			low = mid + 1u;
		}
	}
	_ecv_assert(low <= NumTempTableEntries);
	_ecv_assert(low == 0 || tempTable[low - 1] < ohmsx100);
	_ecv_assert(low == NumTempTableEntries || ohmsx100 <= tempTable[low]);

	if (low == 0)									// if off the bottom of the table
	{
		t = BadErrorTemperature;
		return TemperatureError::shortCircuit;
	}

	if (low >= NumTempTableEntries)					// if off the top of the table
	{
		t = BadErrorTemperature;
		return TemperatureError::openCircuit;
	}

	const float temperatureFraction = (float)(ohmsx100 - tempTable[low - 1])/(float)(tempTable[low] - tempTable[low - 1]);
	t = CelsiusInterval * (low - 1 + temperatureFraction) + CelsiusMin;

	//debugPrintf("raw %f low %u temp %f\n", ohmsx100, low, t);
	return TemperatureError::ok;
}

// End
