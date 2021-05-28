/*
 * SensorWithPort.h
 *
 *  Created on: 18 Jul 2019
 *      Author: David
 */

#ifndef SRC_HEATING_SENSORS_SENSORWITHPORT_H_
#define SRC_HEATING_SENSORS_SENSORWITHPORT_H_

#include "TemperatureSensor.h"

class CanMessageGenericParser;

class SensorWithPort : public TemperatureSensor
{
protected:
	SensorWithPort(unsigned int sensorNum, const char *type) noexcept;
	~SensorWithPort() noexcept;

	// Try to configure the port
	bool ConfigurePort(GCodeBuffer& gb, const StringRef& reply, PinAccess access, bool& seen) THROWS(GCodeException);

#if SUPPORT_REMOTE_COMMANDS
	// Try to configure the port
	bool ConfigurePort(const CanMessageGenericParser& parser, const StringRef& reply, PinAccess access, bool& seen);
#endif

	// Copy the basic details to the reply buffer. This hides the version in the base class.
	void CopyBasicDetails(const StringRef& reply) const noexcept;

	IoPort port;
};

#endif /* SRC_HEATING_SENSORS_SENSORWITHPORT_H_ */
