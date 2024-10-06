/*
 * AdditionalOutputSensor.h
 *
 *  Created on: 17 Oct 2019
 *      Author: manuel
 */

#ifndef ADDITIONALOUTPUTSENSOR_H_
#define ADDITIONALOUTPUTSENSOR_H_

# include "TemperatureSensor.h"

class AdditionalOutputSensor : public TemperatureSensor
{
public:
	AdditionalOutputSensor(unsigned int sensorNum, const char *_ecv_array type, bool pEnforcePollOrder) noexcept;
	virtual ~AdditionalOutputSensor() noexcept override;

	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply, bool& changed) THROWS(GCodeException) override;
#if SUPPORT_REMOTE_COMMANDS
	GCodeResult Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept override;
#endif

	void Poll() noexcept override;

protected:
	// Append the pin details to the reply buffer
	void AppendPinDetails(const StringRef& reply) const noexcept override;

	uint8_t parentSensor;
	uint8_t outputNumber;

private:
	GCodeResult ConfigurePort(const char *_ecv_array portName, const StringRef& reply) noexcept;

	bool enforcePollOrder;
};

#endif /* ADDITIONALOUTPUTSENSOR_H_ */
