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
	AdditionalOutputSensor(unsigned int sensorNum, const char *type, bool enforcePollOrder);
	virtual ~AdditionalOutputSensor();
	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply) override;
	void Poll() override;

protected:
	uint8_t parentSensor;
	uint8_t outputNumber;

private:
	bool enforcePollOrder;
};

#endif /* ADDITIONALOUTPUTSENSOR_H_ */
