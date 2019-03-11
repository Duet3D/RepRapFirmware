/*
 * TemperatureError.cpp
 *
 *  Created on: 21 Apr 2016
 *      Author: David
 */

#include "TemperatureError.h"

const char* TemperatureErrorString(TemperatureError err)
{
	switch(err)
	{
	case TemperatureError::success:			return "success";
	case TemperatureError::shortCircuit:	return "short-circuit in sensor";
	case TemperatureError::shortToVcc:		return "sensor short to Vcc";
	case TemperatureError::shortToGround:	return "sensor short to ground";
	case TemperatureError::openCircuit:		return "sensor open circuit";
	case TemperatureError::timeout:			return "sensor timeout";
	case TemperatureError::ioError:			return "sensor I/O error";
	case TemperatureError::hardwareError:	return "sensor hardware error";
	case TemperatureError::notReady:		return "sensor not ready";
	case TemperatureError::busBusy:			return "sensor bus busy";
	case TemperatureError::badResponse:		return "bad response from sensor";
	case TemperatureError::unknownChannel:	return "unknown temperature sensor channel";
	case TemperatureError::notInitialised:	return "sensor not initialised";
	case TemperatureError::unknownHeater:	return "unknown heater";
	case TemperatureError::overOrUnderVoltage:	return "sensor short to other wiring";
	case TemperatureError::badVref:			return "bad Vref";
	case TemperatureError::badVssa:			return "bad Vssa";
	default:								return "unknown temperature sense error";
	}
}

// End
