/*
 * HeaterProtection.h
 *
 *  Created on: 16 Nov 2017
 *      Author: Christian
 */

#ifndef HEATERPROTECTION_H
#define HEATERPROTECTION_H

#include "RepRapFirmware.h"


// Condition of a heater protection event
enum class HeaterProtectionTrigger : uint8_t
{
	TemperatureExceeded,
	TemperatureTooLow
};

const HeaterProtectionTrigger MaxHeaterProtectionTrigger = HeaterProtectionTrigger::TemperatureTooLow;

// The action to trigger when the target condition is met
enum class HeaterProtectionAction : uint8_t
{
	GenerateFault = 0,
	PermanentSwitchOff,
	TemporarySwitchOff
};

const HeaterProtectionAction MaxHeaterProtectionAction = HeaterProtectionAction::TemporarySwitchOff;


class Heat;

class HeaterProtection
{
public:
	friend class Heat;

	HeaterProtection(size_t index);
	void Init(float tempLimit);

	HeaterProtection *Next() const { return next; }
	void SetNext(HeaterProtection *n) { next = n; }

	bool Check();													// Check if any action needs to be taken

	int GetHeater() const { return heater; }
	void SetHeater(int newHeater);									// Set the heater to control

	int GetSensorNumber() const { return sensorNumber; }			// Get the supervisory sensor number
	void SetSensorNumber(int sn);									// Set the supervisory sensor number

	float GetTemperatureLimit() const { return limit; }				// Get the temperature limit
	void SetTemperatureLimit(float newLimit);						// Set the temperature limit

	HeaterProtectionAction GetAction() const { return action; }		// Get the action to trigger when a temperature event occurs
	void SetAction(HeaterProtectionAction newAction);				// Set the action to trigger when a temperature event occurs

	HeaterProtectionTrigger GetTrigger() const { return trigger; }	// Get the condition for a temperature event
	void SetTrigger(HeaterProtectionTrigger newTrigger);			// Set the condition for a temperature event

private:
	HeaterProtection *next;											// link to next HeaterProtection item for the same heater

	float limit;													// temperature limit
	int heater;														// number of the heater we are protecting
	int sensorNumber;												// the sensor that we use to monitor the heater
	HeaterProtectionAction action;									// what action we take of we detect a fault
	HeaterProtectionTrigger trigger;								// what is treated a fault

	size_t badTemperatureCount;										// how many consecutive sensor reading faults we have had
};

inline void HeaterProtection::SetSensorNumber(int sn)
{
	sensorNumber = sn;
}

inline void HeaterProtection::SetTemperatureLimit(float newLimit)
{
	limit = newLimit;
}

inline void HeaterProtection::SetAction(HeaterProtectionAction newAction)
{
	action = newAction;
}

inline void HeaterProtection::SetTrigger(HeaterProtectionTrigger newTrigger)
{
	trigger = newTrigger;
}

#endif
