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

	HeaterProtection(size_t index) noexcept;
	void Init(float tempLimit) noexcept;

	HeaterProtection *Next() const noexcept { return next; }
	void SetNext(HeaterProtection *n) noexcept { next = n; }

	bool Check() noexcept;													// Check if any action needs to be taken

	int GetHeater() const noexcept { return heater; }
	void SetHeater(int newHeater) noexcept;									// Set the heater to control

	int GetSensorNumber() const noexcept { return sensorNumber; }			// Get the supervisory sensor number
	void SetSensorNumber(int sn) noexcept;									// Set the supervisory sensor number

	float GetTemperatureLimit() const noexcept { return limit; }			// Get the temperature limit
	void SetTemperatureLimit(float newLimit) noexcept;						// Set the temperature limit

	HeaterProtectionAction GetAction() const noexcept { return action; }	// Get the action to trigger when a temperature event occurs
	void SetAction(HeaterProtectionAction newAction) noexcept;				// Set the action to trigger when a temperature event occurs

	HeaterProtectionTrigger GetTrigger() const noexcept { return trigger; }	// Get the condition for a temperature event
	void SetTrigger(HeaterProtectionTrigger newTrigger) noexcept;			// Set the condition for a temperature event

private:
	HeaterProtection *next;											// link to next HeaterProtection item for the same heater

	float limit;													// temperature limit
	int heater;														// number of the heater we are protecting
	int sensorNumber;												// the sensor that we use to monitor the heater
	HeaterProtectionAction action;									// what action we take of we detect a fault
	HeaterProtectionTrigger trigger;								// what is treated a fault

	size_t badTemperatureCount;										// how many consecutive sensor reading faults we have had
};

inline void HeaterProtection::SetSensorNumber(int sn) noexcept
{
	sensorNumber = sn;
}

inline void HeaterProtection::SetTemperatureLimit(float newLimit) noexcept
{
	limit = newLimit;
}

inline void HeaterProtection::SetAction(HeaterProtectionAction newAction) noexcept
{
	action = newAction;
}

inline void HeaterProtection::SetTrigger(HeaterProtectionTrigger newTrigger) noexcept
{
	trigger = newTrigger;
}

#endif
