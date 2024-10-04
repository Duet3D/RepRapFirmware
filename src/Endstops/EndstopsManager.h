/*
 * Endstop.h
 *
 *  Created on: 3 Apr 2019
 *      Author: David
 */

#ifndef SRC_ENDSTOPS_ENDSTOPMANAGER_H_
#define SRC_ENDSTOPS_ENDSTOPMANAGER_H_

#include <RepRapFirmware.h>
#include "EndstopDefs.h"
#include <ObjectModel/ObjectModel.h>
#include <RTOSIface/RTOSIface.h>

#if SUPPORT_CAN_EXPANSION
# include "CanId.h"
class CanMessageBuffer;
#endif

class StallDetectionEndstop;

// Endstop manager class
class EndstopsManager INHERIT_OBJECT_MODEL
{
public:
	EndstopsManager() noexcept;
	EndstopsManager(const EndstopsManager&) = delete;

	void Init() noexcept;

	// Set no active endstops
	void ClearEndstops() noexcept;

	// Set up the active endstop list according to the axes commanded to move in a G0/G1 S1/S3 command returning true if successful
	bool EnableAxisEndstops(AxesBitmap axes, bool forHoming, bool& reduceAcceleration) noexcept __attribute__ ((warn_unused_result));

	// Clear all endstops then set up the active endstops for Z probing returning true if successful
	bool EnableZProbe(size_t probeNumber, bool probingAway = false) noexcept __attribute__ ((warn_unused_result));

	// Enable extruder endstops, adding to any axis endstops already set up
	bool EnableExtruderEndstops(ExtrudersBitmap extruders) noexcept;

	// Get the first endstop that has triggered and remove it from the active list if appropriate
	EndstopHitDetails CheckEndstops() noexcept;

	// Configure the endstops in response to M574
	GCodeResult HandleM574(GCodeBuffer& gb, const StringRef& reply, OutputBuffer*& outbuf) THROWS(GCodeException);

	EndStopPosition GetEndStopPosition(size_t axis) const noexcept pre(axis < MaxAxes);
	bool HomingZWithProbe() const noexcept;

	bool Stopped(size_t axis) const noexcept;

	// Return true if we have any endstops active that have not been triggered yet
	bool AnyEndstopsActive() const noexcept { return activeEndstops != nullptr; }

	void GetM119report(const StringRef& reply) noexcept;

	// Validate any enabled stall endstops, returning true if all OK, else store the error details and return false
	bool ValidateEndstops(const DDA& dda) noexcept;

	// Get and clear the validation result
	EndstopValidationResult GetEndstopValidationResult(uint8_t& driver) noexcept;

	// Z probe
	GCodeResult HandleM558(GCodeBuffer& gb, const StringRef &reply) THROWS(GCodeException);		// M558
	GCodeResult HandleG31(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);		// G31

	ReadLockedPointer<ZProbe> GetZProbe(size_t index) const noexcept;
	ReadLockedPointer<ZProbe> GetZProbeOrDefault(size_t index) const noexcept;
	ZProbe *_ecv_null GetZProbeFromISR(size_t index) const noexcept;
	ZProbe& GetDefaultZProbeFromISR() const noexcept;

	void SetZProbeDefaults() noexcept;
	GCodeResult ProgramZProbe(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);

#if SUPPORT_CAN_EXPANSION
	void HandleRemoteEndstopChange(CanAddress src, uint8_t handleMajor, uint8_t handleMinor, bool state) noexcept;
	void HandleRemoteZProbeChange(CanAddress src, uint8_t handleMajor, uint8_t handleMinor, bool state, uint32_t reading) noexcept;
	void HandleRemoteAnalogZProbeValueChange(CanAddress src, uint8_t handleMajor, uint8_t handleMinor, uint32_t reading) noexcept;
#endif

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	bool WriteZProbeParameters(FileStore *f, bool includingG31) const noexcept;
#endif

protected:
	DECLARE_OBJECT_MODEL_WITH_ARRAYS

private:
	// Add an endstop to the active list
	void AddToActive(EndstopOrZProbe& e) noexcept;

#if SUPPORT_OBJECT_MODEL
	size_t GetNumProbesToReport() const noexcept;
#endif

	// Translate end stop result to text
	static const char *TranslateEndStopResult(bool hit, bool atHighEnd) noexcept;

	// Return a pointer to an endstop. Caller must already own a read lock on endstopsLock.
	const Endstop *FindEndstopWhenLockOwned(size_t axis) const noexcept;

	static ReadWriteLock endstopsLock;
	static ReadWriteLock zProbesLock;

	EndstopOrZProbe *_ecv_null volatile activeEndstops;	// linked list of endstops and Z probes that are active for the current move

	Endstop *_ecv_null axisEndstops[MaxAxes];			// the endstops assigned to each axis (each one may have several switches), each may be null
#if HAS_STALL_DETECT
	StallDetectionEndstop *extrudersEndstop;			// the endstop used for extruder stall detection, one will do for all extruders
#endif
	ZProbe *_ecv_null zProbes[MaxZProbes];				// the Z probes used. The first one is always non-null.
	ZProbe *defaultZProbe;

	EndstopValidationResult validationResult;			// the error coder we got from validating endstops
	uint8_t failingDriverNumber;						// the number of the local driver that failed validation
	bool isHomingMove;									// true if calls to CheckEndstops are for the purpose of homing
};

// Get and clear the validation result
inline EndstopValidationResult EndstopsManager::GetEndstopValidationResult(uint8_t& driver) noexcept
{
	driver = failingDriverNumber;
	const EndstopValidationResult ret = validationResult;
	validationResult = EndstopValidationResult::ok;
	return ret;
}

#endif /* SRC_ENDSTOPS_ENDSTOPMANAGER_H_ */
