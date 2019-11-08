/*
 * Endstop.h
 *
 *  Created on: 3 Apr 2019
 *      Author: David
 */

#ifndef SRC_ENDSTOPS_ENDSTOPMANAGER_H_
#define SRC_ENDSTOPS_ENDSTOPMANAGER_H_

#include "RepRapFirmware.h"
#include "EndstopDefs.h"
#include "GCodes/GCodeResult.h"
#include <RTOSIface/RTOSIface.h>

#if SUPPORT_CAN_EXPANSION
# include "CanId.h"
class CanMessageBuffer;
#endif

// Endstop manager class
class EndstopsManager
{
public:
	EndstopsManager();

	void Init();

	// Set up the active endstop list according to the axes commanded to move in a G0/G1 S1/S3 command returning true if successful
	bool EnableAxisEndstops(AxesBitmap axes, bool forHoming) __attribute__ ((warn_unused_result));

	// Set up the active endstops for Z probing returning true if successful
	bool EnableZProbe(size_t probeNumber, bool probingAway = false) __attribute__ ((warn_unused_result));

	// Set up the active endstops for Z probing with the current probe
	bool EnableCurrentZProbe(bool probingAway = false) __attribute__ ((warn_unused_result)) { return EnableZProbe(currentZProbeNumber, probingAway); }

	// Enable extruder endstops
	bool EnableExtruderEndstop(size_t extruder);

	// Get the first endstop that has triggered and remove it from the active list if appropriate
	EndstopHitDetails CheckEndstops(bool goingSlow);

	// Configure the endstops in response to M574
	GCodeResult HandleM574(GCodeBuffer& gb, const StringRef& reply);

	EndStopPosition GetEndStopPosition(size_t axis) const pre(axis < MaxAxes);
	bool HomingZWithProbe() const;

	EndStopHit Stopped(size_t axis) const;

	void GetM119report(const StringRef& reply);

	// Z probe
	GCodeResult HandleM558(GCodeBuffer& gb, const StringRef &reply);		// M558
	GCodeResult HandleG31(GCodeBuffer& gb, const StringRef& reply);			// G31

	ZProbe& GetCurrentZProbe() const;
	const size_t GetCurrentZProbeNumber() const { return currentZProbeNumber; }
	ZProbe *GetZProbe(size_t num) const;
	void SetZProbeDefaults();
	GCodeResult ProgramZProbe(GCodeBuffer& gb, const StringRef& reply);

#if SUPPORT_CAN_EXPANSION
	void HandleRemoteInputChange(CanAddress src, uint8_t handleMajor, uint8_t handleMinor, bool state);

	void OnEndstopStatesChanged();
#endif

#if HAS_MASS_STORAGE
	bool WriteZProbeParameters(FileStore *f, bool includingG31) const;
#endif

private:
	// Add an endstop to the active list
	void AddToActive(EndstopOrZProbe& e);

	// Translate end stop result to text
	static const char *TranslateEndStopResult(EndStopHit es, bool atHighEnd);

	ReadLockedPointer<ZProbe> FindZProbe(size_t index) const;
	ReadLockedPointer<Endstop> FindEndstop(size_t axis) const;

	static ReadWriteLock endstopsLock;					// used to lock both endstops and Z probes

	EndstopOrZProbe * volatile activeEndstops;			// linked list of endstops and Z probes that are active for the current move
	size_t currentZProbeNumber;							// which Z probe we are using

	Endstop *axisEndstops[MaxAxes];						// the endstops assigned to each axis (each one may have several switches), each may be null
	ZProbe *zProbes[MaxZProbes];						// the Z probes used. The first one is always non-null.
	ZProbe *defaultZProbe;

	bool isHomingMove;									// true if calls to CheckEndstops are for the purpose of homing
};

#endif /* SRC_ENDSTOPS_ENDSTOPMANAGER_H_ */
