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
#include "ZProbeProgrammer.h"
#include "GCodes/GCodeResult.h"

// Endstop manager class
class EndstopsManager
{
public:
	EndstopsManager();

	void Init();

	// Set up the active endstop list according to the axes commanded to move in a G0/G1 S1/S3 command
	void EnableAxisEndstops(AxesBitmap axes, bool forHoming);

	// Set up the active endstops for Z probing
	void EnableZProbe(size_t probeNumber);

	// Set up the active endstops for Z probing with the current probe
	void EnableCurrentZProbe() { EnableZProbe(currentZProbeNumber); }

#ifndef NO_EXTRUDER_ENDSTOPS
	// Enable extruder endstops
	void EnableExtruderEndstop(size_t extruder);
#endif

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

	bool AssignZProbePorts(GCodeBuffer& gb, const StringRef& reply, size_t probeNumber);
	ZProbe& GetCurrentZProbe() const { return *zProbes[currentZProbeNumber]; }
	ZProbe *GetZProbe(size_t num) const;
	void SetZProbeDefaults();
	GCodeResult ProgramZProbe(GCodeBuffer& gb, const StringRef& reply);
	bool WriteZProbeParameters(FileStore *f, bool includingG31) const;

private:
	// Add an endstop to the active list
	void AddToActive(EndstopOrZProbe& e);

	// Translate end stop result to text
	static const char *TranslateEndStopResult(EndStopHit es, bool atHighEnd);

	EndstopOrZProbe * volatile activeEndstops;			// linked list of endstops and Z probes that are active for the current move
	size_t currentZProbeNumber;							// which Z probe we are using

	Endstop *axisEndstops[MaxAxes];						// the endstops assigned to each axis (each one may have several switches), each may be null
	ZProbe *zProbes[MaxZProbes];						// the Z probes used. The first one is always non-null.

	ZProbeProgrammer zProbeProg;

	bool isHomingMove;									// true if calls to CheckEndstops are for the purpose of homing
};

#endif /* SRC_ENDSTOPS_ENDSTOPMANAGER_H_ */
