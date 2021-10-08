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

	// Set up the active endstop list according to the axes commanded to move in a G0/G1 S1/S3 command returning true if successful
	bool EnableAxisEndstops(AxesBitmap axes, bool forHoming) noexcept __attribute__ ((warn_unused_result));

	// Set up the active endstops for Z probing returning true if successful
	bool EnableZProbe(size_t probeNumber, bool probingAway = false) noexcept __attribute__ ((warn_unused_result));

	// Enable extruder endstops
	bool EnableExtruderEndstops(ExtrudersBitmap extruders) noexcept;

	// Get the first endstop that has triggered and remove it from the active list if appropriate
	EndstopHitDetails CheckEndstops() noexcept;

	// Configure the endstops in response to M574
	GCodeResult HandleM574(GCodeBuffer& gb, const StringRef& reply, OutputBuffer*& outbuf) THROWS(GCodeException);

	EndStopPosition GetEndStopPosition(size_t axis) const noexcept pre(axis < MaxAxes);
	bool HomingZWithProbe() const noexcept;

	bool Stopped(size_t axis) const noexcept;

	void GetM119report(const StringRef& reply) noexcept;

	// Z probe
	GCodeResult HandleM558(GCodeBuffer& gb, const StringRef &reply) THROWS(GCodeException);		// M558
	GCodeResult HandleG31(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);		// G31

	ReadLockedPointer<ZProbe> GetZProbe(size_t index) const noexcept;
	ReadLockedPointer<ZProbe> GetZProbeOrDefault(size_t index) const noexcept;
	ZProbe& GetDefaultZProbeFromISR() const noexcept;

	void SetZProbeDefaults() noexcept;
	GCodeResult ProgramZProbe(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);

#if SUPPORT_CAN_EXPANSION
	void HandleRemoteEndstopChange(CanAddress src, uint8_t handleMajor, uint8_t handleMinor, bool state) noexcept;
	void HandleRemoteZProbeChange(CanAddress src, uint8_t handleMajor, uint8_t handleMinor, bool state) noexcept;
	void OnEndstopOrZProbeStatesChanged() noexcept;
#endif

#if HAS_MASS_STORAGE || HAS_LINUX_INTERFACE
	bool WriteZProbeParameters(FileStore *f, bool includingG31) const noexcept;
#endif

protected:
	DECLARE_OBJECT_MODEL
	OBJECT_MODEL_ARRAY(sensors)
	OBJECT_MODEL_ARRAY(endstops)
	OBJECT_MODEL_ARRAY(filamentMonitors)
	OBJECT_MODEL_ARRAY(gpin)
	OBJECT_MODEL_ARRAY(probes)

private:
	// Add an endstop to the active list
	void AddToActive(EndstopOrZProbe& e) noexcept;

#if SUPPORT_OBJECT_MODEL
	size_t GetNumProbesToReport() const noexcept;
#endif

	// Translate end stop result to text
	static const char *TranslateEndStopResult(bool hit, bool atHighEnd) noexcept;

	ReadLockedPointer<Endstop> FindEndstop(size_t axis) const noexcept;

	static ReadWriteLock endstopsLock;
	static ReadWriteLock zProbesLock;

	EndstopOrZProbe * volatile activeEndstops;			// linked list of endstops and Z probes that are active for the current move

	Endstop *axisEndstops[MaxAxes];						// the endstops assigned to each axis (each one may have several switches), each may be null
#if HAS_STALL_DETECT
	StallDetectionEndstop *extrudersEndstop;			// the endstop used for extruder stall detection, one will do for all extruders
#endif
	ZProbe *zProbes[MaxZProbes];						// the Z probes used. The first one is always non-null.
	ZProbe *defaultZProbe;

	bool isHomingMove;									// true if calls to CheckEndstops are for the purpose of homing
};

#endif /* SRC_ENDSTOPS_ENDSTOPMANAGER_H_ */
