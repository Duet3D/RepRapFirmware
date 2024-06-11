/*
 * FilamentMonitor.h
 *
 *  Created on: 20 Jul 2017
 *      Author: David
 */

#ifndef SRC_FILAMENTSENSORS_FILAMENTMONITOR_H_
#define SRC_FILAMENTSENSORS_FILAMENTMONITOR_H_

#include <RepRapFirmware.h>
#include <Hardware/IoPorts.h>
#include <ObjectModel/ObjectModel.h>
#include <RTOSIface/RTOSIface.h>
#include <RRF3Common.h>

#if SUPPORT_CAN_EXPANSION
struct CanMessageFilamentMonitorsStatusNew;
struct FilamentMonitorDataNew;
#endif

#if SUPPORT_REMOTE_COMMANDS
struct CanMessageCreateFilamentMonitor;
struct CanMessageDeleteFilamentMonitor;
struct CanMessageGeneric;
class CanMessageGenericParser;
#endif

class FilamentMonitor INHERIT_OBJECT_MODEL
{
public:
	// Override the virtual destructor if your derived class allocates any dynamic memory
	virtual ~FilamentMonitor() noexcept;

	// We don't want to copy filament monitors
	FilamentMonitor(const FilamentMonitor&) = delete;

	// Static initialisation
	static void InitStatic() noexcept;

	// Poll the filament sensors
	static void Spin() noexcept;

	// Check the drive assignments. Called when M584 may have been used to remap extruder drives. Return true if we need to output the warning appended to 'reply'.
	static bool CheckDriveAssignments(const StringRef& reply) noexcept;

	// Close down the filament monitors, in particular stop them generating interrupts. Called when we are about to update firmware.
	static void Exit() noexcept;

	// Handle M591
	static GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply, unsigned int extruder) THROWS(GCodeException)
	pre(extruder < MaxExtruders; extruder < reprap.GetGCodes().GetNumExtruders());

	// Send diagnostics info
	static void Diagnostics(MessageType mtype) noexcept;

#if SUPPORT_OBJECT_MODEL
	// Get the number of monitors to report in the OM
	static size_t GetNumMonitorsToReport() noexcept;

	// Get access to a filament monitor when we already have a read lock
	static FilamentMonitor *GetMonitorAlreadyLocked(size_t extruder) noexcept { return filamentSensors[extruder]; }
#endif

#if SUPPORT_CAN_EXPANSION
	static void UpdateRemoteFilamentStatus(CanAddress src, CanMessageFilamentMonitorsStatusNew& msg) noexcept;
#endif

#if SUPPORT_REMOTE_COMMANDS
	// Create a new filament monitor, or replace an existing one
	static GCodeResult Create(const CanMessageCreateFilamentMonitor& msg, const StringRef& reply) noexcept;

	// Delete a filament monitor
	static GCodeResult Delete(const CanMessageDeleteFilamentMonitor& msg, const StringRef& reply) noexcept;

	// Configure a filament monitor
	static GCodeResult Configure(const CanMessageGeneric& msg, const StringRef& reply) noexcept;

	// Delete all filament monitors
	static void DeleteAll() noexcept;

	// Generate diagnostics info
	static void GetDiagnostics(const StringRef& reply) noexcept;
#endif

	// This must be public so that the array descriptor in class RepRap can lock it
	static ReadWriteLock filamentMonitorsLock;

protected:
	DECLARE_OBJECT_MODEL

	FilamentMonitor(unsigned int drv, unsigned int monitorType, DriverId did) noexcept;

	// Configure this sensor, returning true if error and setting 'seen' if we processed any configuration parameters
	virtual GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply, bool& seen) THROWS(GCodeException) = 0;

	// Print diagnostic info for this sensor
	virtual void Diagnostics(MessageType mtype, unsigned int extruder) noexcept = 0;

	// ISR for when the pin state changes. It should return true if the ISR wants the commanded extrusion to be fetched.
	virtual bool Interrupt() noexcept = 0;

	// Call this to disable the interrupt before deleting a filament monitor
	virtual void Disable() noexcept;

	// Get the type of filament monitor to report in the OM
	virtual const char *_ecv_array GetTypeText() const noexcept = 0;

	// Return the type of this sensor
	unsigned int GetType() const noexcept { return type; }

	// Return the enable state of this sensor
	uint8_t GetEnableMode() const noexcept { return enableMode; }

	// Check that this monitor still refers to a valid extruder
	bool IsValid(size_t extruderNumber) const noexcept;

	// Get the status of the filament monitor as a string
	const char *GetStatusText() const noexcept { return lastStatus.ToString(); }

	// Call the following at intervals to check the status. This is only called when extrusion is in progress or imminent.
	// 'filamentConsumed' is the net amount of extrusion since the last call to this function.
	virtual FilamentSensorStatus Check(bool isPrinting, bool fromIsr, uint32_t isrMillis, float filamentConsumed) noexcept = 0;

	// Clear the measurement state - called when we are not printing a file. Return the present/not present status if available.
	virtual FilamentSensorStatus Clear() noexcept = 0;

#if SUPPORT_REMOTE_COMMANDS
	// Configure this sensor, returning an error code and setting 'seen' if we processed any configuration parameters
	virtual GCodeResult Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept = 0;

	// Store collected data in a CAN message slot returning true if there was data worth sending
	virtual void GetLiveData(FilamentMonitorDataNew& data) const noexcept = 0;

	// Print diagnostic info for this sensor
	virtual void Diagnostics(const StringRef& reply) noexcept = 0;
#endif

#if SUPPORT_CAN_EXPANSION
	// Update live filament monitor data received from a remote filament monitor
	virtual void UpdateLiveData(const FilamentMonitorDataNew& data) noexcept = 0;
#endif

	GCodeResult CommonConfigure(GCodeBuffer& gb, const StringRef& reply, InterruptMode interruptMode, bool& seen) THROWS(GCodeException);
#if SUPPORT_REMOTE_COMMANDS
	GCodeResult CommonConfigure(const CanMessageGenericParser& parser, const StringRef& reply, InterruptMode interruptMode, bool& seen) noexcept;
	uint8_t GetDriver() const noexcept { return driveNumber; }
#endif

	const IoPort& GetPort() const noexcept { return port; }
	bool HaveIsrStepsCommanded() const noexcept { return haveIsrStepsCommanded; }

	static int32_t ConvertToPercent(float f)
	{
		return lrintf(100 * f);
	}

#if SUPPORT_CAN_EXPANSION
	bool IsLocal() const noexcept { return !hasRemote; }
#endif

private:
	// Create a filament sensor returning null if not a valid sensor type
	static FilamentMonitor *Create(unsigned int extruder, unsigned int monitorType, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	static void InterruptEntry(CallbackParameter param) noexcept;

	static constexpr size_t NumFilamentMonitors =
#if SUPPORT_REMOTE_COMMANDS
	// When running as an expansion board, filament monitors are indexed by driver number; otherwise they are indexed by extruder number.
								max<size_t>(MaxExtruders, NumDirectDrivers);
#else
								MaxExtruders;
#endif

	static FilamentMonitor *filamentSensors[NumFilamentMonitors];

#if SUPPORT_REMOTE_COMMANDS
	static constexpr uint32_t StatusUpdateInterval = 2000;				// how often we send status reports when there isn't a change
	static constexpr uint32_t LiveStatusUpdateInterval = 250;			// how often we report live status
	static uint32_t whenStatusLastSent;
	static size_t firstDriveToSend;
#endif

	int32_t isrExtruderStepsCommanded;
	uint32_t lastIsrMillis;
	unsigned int driveNumber;											// the logical drive number that represents this extruder (but the actual drive number if in expansion mode)
	unsigned int type;
	IoPort port;
	DriverId driverId;													// the board address and physical driver that the logical drive number maps to

	uint8_t enableMode;													// 0 = disabled, 1 = enabled when SD card printing, 2 = always enabled
	bool isrWasPrinting;
	bool haveIsrStepsCommanded;
	FilamentSensorStatus lastStatus;
#if SUPPORT_CAN_EXPANSION
	FilamentSensorStatus lastRemoteStatus;
	bool hasRemote;
#endif
};

#endif /* SRC_FILAMENTSENSORS_FILAMENTMONITOR_H_ */
