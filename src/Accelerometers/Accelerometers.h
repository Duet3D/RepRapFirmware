/*
 * Accelerometers.h
 *
 *  Created on: 19 Mar 2021
 *      Author: David
 */

#ifndef SRC_ACCELEROMETERS_ACCELEROMETERS_H_
#define SRC_ACCELEROMETERS_ACCELEROMETERS_H_

#include <RepRapFirmware.h>

#if SUPPORT_ACCELEROMETERS

#include <GCodes/GCodeException.h>
#include <ObjectModel/ObjectModel.h>

#if SUPPORT_CAN_EXPANSION
# include <CanId.h>
#endif

class CanMessageAccelerometerData;

// One entry of sensors.accelerometers[], indexed by the M955/M956 P number
class Accelerometer INHERIT_OBJECT_MODEL
{
public:
	Accelerometer() noexcept;

	bool IsConfigured() const noexcept;
	void Clear() noexcept;

#if SUPPORT_CAN_EXPANSION
	bool IsRemote() const noexcept;

	CanAddress boardAddress;								// CanId::NoAddress when this number is not configured
#endif
	AutoStringHandle port;									// pin names as given to M955 C, including the board prefix
	uint32_t lastRunDataPoints;								// samples collected by the last run, 0 if it failed
	uint32_t runs;											// number of completed runs
	uint16_t samplingRate;									// rate the device settled on, 0 if unknown
	uint8_t resolution;										// bits per sample the device settled on, 0 if unknown
	uint8_t orientation;

protected:
	DECLARE_OBJECT_MODEL
};

namespace Accelerometers
{
	size_t GetNumAccelerometersToReport() noexcept;
	const Accelerometer *_ecv_null GetAccelerometer(size_t index) noexcept;
	GCodeResult ConfigureAccelerometer(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	GCodeResult StartAccelerometer(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	void Exit() noexcept;
#if SUPPORT_CAN_EXPANSION
	void ProcessReceivedData(CanAddress src, const CanMessageAccelerometerData& msg, size_t msgLen) noexcept;
	void RemoteBoardRestarted(CanAddress src) noexcept;
#endif
#if 0	// We don't currently support accelerometers on main boards used as expansion boards
//#if SUPPORT_REMOTE_COMMANDS
	void Diagnostics(const StringRef& reply) noexcept;
#endif
}

#endif

#endif /* SRC_ACCELEROMETERS_ACCELEROMETERS_H_ */
