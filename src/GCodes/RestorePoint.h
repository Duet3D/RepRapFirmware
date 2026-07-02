/*
 * RestorePoint.h
 *
 *  Created on: 14 Jun 2017
 *      Author: David
 */

#ifndef SRC_GCODES_RESTOREPOINT_H_
#define SRC_GCODES_RESTOREPOINT_H_

#include <RepRapFirmware.h>
#include <ObjectModel/ObjectModel.h>

class RestorePoint INHERIT_OBJECT_MODEL
{
public:
	float moveCoords[MaxAxes];				// The user coordinates when we paused
	float originalFeedRate;					// The feed rate for the current move in original units
	float virtualExtruderPosition;			// The virtual extruder position at the start of this move
	float proportionDone;					// How much of this move we have already done (zero unless we interrupted a move)
	FilePosition filePos;					// The file position that this move was read from
	int8_t gCommandNumber;					// Which of G0/G1/G2/G3 generated the move at filePos (0-3), or -1 if unknown; used to restore the modal command context on resume
	float initialUserC0, initialUserC1;		// If we paused during an arc move and proportionDone is nonzero, the X and Y user coordinates at the start of the move
	int toolNumber;							// The tool number that was active
	float fanSpeed;							// the last fan speed that was set by M106 with no P parameter
#if SUPPORT_ASYNC_MOVES
	AxesBitmap axesAndExtrudersOwned;		// axes and extruders that this movement system owned when we saved the restore point. Only used when resuming after skipped objects.
#endif

#if SUPPORT_LASER || SUPPORT_IOBITS
	LaserPwmOrIoBits laserPwmOrIoBits;		// The output port bits setting for this move, or the laser power
#endif

#if SUPPORT_LASER
	LaserPixelData laserPixelData;
#endif

	RestorePoint() noexcept;
	void Init() noexcept;

protected:
	DECLARE_OBJECT_MODEL_WITH_ARRAYS
};

#endif /* SRC_GCODES_RESTOREPOINT_H_ */
