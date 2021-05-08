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
	float moveCoords[MaxAxes];				// The axis locations when we paused
	float feedRate;							// The feed rate for the current move
	float virtualExtruderPosition;			// The virtual extruder position at the start of this move
	float proportionDone;					// How much of this move we have already done (zero unless we interrupted a move)
	FilePosition filePos;					// The file position that this move was read from
	float initialUserC0, initialUserC1;		// If we paused during an arc move and proportionDone is nonzero, the X and Y user coordinates at the start of the move
	int toolNumber;							// The tool number that was active
	float fanSpeed;							// the last fan speed that was set by M106 with no P parameter

#if SUPPORT_LASER || SUPPORT_IOBITS
	LaserPwmOrIoBits laserPwmOrIoBits;		// The output port bits setting for this move, or the laser power
#endif

	RestorePoint() noexcept;
	void Init() noexcept;

protected:
	DECLARE_OBJECT_MODEL
	OBJECT_MODEL_ARRAY(coordinates)
};

#endif /* SRC_GCODES_RESTOREPOINT_H_ */
