/*
 * RestorePoint.h
 *
 *  Created on: 14 Jun 2017
 *      Author: David
 */

#ifndef SRC_GCODES_RESTOREPOINT_H_
#define SRC_GCODES_RESTOREPOINT_H_

#include "RepRapFirmware.h"

#if SUPPORT_IOBITS
#include "PortControl.h"
#endif

struct RestorePoint
{
	float moveCoords[MaxAxes];				// The axis locations when we paused
	float feedRate;							// The feed rate for the current move
	float virtualExtruderPosition;			// The virtual extruder position at the start of this move
	float proportionDone;					// How much of this move we have already done (zero unless we interrupted a move)
	FilePosition filePos;					// The file position that this move was read from

#if SUPPORT_IOBITS
	IoBits_t ioBits;						// The output port bits setting for this move
#endif
	int toolNumber;							// The tool number that was active

	RestorePoint();
	void Init();
};

#endif /* SRC_GCODES_RESTOREPOINT_H_ */
