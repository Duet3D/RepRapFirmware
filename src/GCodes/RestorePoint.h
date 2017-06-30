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
	float moveCoords[DRIVES];
	float feedRate;
	float virtualExtruderPosition;
	FilePosition filePos;
#if SUPPORT_IOBITS
	IoBits_t ioBits;
#endif
	RestorePoint();
	void Init();
};

#endif /* SRC_GCODES_RESTOREPOINT_H_ */
