/*
 * StraightProbe.h
 *
 *  Created on: 4 Oct 2019
 *      Author: manuel
 */

#ifndef SRC_MOVEMENT_STRAIGHTPROBESETTINGS_H_
#define SRC_MOVEMENT_STRAIGHTPROBESETTINGS_H_

#include "RepRapFirmware.h"
#include "ObjectModel/ObjectModel.h"
#include "Endstops/ZProbe.h"

enum class StraightProbeType : uint8_t {
	unset,
	towardsWorkpieceErrorOnFailure,  // probe toward workpiece, stop on contact, signal error if failure
	towardsWorkpiece,				 // probe toward workpiece, stop on contact
	awayFromWorkpieceErrorOnFailure, // probe away from workpiece, stop on loss of contact, signal error if failure
	awayFromWorkpiece				 // probe away from workpiece, stop on loss of contact
};

class StraightProbeSettings INHERIT_OBJECT_MODEL
{
public:
	StraightProbeSettings();

	void Reset();

	void SetCoordsToTarget(float[MaxAxes]) const;
	void SetTarget(const float[MaxAxes]);                            // Set target for G38 move

	const StraightProbeType GetType() const { return type; }
	void SetStraightProbeType(const StraightProbeType t) { type = t; }

	const AxesBitmap GetMovingAxes() const { return movingAxes; }
	void AddMovingAxis(const size_t);

	const size_t GetZProbeToUse() const { return probeToUse; }
	void SetZProbeToUse(size_t probeNumber) { probeToUse = probeNumber; }

protected:
	DECLARE_OBJECT_MODEL

private:
	AxesBitmap movingAxes;                 // Axes supposed to move - this is only used for manual probing
	size_t probeToUse;                     // Use this ZProbe
	float target[MaxAxes];                 // G38 target coordinates for straight probe moves
	StraightProbeType type;                // Type of move
};

inline void StraightProbeSettings::AddMovingAxis(const size_t axis)
{
	SetBit(movingAxes, axis);
}

#endif /* SRC_MOVEMENT_STRAIGHTPROBESETTINGS_H_ */
