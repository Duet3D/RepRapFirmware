/*
 * StraightProbeSettings.h
 *
 *  Created on: 4 Oct 2019
 *      Author: manuel
 *
 *  This class holds information for G38 Straight Probe that otherwise would no longer be available
 *  since the GCodeBuffer can be overwritten when probe is deployed.
 */

#ifndef SRC_MOVEMENT_STRAIGHTPROBESETTINGS_H_
#define SRC_MOVEMENT_STRAIGHTPROBESETTINGS_H_

#include "RepRapFirmware.h"

enum class StraightProbeType : uint8_t {
	unset,
	towardsWorkpieceErrorOnFailure,  // probe toward workpiece, stop on contact, signal error if failure
	towardsWorkpiece,				 // probe toward workpiece, stop on contact
	awayFromWorkpieceErrorOnFailure, // probe away from workpiece, stop on loss of contact, signal error if failure
	awayFromWorkpiece				 // probe away from workpiece, stop on loss of contact
};

class StraightProbeSettings
{
public:
	StraightProbeSettings() noexcept;

	void Reset() noexcept;

	void SetCoordsToTarget(float[MaxAxes]) const noexcept;
	void SetTarget(const float[MaxAxes]) noexcept;						// Set target for G38 move

	const StraightProbeType GetType() const noexcept { return type; }
	void SetStraightProbeType(const StraightProbeType t) noexcept { type = t; }

	const AxesBitmap GetMovingAxes() const noexcept { return movingAxes; }
	void AddMovingAxis(const size_t) noexcept;

	const size_t GetZProbeToUse() const noexcept { return probeToUse; }
	void SetZProbeToUse(const size_t probeNumber) noexcept { probeToUse = probeNumber; }

	const bool ProbingAway() const noexcept;
	const bool SignalError() const noexcept;

private:
	AxesBitmap movingAxes;                 // Axes supposed to move - this is only used for manual probing
	size_t probeToUse;                     // Use this ZProbe
	float target[MaxAxes];                 // G38 target coordinates for straight probe moves
	StraightProbeType type;                // Type of move
};

inline void StraightProbeSettings::AddMovingAxis(const size_t axis) noexcept
{
	movingAxes.SetBit(axis);
}

inline const bool StraightProbeSettings::ProbingAway() const noexcept
{
	return type == StraightProbeType::awayFromWorkpieceErrorOnFailure || type == StraightProbeType::awayFromWorkpiece;
}

inline const bool StraightProbeSettings::SignalError() const noexcept
{
	return type == StraightProbeType::awayFromWorkpieceErrorOnFailure || type == StraightProbeType::towardsWorkpieceErrorOnFailure;
}

#endif /* SRC_MOVEMENT_STRAIGHTPROBESETTINGS_H_ */
