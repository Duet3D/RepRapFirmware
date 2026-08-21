/*
 * ZLeadscrewKinematics.h
 *
 *  Created on: 8 Jul 2017
 *      Author: David
 */

#ifndef SRC_MOVEMENT_KINEMATICS_ZLEADSCREWKINEMATICS_H_
#define SRC_MOVEMENT_KINEMATICS_ZLEADSCREWKINEMATICS_H_

#include "Kinematics.h"

// This is used as the base class for any kinematic that supports auto or manual bed levelling (as distinct from bed compensation) using leadscrews or bed adjusting screws.
class ZLeadscrewKinematics : public Kinematics
{
public:
	ZLeadscrewKinematics(KinematicsType k) noexcept;
	ZLeadscrewKinematics(KinematicsType t, SegmentationType segType) noexcept;

	bool Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) THROWS(GCodeException) override;
	bool SupportsAutoCalibration() const noexcept override;
	bool DoAutoCalibration(MovementState& ms, size_t numFactors, const RandomProbePointSet& probePoints, const StringRef& reply) noexcept override;

	// Return the ordered M671 geometry without exposing mutable calibration state.
	// The index order matches the physical Z drivers configured by M584.
	size_t GetNumLeadscrews() const noexcept { return numLeadscrews; }
	bool GetLeadscrewCoordinates(size_t index, float& x, float& y) const noexcept
	{
		if (index >= numLeadscrews)
		{
			return false;
		}
		x = leadscrewX[index];
		y = leadscrewY[index];
		return true;
	}

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	bool WriteResumeSettings(FileStore *f) const noexcept override;
#endif

protected:
	DECLARE_OBJECT_MODEL_WITH_ARRAYS

private:
	void AppendCorrections(const floatc_t corrections[], const StringRef& reply) const noexcept;

	static const unsigned int MaxLeadscrews = 4;			// the maximum we support for auto bed levelling

	unsigned int numLeadscrews;
	float leadscrewX[MaxLeadscrews], leadscrewY[MaxLeadscrews];
	float correctionFactor;
	float maxCorrection;
	float screwPitch;
	float lastCorrections[MaxLeadscrews];
};

#endif /* SRC_MOVEMENT_KINEMATICS_ZLEADSCREWKINEMATICS_H_ */
