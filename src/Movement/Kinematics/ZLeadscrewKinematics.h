/*
 * ZLeadscrewKinematics.h
 *
 *  Created on: 8 Jul 2017
 *      Author: David
 */

#ifndef SRC_MOVEMENT_KINEMATICS_ZLEADSCREWKINEMATICS_H_
#define SRC_MOVEMENT_KINEMATICS_ZLEADSCREWKINEMATICS_H_

#include "Kinematics.h"

// This is used as the base class for any kinematic that supports auto or manual bed levelling (as distinct from bed compensation)
// using leadscrews or bed adjusting screws.
class ZLeadscrewKinematics : public Kinematics
{
public:
	ZLeadscrewKinematics(KinematicsType k) noexcept;
	ZLeadscrewKinematics(KinematicsType t, float segsPerSecond, float minSegLength, bool doUseRawG0) noexcept;

	bool Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) THROWS(GCodeException) override;
	bool SupportsAutoCalibration() const noexcept override;
	bool DoAutoCalibration(size_t numFactors, const RandomProbePointSet& probePoints, const StringRef& reply) noexcept override;
#if HAS_MASS_STORAGE
	bool WriteResumeSettings(FileStore *f) const noexcept override;
#endif

private:
	void AppendCorrections(const floatc_t corrections[], const StringRef& reply) const noexcept;

	static const unsigned int MaxLeadscrews = 8;			// some Folgertech FT5 printers have 8 bed adjusting screws

	unsigned int numLeadscrews;
	float leadscrewX[MaxLeadscrews], leadscrewY[MaxLeadscrews];
	float correctionFactor;
	float maxCorrection;
	float screwPitch;
};

#endif /* SRC_MOVEMENT_KINEMATICS_ZLEADSCREWKINEMATICS_H_ */
