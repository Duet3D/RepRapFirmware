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
	ZLeadscrewKinematics(KinematicsType k);
	ZLeadscrewKinematics(KinematicsType t, float segsPerSecond, float minSegLength, bool doUseRawG0);

	bool Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) override;
	bool SupportsAutoCalibration() const override;
	bool DoAutoCalibration(size_t numFactors, const RandomProbePointSet& probePoints, const StringRef& reply) override;
	bool WriteResumeSettings(FileStore *f) const override;

private:
	void AppendCorrections(const floatc_t corrections[], const StringRef& reply) const;

	static const unsigned int MaxLeadscrews = 4;

	unsigned int numLeadscrews;
	float leadscrewX[MaxLeadscrews], leadscrewY[MaxLeadscrews];
	float correctionFactor;
	float maxCorrection;
	float screwPitch;
};

#endif /* SRC_MOVEMENT_KINEMATICS_ZLEADSCREWKINEMATICS_H_ */
