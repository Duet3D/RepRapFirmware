/*
 * ZLeadscrewKinematics.h
 *
 *  Created on: 8 Jul 2017
 *      Author: David
 */

#ifndef SRC_MOVEMENT_KINEMATICS_ZLEADSCREWKINEMATICS_H_
#define SRC_MOVEMENT_KINEMATICS_ZLEADSCREWKINEMATICS_H_

#include "Kinematics.h"

class ZLeadscrewKinematics : public Kinematics
{
public:
	ZLeadscrewKinematics(KinematicsType k);
	bool Configure(unsigned int mCode, GCodeBuffer& gb, StringRef& reply, bool& error) override;
	bool SupportsAutoCalibration() const override;
	void DoAutoCalibration(size_t numFactors, const RandomProbePointSet& probePoints, StringRef& reply) override;

#ifdef DUET_NG
	bool WriteResumeSettings(FileStore *f) const override;
#endif

private:
	void AppendCorrections(const floatc_t corrections[], StringRef& reply) const;

	static const unsigned int MaxLeadscrews = 4;

	unsigned int numLeadscrews;
	float leadscrewX[MaxLeadscrews], leadscrewY[MaxLeadscrews];
	float maxCorrection;
};

#endif /* SRC_MOVEMENT_KINEMATICS_ZLEADSCREWKINEMATICS_H_ */
