/*
 * RemoteFan.h
 *
 *  Created on: 3 Sep 2019
 *      Author: David
 */

#ifndef SRC_FANS_REMOTEFAN_H_
#define SRC_FANS_REMOTEFAN_H_

#include "Fan.h"

#if SUPPORT_CAN_EXPANSION

class RemoteFan : public Fan
{
public:
	RemoteFan(unsigned int fanNum);
	~RemoteFan();

	bool Check() override;									// update the fan PWM returning true if it is a thermostatic fan that is on
	bool IsEnabled() const override;
	void SetPwmFrequency(PwmFrequency freq) override;
	int32_t GetRPM() override;
	void AppendPortDetails(const StringRef& str) const override;

protected:
	void UpdateFanConfiguration() override;
	void Refresh() override;

private:
	static constexpr uint32_t RpmReadingTimeout = 2000;		// any reading older than this number of milliseconds is considered unreliable

	int32_t lastRpm;
	uint32_t whenLastRpmReceived;
};

#endif

#endif /* SRC_FANS_REMOTEFAN_H_ */
