/*
 * AveragingFilter.h
 *
 *  Created on: 17 Oct 2023
 *      Author: David
 */

#ifndef SRC_PLATFORM_AVERAGINGFILTER_H_
#define SRC_PLATFORM_AVERAGINGFILTER_H_

// Class to perform averaging of values read from the ADC
// numAveraged should be a power of 2 for best efficiency
template<size_t numAveraged> class AveragingFilter
{
public:
	AveragingFilter() noexcept
	{
		Init(0);
	}

	void Init(uint16_t val) volatile noexcept
	{
		AtomicCriticalSectionLocker lock;

		sum = (uint32_t)val * (uint32_t)numAveraged;
		index = 0;
		isValid = false;
		for (size_t i = 0; i < numAveraged; ++i)
		{
			readings[i] = val;
		}
	}

	// Call this to put a new reading into the filter
	// This is called by the ISR and by the ADC callback function
	void ProcessReading(uint16_t r) volatile noexcept
	{
		size_t locIndex = index;				// avoid repeatedly reloading volatile variable
		sum = sum - readings[locIndex] + r;
		readings[locIndex] = r;
		++locIndex;
		if (locIndex == numAveraged)
		{
			locIndex = 0;
			isValid = true;
		}
		index = locIndex;
	}

	// Return the raw sum
	uint32_t GetSum() const volatile noexcept
	{
		return sum;
	}

	// Return true if we have a valid average
	bool IsValid() const volatile noexcept
	{
		return isValid;
	}

	static constexpr size_t NumAveraged() noexcept { return numAveraged; }

	// Function used as an ADC callback to feed a result into an averaging filter
	static void CallbackFeedIntoFilter(CallbackParameter cp, uint32_t val) noexcept;

private:
	uint16_t readings[numAveraged];
	size_t index;
	uint32_t sum;
	bool isValid;
	//invariant(sum == + over readings)
	//invariant(index < numAveraged)
};

template<size_t numAveraged> void AveragingFilter<numAveraged>::CallbackFeedIntoFilter(CallbackParameter cp, uint32_t val) noexcept
{
	static_cast<AveragingFilter<numAveraged>*>(cp.vp)->ProcessReading((uint16_t)val);
}

#endif /* SRC_PLATFORM_AVERAGINGFILTER_H_ */
