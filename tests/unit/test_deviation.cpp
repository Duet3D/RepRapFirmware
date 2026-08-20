#include <cmath>

#include "../../../RRFLibraries/src/Math/Deviation.h"

#include "../support/TestRunner.h"

TEST_CASE(DeviationCalculatesMeanAndSpread)
{
	Deviation deviation;
	deviation.Set(30.0f, 10.0f, 4);

	EXPECT_NEAR(deviation.GetMean(), 2.5, 1e-6);
	EXPECT_NEAR(deviation.GetDeviationFromMean(), std::sqrt(1.25), 1e-6);
}

TEST_CASE(DeviationHandlesEmptyInput)
{
	Deviation deviation;
	deviation.Set(123.0f, 456.0f, 0);

	EXPECT_EQ(deviation.GetMean(), 0.0f);
	EXPECT_EQ(deviation.GetDeviationFromMean(), 0.0f);
}

TEST_CASE(DeviationClampsTinyNegativeVariance)
{
	Deviation deviation;
	deviation.Set(4.0f, 4.0f, 4);

	EXPECT_EQ(deviation.GetMean(), 1.0f);
	EXPECT_EQ(deviation.GetDeviationFromMean(), 0.0f);
}