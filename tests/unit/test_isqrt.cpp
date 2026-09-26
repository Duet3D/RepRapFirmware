#include <limits>

#include "../../../RRFLibraries/src/Math/Isqrt.h"

#include "../support/TestRunner.h"

TEST_CASE(Isqrt64Handles32BitInputs)
{
	EXPECT_EQ(isqrt64(0), 0u);
	EXPECT_EQ(isqrt64(1), 1u);
	EXPECT_EQ(isqrt64(15), 3u);
	EXPECT_EQ(isqrt64(16), 4u);
	EXPECT_EQ(isqrt64(17), 4u);
	EXPECT_EQ(isqrt64(4294967295ULL), 65535u);
}

TEST_CASE(Isqrt64HandlesLarge62BitInputs)
{
	EXPECT_EQ(isqrt64(4611686014132420609ULL), 2147483647u);
	EXPECT_EQ(isqrt64(4611686018427387903ULL), 2147483647u);
	EXPECT_EQ(isqrt64(1125899906842624ULL), 33554432u);
}

TEST_CASE(Isqrt64RejectsOutOfRangeInputs)
{
	EXPECT_EQ(isqrt64(0xC000000000000000ULL), std::numeric_limits<uint32_t>::max());
}

TEST_CASE(FastSqrtfMatchesReferenceForNormalInputs)
{
	EXPECT_NEAR(fastSqrtf(0.25f), 0.5, 1e-6);
	EXPECT_NEAR(fastSqrtf(2.0f), std::sqrt(2.0), 1e-6);
	EXPECT_NEAR(fastSqrtf(1234.5f), std::sqrt(1234.5), 1e-4);
	EXPECT_EQ(fastSqrtf(-1.0f), 0.0f);
}