#include "../../../RRFLibraries/src/General/Bitmap.h"

#include "../support/TestRunner.h"

TEST_CASE(BitmapSetsClearsAndCombinesBits)
{
	Bitmap<uint32_t> bits;
	EXPECT_TRUE(bits.IsEmpty());

	bits.SetBit(1);
	bits.SetBit(4);
	EXPECT_TRUE(bits.IsBitSet(1));
	EXPECT_TRUE(bits.IsBitSet(4));
	EXPECT_EQ(bits.GetRaw(), 18u);

	bits.ClearBit(1);
	EXPECT_TRUE(!bits.IsBitSet(1));
	EXPECT_EQ(bits.GetRaw(), 16u);

	Bitmap<uint32_t> other;
	other.SetBit(2);
	other.SetBit(4);

	EXPECT_TRUE(bits.Intersects(other));
	EXPECT_TRUE(!bits.Disjoint(other));
	EXPECT_EQ((bits | other).GetRaw(), 20u);
	EXPECT_EQ((bits & other).GetRaw(), 16u);
	EXPECT_EQ((other - bits).GetRaw(), 4u);
}

TEST_CASE(BitmapContainsAndShiftOperations)
{
	Bitmap<uint16_t> bits;
	bits.SetBit(0);
	bits.SetBit(3);

	Bitmap<uint16_t> subset;
	subset.SetBit(3);

	EXPECT_TRUE(bits.Contains(subset));
	EXPECT_TRUE(bits.IsAnyBitSet(0, 1));
	EXPECT_TRUE(bits.IsAnyBitSet(1, 2, 3));
	EXPECT_EQ(bits.ShiftUp(2).GetRaw(), static_cast<uint16_t>(36));
}

TEST_CASE(ExtractBitHelpersMoveBitsCorrectly)
{
	EXPECT_EQ(ExtractBit<uint32_t>(0b0010u, 1, 4), 0b10000u);
	EXPECT_EQ(ExtractBit<uint32_t>(0b1000u, 3, 0), 0b0001u);
	EXPECT_EQ(ExtractTwoBits<uint32_t>(0b1100u, 2, 0), 0b0011u);
	EXPECT_EQ(ExtractTwoBits<uint32_t>(0b0011u, 0, 4), 0b110000u);
}