#include "RepRapFirmware.h"

// Fast 62-bit integer square root function (thanks dmould)
uint32_t isqrt64(uint64_t num)
{
	uint32_t numHigh = (uint32_t)(num >> 32);
	if (numHigh == 0)
	{
		// 32-bit square root - thanks to Wilco Dijksra for this efficient ARM algorithm
		uint32_t num32 = (uint32_t)num;
		uint32_t res = 0;

		#define iter32(N)						\
		{										\
			uint32_t temp = res | (1 << N);		\
			if (num32 >= temp << N)				\
			{									\
				num32 -= temp << N;				\
				res |= 2 << N;					\
			}									\
		}

		// We need to do 16 iterations
		iter32(15); iter32(14); iter32(13); iter32(12);
		iter32(11); iter32(10); iter32(9); iter32(8);
		iter32(7); iter32(6); iter32(5); iter32(4);
		iter32(3); iter32(2); iter32(1); iter32(0);

		return res >> 1;
	}
	else if ((numHigh & (3u << 30)) != 0)
	{
		// Input out of range - probably negative, so return -1
		return 0xFFFFFFFF;
	}
	else
	{
		// 62-bit square root
		uint32_t res = 0;

#define iter64a(N) 								\
		{										\
			res <<= 1;							\
			uint32_t temp = (res | 1) << (N);	\
			if (numHigh >= temp)				\
			{									\
				numHigh -= temp;				\
				res |= 2;						\
			}									\
		}

		// We need to do 15 iterations (not 16 because we have eliminated the top 2 bits)
					iter64a(28) iter64a(26) iter64a(24)
		iter64a(22) iter64a(20) iter64a(18) iter64a(16)
		iter64a(14) iter64a(12) iter64a(10) iter64a(8)
		iter64a(6)  iter64a(4)  iter64a(2)  iter64a(0)

		// resHigh is twice the square root of the msw, in the range 0..2^16-1 with the input restricted to 62 bits
		uint64_t numAll = ((uint64_t)numHigh << 32) | (uint32_t)num;

#define iter64b(N) 										\
		{												\
			res <<= 1;									\
			uint64_t temp = ((uint64_t)res | 1) << (N);	\
			if (numAll >= temp)							\
			{											\
				numAll -= temp;							\
				res |= 2;								\
			}											\
		}

		// We need to do 16 iterations.
		// After the last iteration, numAll may be between 0 and (1 + 2 * res) inclusive.
		// So to take square roots of numbers up to 62 bits, we need to do all these iterations using 64 bit maths.
		// If we restricted the input to e.g. 48 bits, then we could do some of the final iterations using 32-bit maths.
		iter64b(30) iter64b(28) iter64b(26) iter64b(24)
		iter64b(22) iter64b(20) iter64b(18) iter64b(16)
		iter64b(14) iter64b(12) iter64b(10) iter64b(8)
		iter64b(6)  iter64b(4)  iter64b(2)  iter64b(0)

		return res >> 1;

#undef iter64a
#undef iter64b

	}
}

// End
