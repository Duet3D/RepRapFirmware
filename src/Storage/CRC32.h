#ifndef CRC32_H
#define CRC32_H

#include <RepRapFirmware.h>

#if SAME5x
# define USE_SAME5x_HARDWARE_CRC	1
#else
# define USE_SAME5x_HARDWARE_CRC	0
#endif

// Note: when USE_SAME5x_HARDWARE_CRC is true, the Update methods must not be called from an ISR!
class CRC32
{
private:
	uint32_t crc;

public:
	CRC32() noexcept;

	void Update(char c) noexcept;
	void Update(const char *_ecv_array s, size_t len) noexcept pre(s.lim >= len)
		__attribute__((optimize("no-unroll-loops")));	// we already optimised the loops, and on the SAME5x unrolling them could make us feed data to the CRC unit too fast
	void Reset(uint32_t initialValue = 0xFFFFFFFFu) noexcept;
	uint32_t Get() const noexcept;

#if USE_SAME5x_HARDWARE_CRC
	// Special function used to CRC a whole number of 32-bit words aligned on a word boundary, used to check for memory corruption
	static uint32_t CalcCRC32(const uint32_t *_ecv_array data, const uint32_t *_ecv_array end) noexcept;
		__attribute__((optimize("no-unroll-loops")));	// we already optimised the loops, and on the SAME5x unrolling them could make us feed data to the CRC unit too fast
#endif
};

inline uint32_t CRC32::Get() const noexcept
{
	return ~crc;
}

inline void CRC32::Reset(uint32_t initialValue) noexcept
{
	crc = initialValue;
}

#endif
