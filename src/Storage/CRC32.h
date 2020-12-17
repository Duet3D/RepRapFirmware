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
	void Update(const char *s, size_t len) noexcept
		__attribute__((optimize("no-unroll-loops")));	// we already optimised the loops, and on the SAME5x unrolling them could make us feed data to the CRC unit too fast
	void Reset(uint32_t initialValue = 0xFFFFFFFF) noexcept;
	uint32_t Get() const noexcept;
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
