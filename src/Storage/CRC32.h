#ifndef CRC32_H
#define CRC32_H

#include <RepRapFirmware.h>

class CRC32
{
private:
	uint32_t crc;

public:
	CRC32() noexcept;
	~CRC32();
	void Update(char c) noexcept;
	void Update(const char *s, size_t len) noexcept;
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
