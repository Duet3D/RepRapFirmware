#ifndef CRC32_H
#define CRC32_H

#include <RepRapFirmware.h>

class CRC32
{
private:
	uint32_t crc;

public:
	CRC32() noexcept;
	void Update(char c) noexcept;
	void Update(const char *c, size_t len) noexcept;
	void Reset() noexcept;
	uint32_t Get() const noexcept;
};

inline uint32_t CRC32::Get() const noexcept
{
	return ~crc;
}

#endif
