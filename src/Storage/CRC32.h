#ifndef CRC32_H
#define CRC32_H

#include <RepRapFirmware.h>

class CRC32
{
private:
	uint32_t crc;

public:
	CRC32();
	void Update(char c);
	void Update(const char *c, size_t len);
	void Reset();
	uint32_t Get() const;
};

inline uint32_t CRC32::Get() const
{
	return ~crc;
}

#endif
