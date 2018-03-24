#ifndef CRC32_H
#define CRC32_H

#include <cstdint> // for uint32_t
#include <cstddef> // for size_t

class CRC32
{
private:
	static const uint32_t CRC_32_TAB[];
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