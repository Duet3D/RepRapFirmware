#ifndef UTIL_H
#define UTIL_H

static inline uint16_t htons(uint16_t x)
{
	return (x << 8) | ((x >> 8) & 0xFF);
}

#define ntohs(x) htons(x)

#define htonl(x) ( ((x)<<24 & 0xFF000000UL) | \
                   ((x)<< 8 & 0x00FF0000UL) | \
                   ((x)>> 8 & 0x0000FF00UL) | \
                   ((x)>>24 & 0x000000FFUL) )
#define ntohl(x) htonl(x)

// Get a 16 bit word in network byte order from a buffer
static inline uint16_t get16nb(const uint8_t *buf)
{
	return ((uint16_t)buf[0] << 8) | buf[1];
}

// Get a 32 bit word in network byte order from a buffer
static inline uint16_t get32nb(const uint8_t *buf)
{
	return ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | buf[3];
}

// Get a 16 bit word in native byte order from a buffer
static inline uint16_t get16(const uint8_t *buf)
{
	return ((uint16_t)buf[1] << 8) | buf[0];
}

// Put a 16-bit word in native byte order in a buffer
static inline void put16(uint8_t *buf, uint16_t n)
{
	buf[0] = (uint8_t)n;
	buf[1] = (uint8_t)(n >> 8);
}

#endif
