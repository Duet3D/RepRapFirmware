/*
 * CRC16.h
 *
 *  Created on: 4 Dec 2020
 *      Author: David
 *
 * This calculates CRC16 CCIT with initial CRC value Zero.
 */

#ifndef SRC_STORAGE_CRC16_H_
#define SRC_STORAGE_CRC16_H_

#include <RepRapFirmware.h>

class CRC16
{
public:
	CRC16() noexcept;

	void Reset(uint16_t initialValue) noexcept;
	uint16_t Get() const noexcept;

	void Update(char c) noexcept { Update(c, crc16_xmodem_table); }
	void Update(const char *c, size_t len) noexcept { Update(c, len, crc16_xmodem_table); }

#if SUPPORT_MODBUS_RTU
	void UpdateModbus(char c) noexcept { Update(c, crc16_modbus_table); }
	void UpdateModbus(const char *c, size_t len) noexcept { Update(c, len, crc16_modbus_table); }
#endif

private:
	void Update(char c, const uint16_t *table) noexcept;
	void Update(const char *c, size_t len, const uint16_t *table) noexcept;

	static const uint16_t crc16_xmodem_table[];

#if SUPPORT_MODBUS_RTU
	static const uint16_t crc16_modbus_table[];
#endif

	uint16_t crc;
};

inline CRC16::CRC16() noexcept
{
	Reset(0);
}

inline uint16_t CRC16::Get() const noexcept
{
	return crc;
}

inline void CRC16::Reset(uint16_t initialValue) noexcept
{
	crc = initialValue;
}

#endif /* SRC_STORAGE_CRC16_H_ */
