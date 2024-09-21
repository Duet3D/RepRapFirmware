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

	void Update(uint8_t c) noexcept { UpdateNormal(c, crc16_xmodem_table); }
	void Update(const uint8_t *_ecv_array c, size_t len) noexcept pre(c.lim >= len) { UpdateNormal(c, len, crc16_xmodem_table); }

#if SUPPORT_MODBUS_RTU
	void UpdateModbus(uint8_t c) noexcept pre(table.lim == 256) { UpdateReflected(c, crc16_modbus_table); }
	void UpdateModbus(const uint8_t *c, size_t len) noexcept pre(c.lim >= len) { UpdateReflected(c, len, crc16_modbus_table); }
#endif

private:
	void UpdateNormal(uint8_t c, const uint16_t *_ecv_array table) noexcept pre(table.lim == 256);
	void UpdateNormal(const uint8_t *_ecv_array c, size_t len, const uint16_t *_ecv_array table) noexcept pre(c.lim >= len; table.lim == 256);
	void UpdateReflected(uint8_t c, const uint16_t *_ecv_array table) noexcept pre(table.lim == 256);
	void UpdateReflected(const uint8_t *_ecv_array c, size_t len, const uint16_t *_ecv_array table) noexcept pre(c.lim >= len; table.lim == 256);

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
