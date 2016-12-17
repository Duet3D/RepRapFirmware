/*
 * Copyright (c) 2010 by WIZnet <support@wiznet.co.kr>
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 *
 * - 10 Apr. 2015
 * Added support for Arduino Ethernet Shield 2
 * by Arduino.org team
 */

#include <stdio.h>
#include <string.h>
#include "Core.h"

#include "utility/w5500.h"
#include "utility/WizSpi.h"

// W5500 controller instance
W5500Class w5500;


void W5500Class::init(void)
{
	WizSpi::Init();

	for (int i=0; i<MAX_SOCK_NUM; i++)
	{
        const uint8_t cntl_byte = (0x0C + (i<<5));
        write( 0x1E, cntl_byte, 2); //0x1E - Sn_RXBUF_SIZE
        write( 0x1F, cntl_byte, 2); //0x1F - Sn_TXBUF_SIZE
    }
}

void W5500Class::stop()
{
	WizSpi::Stop();
}

uint16_t W5500Class::getTXFreeSize(SOCKET s)
{
    uint16_t val=0, val1=0;
    do {
        val1 = readSnTX_FSR(s);
        if (val1 != 0)
            val = readSnTX_FSR(s);
    }
    while (val != val1);
    return val;
}

uint16_t W5500Class::getRXReceivedSize(SOCKET s)
{
    uint16_t val=0,val1=0;
    do {
        val1 = readSnRX_RSR(s);
        if (val1 != 0)
            val = readSnRX_RSR(s);
    }
    while (val != val1);
    return val;
}

void W5500Class::send_data_processing(SOCKET s, const uint8_t *data, uint16_t len)
{
  // This is same as having no offset in a call to send_data_processing_offset
  send_data_processing_offset(s, 0, data, len);
}

void W5500Class::send_data_processing_offset(SOCKET s, uint16_t data_offset, const uint8_t *data, uint16_t len)
{
    uint16_t ptr = readSnTX_WR(s);
    uint8_t cntl_byte = (0x14+(s<<5));
    ptr += data_offset;
    write(ptr, cntl_byte, data, len);
    ptr += len;
    writeSnTX_WR(s, ptr);

}

void W5500Class::recv_data_processing(SOCKET s, uint8_t *data, uint16_t len, uint8_t peek)
{
    uint16_t ptr = readSnRX_RD(s);

    read_data(s, ptr, data, len);
    if (!peek)
    {
        ptr += len;
        writeSnRX_RD(s, ptr);
    }
}

void W5500Class::read_data(SOCKET s, volatile uint16_t src, volatile uint8_t *dst, uint16_t len)
{
    uint8_t cntl_byte = (0x18+(s<<5));
    read((uint16_t)src , cntl_byte, (uint8_t *)dst, len);
}

uint8_t W5500Class::write(uint16_t _addr, uint8_t _cb, uint8_t _data)
{
	WizSpi::AssertSS();
	WizSpi::ExchangeByte(_addr >> 8, false);
	WizSpi::ExchangeByte(_addr & 0xFF, false);
	WizSpi::ExchangeByte(_cb, false);
	WizSpi::ExchangeByte(_data, true);
	WizSpi::ReleaseSS();
	return 1;
}

uint16_t W5500Class::write(uint16_t _addr, uint8_t _cb, const uint8_t *_buf, uint16_t _len)
{
	WizSpi::AssertSS();
	WizSpi::ExchangeByte(_addr >> 8, false);
	WizSpi::ExchangeByte(_addr & 0xFF, false);
	WizSpi::ExchangeByte(_cb, false);
	WizSpi::SendBurst(_buf, _len);
    WizSpi::ReleaseSS();
    return _len;
}

uint8_t W5500Class::read(uint16_t _addr, uint8_t _cb)
{
	WizSpi::AssertSS();
	WizSpi::ExchangeByte(_addr >> 8, false);
	WizSpi::ExchangeByte(_addr & 0xFF, false);
	WizSpi::ExchangeByte(_cb, false);
    const uint8_t _data = WizSpi::ExchangeByte(0, true);
    WizSpi::ReleaseSS();
    return _data;
}

uint16_t W5500Class::read(uint16_t _addr, uint8_t _cb, uint8_t *_buf, uint16_t _len)
{
	WizSpi::AssertSS();
	WizSpi::ExchangeByte(_addr >> 8, false);
	WizSpi::ExchangeByte(_addr & 0xFF, false);
	WizSpi::ExchangeByte(_cb, false);
	WizSpi::ReadBurst(_buf, _len);
    WizSpi::ReleaseSS();
    return _len;
}

void W5500Class::execCmdSn(SOCKET s, SockCMD _cmd) {
    // Send command to socket
    writeSnCR(s, _cmd);
    // Wait for command to complete
    while (readSnCR(s))
    ;
}

// End
