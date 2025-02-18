///////////////////////////////////////////////////////////////////////////////
// BOSSA
//
// Copyright (c) 2011-2018, ShumaTech
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the <organization> nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
///////////////////////////////////////////////////////////////////////////////

#include "Samba.h"

#include "RepRapFirmware.h"
#include "Storage/CRC16.h"

// XMODEM definitions
constexpr size_t BLK_SIZE = 128;
constexpr unsigned int MAX_RETRIES = 5;
#define SOH         0x01
#define EOT         0x04
#define ACK         0x06
#define NAK         0x15
#define CAN         0x18
#define START       'C'

#define TIMEOUT_QUICK   100
#define TIMEOUT_NORMAL  1000
#define TIMEOUT_LONG    5000

bool Samba::init() noexcept
{
    uint8_t cmd[3];

    _port->timeout(TIMEOUT_QUICK);

    // Flush garbage
    uint8_t dummy[1024];
    _port->read(dummy, sizeof(dummy));

    // Set binary mode
#if DEBUG_BOSSA
	debugPrintf("Set binary mode\n");
#endif

    cmd[0] = (uint8_t)'N';
    cmd[1] = (uint8_t)'#';
    _port->write(cmd, 2);
    _port->read(cmd, 2);

    _port->timeout(TIMEOUT_NORMAL);

    return true;
}

bool Samba::connect(SerialPort *_ecv_from port, int bps) noexcept
{
    _port = port;

    // Try the serial port at slower speed
    if (_port->open(bps) && init())
    {
#if DEBUG_BOSSA
		debugPrintf("Connected at %d baud\n", bps);
#endif
        return true;
    }

    disconnect();
    return false;
}

void Samba::disconnect() noexcept
{
    _port->close();
}


void Samba::writeWord(uint32_t addr, uint32_t value) THROWS(GCodeException)
{
    uint8_t cmd[20];

#if DEBUG_BOSSA
	debugPrintf("%s(addr=%#" PRIx32 ",value=%#" PRIx32 ")\n", __FUNCTION__, addr, value);
#endif

    SafeSnprintf((char *_ecv_array) cmd, sizeof(cmd), "W%08" PRIX32 ",%08" PRIX32 "#", addr, value);
    if (_port->write(cmd, sizeof(cmd) - 1) != (int)sizeof(cmd) - 1)
        throw SambaError("Samba::writeWord: write failed");
}


uint32_t Samba::readWord(uint32_t addr) THROWS(GCodeException)
{
    uint8_t cmd[13];

    SafeSnprintf((char *_ecv_array) cmd, sizeof(cmd), "w%08" PRIX32 ",4#", addr);
    if (_port->write(cmd, sizeof(cmd) - 1) != (int)sizeof(cmd) - 1)
        throw SambaError("Samba::readWord: write failed");
    if (_port->read(cmd, sizeof(uint32_t)) != (int)sizeof(uint32_t))
        throw SambaError("Samba::readWord: read failed");

    const uint32_t value = (cmd[3] << 24 | cmd[2] << 16 | cmd[1] << 8 | cmd[0] << 0);

#if DEBUG_BOSSA
	debugPrintf("%s(addr=%#" PRIx32 ")=%#" PRIx32 "\n", __FUNCTION__, addr, value);
#endif

    return value;
}

void Samba::readXmodem(uint8_t *_ecv_array buffer, size_t size) THROWS(GCodeException)
{
    uint8_t blk[BLK_SIZE + 5];
    uint32_t blkNum = 1;
    unsigned int retries;
    int bytes;
    CRC16 crc16;

    while (size != 0)
    {
        for (retries = 0; retries < MAX_RETRIES; retries++)
        {
            if (blkNum == 1)
            {
                _port->put(START);
            }

            bytes = _port->read(blk, sizeof(blk));
            crc16.Reset(0);
            crc16.Update(blk + 3, BLK_SIZE);
            const uint16_t receivedCRC16 = blk[BLK_SIZE + 3] << 8 | blk[BLK_SIZE + 4];
            if (   bytes == (int)sizeof(blk)
            	&& blk[0] == SOH
				&& blk[1] == (blkNum & 0xff)
				&& crc16.Get() == receivedCRC16
			   )
                break;

            if (blkNum != 1)
            {
                _port->put(NAK);
            }
        }
        if (retries == MAX_RETRIES)
            throw SambaError("Samba::readXmodem: too many retries 1");

        _port->put(ACK);

		const size_t sizeToMove = min<size_t>(size, BLK_SIZE);
        memmove(buffer, blk + 3, sizeToMove);
        buffer += BLK_SIZE;
        size -= sizeToMove;
        blkNum++;
    }

    for (retries = 0; retries < MAX_RETRIES; retries++)
    {
        if (_port->get() == EOT)
        {
            _port->put(ACK);
            break;
        }
        _port->put(NAK);
    }
    if (retries == MAX_RETRIES)
        throw SambaError("Samba::readXmodem: too many retries 2");
}

void Samba::writeXmodem(const uint8_t *_ecv_array buffer, size_t size) THROWS(GCodeException)
{
    uint8_t blk[BLK_SIZE + 5];
    uint32_t blkNum = 1;
    unsigned int retries;
    int bytes;

    for (retries = 0; retries < MAX_RETRIES; retries++)
    {
        if (_port->get() == (int)START)
            break;
    }
    if (retries == MAX_RETRIES)
        throw SambaError("Samba::writeXmodem: too many retries getting START");

    CRC16 crc16;
    while (size != 0)
    {
        blk[0] = SOH;
        blk[1] = (blkNum & 0xff);
        blk[2] = ~(blkNum & 0xff);
        const size_t sizeToMove = min<size_t>(size, BLK_SIZE);
        memmove(blk + 3, buffer, sizeToMove);
        if (size < BLK_SIZE)
        {
            memset(blk + 3 + size, 0, BLK_SIZE - size);
        }

        crc16.Reset(0);
    	crc16.Update(blk + 3, BLK_SIZE);
        const uint16_t checksum = crc16.Get();
        blk[BLK_SIZE + 3] = (checksum >> 8) & 0xff;
        blk[BLK_SIZE + 4] = checksum & 0xff;

        for (retries = 0; retries < MAX_RETRIES; retries++)
        {
            bytes = _port->write(blk, sizeof(blk));
            if (bytes != (int)sizeof(blk))
                throw SambaError("Samba::writeXmodem: write failed");

            if (_port->get() == ACK)
                break;
        }

        if (retries == MAX_RETRIES)
            throw SambaError("Samba::writeXmodem: too many retries 2");

        buffer += BLK_SIZE;
        size -= sizeToMove;
        blkNum++;
    }

    for (retries = 0; retries < MAX_RETRIES; retries++)
    {
        _port->put(EOT);
        if (_port->get() == ACK)
            break;
    }
    if (retries == MAX_RETRIES)
        throw SambaError("Samba::writeXmodem: too many retries 3");
}

void Samba::read(uint32_t addr, uint8_t *_ecv_array buffer, size_t size) THROWS(GCodeException)
{
    uint8_t cmd[20];

#if DEBUG_BOSSA
	debugPrintf("%s(addr=%#" PRIx32 ",size=%#x)\n", __FUNCTION__, addr, size);
#endif

	SafeSnprintf((char *_ecv_array) cmd, sizeof(cmd), "R%08" PRIX32 ",%08X#", addr, size);
	if (_port->write(cmd, sizeof(cmd) - 1) != (int)sizeof(cmd) - 1)
		throw SambaError("Samba::read: write failed");

	readXmodem(buffer, size);
}

void Samba::write(uint32_t addr, const uint8_t *_ecv_array buffer, size_t size) THROWS(GCodeException)
{
    uint8_t cmd[20];

#if DEBUG_BOSSA
	debugPrintf("%s(addr=%#" PRIx32 ",size=%#x)\n", __FUNCTION__, addr, size);
#endif

	SafeSnprintf((char *_ecv_array) cmd, sizeof(cmd), "S%08" PRIX32 ",%08X#", addr, size);
	if (_port->write(cmd, sizeof(cmd) - 1) != (int)sizeof(cmd) - 1)
		throw SambaError("Samba::write: write failed");

	writeXmodem(buffer, size);
}

void Samba::go(uint32_t addr) THROWS(GCodeException)
{
    uint8_t cmd[11];

#if DEBUG_BOSSA
	debugPrintf("%s(addr=%#" PRIx32 ")\n", __FUNCTION__, addr);
#endif

    SafeSnprintf((char *_ecv_array) cmd, sizeof(cmd), "G%08" PRIX32 "#", addr);
    if (_port->write(cmd, sizeof(cmd) - 1) != (int)sizeof(cmd) - 1)
        throw SambaError("Samba::go: write failed");
}

// End
