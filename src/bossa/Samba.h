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

#ifndef _SAMBA_H
#define _SAMBA_H

#include "General/StringRef.h"

#include "SerialPort.h"

#include "GCodes/GCodeException.h"

typedef GCodeException SambaError;

#define DEBUG_BOSSA (0)

class Samba
{
public:
    Samba() noexcept;
    virtual ~Samba();

    bool connect(SerialPort *_ecv_from port, int bps = 115200) noexcept;
    void disconnect() noexcept;

    void writeWord(uint32_t addr, uint32_t value) THROWS(GCodeException);
    uint32_t readWord(uint32_t addr) THROWS(GCodeException);

    void write(uint32_t addr, const uint8_t *_ecv_array buffer, int size) THROWS(GCodeException);
    void read(uint32_t addr, uint8_t *_ecv_array buffer, int size) THROWS(GCodeException);

    void go(uint32_t addr) THROWS(GCodeException);

#if DEBUG_BOSSA
    void setDebug(bool debug) noexcept { _debug = debug; }
#endif

    const SerialPort& getSerialPort() noexcept { return *_port; }

    // Extended SAM-BA functions
    bool canChipErase() noexcept { return _canChipErase; }

    bool canWriteBuffer() noexcept { return _canWriteBuffer; }
    uint32_t writeBufferSize() noexcept { return 4096; }

    bool canChecksumBuffer() noexcept { return _canChecksumBuffer; }
    uint32_t checksumBufferSize() noexcept { return 4096; }
    uint16_t crc16Calc(const uint8_t *_ecv_array data, int len) noexcept;

private:
    bool _canChipErase;
    bool _canWriteBuffer;
    bool _canChecksumBuffer;
    int _readBufferSize;
#if DEBUG_BOSSA
    bool _debug;
#endif
    SerialPort *_ecv_from _port;

    bool init() noexcept;

    void writeXmodem(const uint8_t *_ecv_array buffer, int size) THROWS(GCodeException);
    void readXmodem(uint8_t *_ecv_array buffer, int size) THROWS(GCodeException);

};


#endif // _SAMBA_H
