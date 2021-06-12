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
#ifndef _EEFCFLASH_H
#define _EEFCFLASH_H

#include <cstdint>

#include "BossaFlash.h"

class EefcFlash : public BossaFlash
{
public:
    EefcFlash(Samba& samba,
    		  const char* name,
              uint32_t addr,
              uint32_t pages,
              uint32_t size,
              uint32_t planes,
              uint32_t lockRegions,
              uint32_t user,
              uint32_t stack,
              uint32_t regs,
              bool canBrownout) THROWS(GCodeException);
    virtual ~EefcFlash();

    void eraseAll(uint32_t offset) THROWS(GCodeException) override;
    void eraseAuto(bool enable) noexcept override;

    Vector<bool, 16> getLockRegions() THROWS(GCodeException) override;

#if ORIGINAL_BOSSA_CODE
    bool getSecurity();

    bool getBod();
#endif
    bool canBod() noexcept override { return _canBrownout; }

#if ORIGINAL_BOSSA_CODE
    bool getBor();
#endif
    bool canBor() noexcept override { return _canBrownout; }

    bool getBootFlash() THROWS(GCodeException) override;
    bool canBootFlash() noexcept override { return true; }

    void writeOptions() THROWS(GCodeException) override;

    void writePage(uint32_t page) THROWS(GCodeException) override;
    void readPage(uint32_t page, uint8_t* data) THROWS(GCodeException) override;

    static const uint32_t PagesPerErase;

private:
    uint32_t _regs;
    bool _canBrownout;
    bool _eraseAuto;

    void waitFSR(int seconds = 1) THROWS(GCodeException);
    void writeFCR0(uint8_t cmd, uint32_t arg) THROWS(GCodeException);
    void writeFCR1(uint8_t cmd, uint32_t arg) THROWS(GCodeException);
    uint32_t readFRR0() THROWS(GCodeException);
    uint32_t readFRR1() THROWS(GCodeException);
};

#endif // _EEFCFLASH_H
