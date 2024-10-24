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
#include "EefcFlash.h"

#define EEFC_KEY        0x5a

#define EEFC0_FMR       (_regs + 0x00)
#define EEFC0_FCR       (_regs + 0x04)
#define EEFC0_FSR       (_regs + 0x08)
#define EEFC0_FRR       (_regs + 0x0C)

#define EEFC1_FMR       (_regs + 0x200)
#define EEFC1_FCR       (_regs + 0x204)
#define EEFC1_FSR       (_regs + 0x208)
#define EEFC1_FRR       (_regs + 0x20C)

#define EEFC_FCMD_GETD  0x0
#define EEFC_FCMD_WP    0x1
#define EEFC_FCMD_WPL   0x2
#define EEFC_FCMD_EWP   0x3
#define EEFC_FCMD_EWPL  0x4
#define EEFC_FCMD_EA    0x5
#define EEFC_FCMD_EPA   0x7
#define EEFC_FCMD_SLB   0x8
#define EEFC_FCMD_CLB   0x9
#define EEFC_FCMD_GLB   0xa
#define EEFC_FCMD_SGPB  0xb
#define EEFC_FCMD_CGPB  0xc
#define EEFC_FCMD_GGPB  0xd

const uint32_t
EefcFlash::PagesPerErase = 8;

EefcFlash::EefcFlash(Samba& samba,
					 const char *_ecv_array name,
                     uint32_t addr,
                     uint32_t pages,
                     uint32_t size,
                     uint32_t planes,
                     uint32_t numLockRegions,
                     uint32_t user,
                     uint32_t stack,
                     uint32_t regs,
                     bool canBrownout) THROWS(GCodeException)
    : BossaFlash(samba, name, addr, pages, size, planes, numLockRegions, user, stack),
      _regs(regs), _canBrownout(canBrownout), _eraseAuto(true)
{

    // SAM3 Errata (FWS must be 6)
    _samba.writeWord(EEFC0_FMR, 0x6 << 8);
    if (planes == 2)
    {
        _samba.writeWord(EEFC1_FMR, 0x6 << 8);
    }
}

EefcFlash::~EefcFlash()
{
}

void
EefcFlash::eraseAll(uint32_t offset) THROWS(GCodeException)
{
    // Do a full chip erase if the offset is 0
    if (offset == 0)
    {
        waitFSR();
        writeFCR0(EEFC_FCMD_EA, 0);
        if (_planes == 2)
        {
            waitFSR();
            writeFCR1(EEFC_FCMD_EA, 0);
        }

        // Erase all can take an exceptionally long time on some devices so wait on FSR for up to 30 seconds
        waitFSR(30);
    }
    // Else we must do it by pages
    else
    {
        // Offset must be on an erase page boundary
        if (offset % (_size * PagesPerErase) != 0)
		{
            throw FlashEraseError("Flash erase error");
		}

        // Erase each PagesPerErase set of pages
        for (uint32_t pageNum = offset / _size; pageNum < _pages; pageNum += PagesPerErase)
        {
            if (_planes == 1 || pageNum < _pages / 2)
            {
                waitFSR();
                writeFCR0(EEFC_FCMD_EPA, pageNum | 0x1);
            }
            else
            {
                waitFSR();
                writeFCR1(EEFC_FCMD_EPA, (pageNum % (_pages / 2)) | 0x1);
            }
        }
    }
}

void EefcFlash::eraseAuto(bool enable) noexcept
{
    _eraseAuto = enable;
}

Bitmap<uint32_t> EefcFlash::getLockRegions() THROWS(GCodeException)
{
	Bitmap<uint32_t> regions;

    waitFSR();
    for (uint32_t region = 0; region < _numLockRegions; region++)
    {
        if (_planes == 2 && region >= _numLockRegions / 2)
        {
        	uint32_t bit = region - _numLockRegions / 2;
            writeFCR1(EEFC_FCMD_GLB, 0);
            waitFSR();
            uint32_t frr = readFRR1();
            while (bit >= 32)
            {
                frr = readFRR1();
                bit -= 32;
            }
            if ((frr & (1 << bit)) != 0)
            {
            	regions.SetBit(region);
            }
        }
        else
        {
        	uint32_t bit = region;
            writeFCR0(EEFC_FCMD_GLB, 0);
            waitFSR();
            uint32_t frr = readFRR0();
            while (bit >= 32)
            {
                frr = readFRR0();
                bit -= 32;
            }
            if ((frr & (1 << bit)) != 0)
            {
            	regions.SetBit(region);
            }
        }
    }

    return regions;
}

#if ORIGINAL_BOSSA_CODE
bool
EefcFlash::getSecurity()
{
    waitFSR();
    writeFCR0(EEFC_FCMD_GGPB, 0);
    waitFSR();
    return (readFRR0() & (1 << 0));
}

bool
EefcFlash::getBod()
{
    if (!_canBrownout)
        return false;

    waitFSR();
    writeFCR0(EEFC_FCMD_GGPB, 0);
    waitFSR();
    return (readFRR0() & (1 << 1));
}

bool
EefcFlash::getBor()
{
    if (!_canBrownout)
        return false;

    waitFSR();
    writeFCR0(EEFC_FCMD_GGPB, 0);
    waitFSR();
    return (readFRR0() & (1 << 2));
}
#endif

bool EefcFlash::getBootFlash() THROWS(GCodeException)
{
    waitFSR();
    writeFCR0(EEFC_FCMD_GGPB, 0);
    waitFSR();
    return (readFRR0() & (1 << (_canBrownout ? 3 : 1)));
}

void EefcFlash::writeOptions() THROWS(GCodeException)
{
    if (canBootFlash() && _bootFlash.isDirty() && _bootFlash.get() != getBootFlash())
    {
        waitFSR();
        writeFCR0(_bootFlash.get() ? EEFC_FCMD_SGPB : EEFC_FCMD_CGPB, (canBod() ? 3 : 1));
    }
#if ORIGINAL_BOSSA_CODE
    if (canBor() && _bor.isDirty() && _bor.get() != getBor())
    {
        waitFSR();
        writeFCR0(_bor.get() ? EEFC_FCMD_SGPB : EEFC_FCMD_CGPB, 2);
    }
    if (canBod() && _bod.isDirty() && _bod.get() != getBod())
    {
        waitFSR();
        writeFCR0(_bod.get() ? EEFC_FCMD_SGPB : EEFC_FCMD_CGPB, 1);
    }
#endif
    if (_regions.isDirty())
    {
        Bitmap<uint32_t> current = getLockRegions();

        for (uint32_t region = 0; region < _numLockRegions; region++)
        {
            if (_regions.get().IsBitSet(region) != current.IsBitSet(region))
            {
                if (_planes == 2 && region >= _numLockRegions / 2)
                {
                	const uint32_t page = (region - _numLockRegions / 2) * _pages / _numLockRegions;
                    waitFSR();
                    writeFCR1(_regions.get().IsBitSet(region) ? EEFC_FCMD_SLB : EEFC_FCMD_CLB, page);
                }
                else
                {
                	const uint32_t page = region * _pages / _numLockRegions;
                    waitFSR();
                    writeFCR0(_regions.get().IsBitSet(region) ? EEFC_FCMD_SLB : EEFC_FCMD_CLB, page);
                }
            }
        }
    }
#if ORIGINAL_BOSSA_CODE
    if (_security.isDirty() && _security.get() == true && _security.get() != getSecurity())
    {
        waitFSR();
        writeFCR0(EEFC_FCMD_SGPB, 0);
    }
#endif
}

void EefcFlash::writePage(uint32_t page) THROWS(GCodeException)
{
    if (page >= _pages)
	{
        throw FlashPageError("EefcFlash::writePage: FlashPageError");
	}

    _wordCopy.setDstAddr(_addr + page * _size);
    _wordCopy.setSrcAddr(_onBufferA ? _pageBufferA : _pageBufferB);
    _onBufferA = !_onBufferA;
    waitFSR();
    _wordCopy.runv();
    if (_planes == 2 && page >= _pages / 2)
    {
        writeFCR1(_eraseAuto ? EEFC_FCMD_EWP : EEFC_FCMD_WP, page - _pages / 2);
    }
    else
    {
        writeFCR0(_eraseAuto ? EEFC_FCMD_EWP : EEFC_FCMD_WP, page);
    }
}

void EefcFlash::readPage(uint32_t page, uint8_t *_ecv_array data) THROWS(GCodeException)
{
    if (page >= _pages)
	{
        throw FlashPageError("EefcFlash::readPage: FlashPageError");
	}

    // The SAM3 firmware has a bug where it returns all zeros for reads
    // directly from the flash so instead, we copy the flash page to
    // SRAM and read it from there.
    _wordCopy.setDstAddr(_onBufferA ? _pageBufferA : _pageBufferB);
    _wordCopy.setSrcAddr(_addr + page * _size);
    waitFSR();
    _wordCopy.runv();
    _samba.read(_onBufferA ? _pageBufferA : _pageBufferB, data, _size);
}

void EefcFlash::waitFSR(int seconds) THROWS(GCodeException)
{
    int tries = seconds * 1000;
    uint32_t fsr1 = 0x1;

    while (tries-- > 0)
    {
    	uint32_t fsr0 = _samba.readWord(EEFC0_FSR);
        if ((fsr0 & 0x2) != 0)
		{
            throw FlashCmdError("EefcFlash::waitFSR: FlashCmdError 1");
		}
        if ((fsr0 & 0x4) != 0)
		{
            throw FlashLockError("EefcFlash::waitFSR: FlashLockError 1");
		}

        if (_planes == 2)
        {
            fsr1 = _samba.readWord(EEFC1_FSR);
            if ((fsr1 & 0x2) != 0)
			{
                throw FlashCmdError("EefcFlash::waitFSR: FlashCmdError 2");
			}
            if ((fsr1 & 0x4) != 0)
			{
                throw FlashLockError("EefcFlash::waitFSR: FlashLockError 2");
			}
        }
        if ((fsr0 & fsr1 & 0x1) != 0)
            break;
        delay(2);
    }
    if (tries == 0)
	{
        throw FlashTimeoutError("EefcFlash::waitFSR: FlashTimeoutError");
	}
}

void EefcFlash::writeFCR0(uint8_t cmd, uint32_t arg) THROWS(GCodeException)
{
    _samba.writeWord(EEFC0_FCR, (EEFC_KEY << 24) | (arg << 8) | cmd);
}

void EefcFlash::writeFCR1(uint8_t cmd, uint32_t arg) THROWS(GCodeException)
{
    _samba.writeWord(EEFC1_FCR, (EEFC_KEY << 24) | (arg << 8) | cmd);
}

uint32_t EefcFlash::readFRR0() THROWS(GCodeException)
{
    return _samba.readWord(EEFC0_FRR);
}

uint32_t EefcFlash::readFRR1() THROWS(GCodeException)
{
    return _samba.readWord(EEFC1_FRR);
}
