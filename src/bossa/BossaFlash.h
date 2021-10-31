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
#ifndef _FLASH_H
#define _FLASH_H

#define ORIGINAL_BOSSA_CODE (0)
#include "General/Vector.hpp"

#include "Samba.h"
#include "WordCopyApplet.h"

#include "GCodes/GCodeException.h"

typedef GCodeException FlashPageError;
typedef GCodeException FlashRegionError;
typedef GCodeException FlashLockError;
typedef GCodeException FlashCmdError;
typedef GCodeException FlashTimeoutError;
typedef GCodeException BootFlashError;
typedef GCodeException FlashEraseError;

template<class T>
class FlashOption
{
public:
    FlashOption() noexcept : _dirty(false) {}
    virtual ~FlashOption() {}
    void set(const T& value) noexcept { _value = value; _dirty = true; }
    const T& get() noexcept { return _value; }
    bool isDirty() noexcept { return _dirty; }

private:
    T    _value;
    bool _dirty;
};

class BossaFlash	// manuel: required to rename to avoid clash with CoreN2G namespace Flash
{
public:
	BossaFlash(Samba& samba,
          const char *_ecv_array p_name,
          uint32_t addr,                 // Flash base address
          uint32_t pages,                // Number of pages
          uint32_t size,                 // Page size in bytes
          uint32_t planes,               // Number of flash planes
          uint32_t p_lockRegions,        // Number of flash lock regions
          uint32_t user,                 // Address in SRAM where the applet and buffers will be placed
          uint32_t stack) THROWS(GCodeException);               // Address in SRAM where the applet stack will be placed
    virtual ~BossaFlash() {}

    const char *_ecv_array name() noexcept { return _name; }

    virtual uint32_t address() noexcept { return _addr; }
    virtual uint32_t pageSize() noexcept { return _size; }
    virtual uint32_t numPages() noexcept { return _pages; }
    virtual uint32_t numPlanes() noexcept { return _planes; }
    virtual uint32_t totalSize() noexcept { return _size * _pages; }
    virtual uint32_t lockRegions() noexcept { return _lockRegions; }

    virtual void eraseAll(uint32_t offset) THROWS(GCodeException) = 0;
    virtual void eraseAuto(bool enable) noexcept = 0;

    virtual Vector<bool, 16> getLockRegions() THROWS(GCodeException) = 0;
    virtual void setLockRegions(const Vector<bool, 16>& regions) THROWS(GCodeException);

#if ORIGINAL_BOSSA_CODE
    virtual bool getSecurity() = 0;
#endif
    virtual void setSecurity() noexcept;

#if ORIGINAL_BOSSA_CODE
    virtual bool getBod() = 0;
#endif
    virtual void setBod(bool enable) noexcept;
    virtual bool canBod() noexcept = 0;

#if ORIGINAL_BOSSA_CODE
    virtual bool getBor() = 0;
#endif
    virtual void setBor(bool enable) noexcept;
    virtual bool canBor() noexcept = 0;

    virtual bool getBootFlash() THROWS(GCodeException) = 0;
    virtual void setBootFlash(bool enable) noexcept;
    virtual bool canBootFlash() noexcept = 0;

    virtual void writeOptions() THROWS(GCodeException) = 0;

    virtual void writePage(uint32_t page) THROWS(GCodeException) = 0;
    virtual void readPage(uint32_t page, uint8_t* data) THROWS(GCodeException) = 0;

    virtual void loadBuffer(const uint8_t* data, uint16_t size) THROWS(GCodeException);

protected:
    Samba& _samba;
    const char *_ecv_array _name;
    uint32_t _addr;
    uint32_t _pages;
    uint32_t _size;
    uint32_t _planes;
    uint32_t _lockRegions;
    uint32_t _user;
    WordCopyApplet _wordCopy;

    FlashOption<bool> _bootFlash;
    FlashOption<Vector<bool, 16>> _regions;
    FlashOption<bool> _bod;
    FlashOption<bool> _bor;
    FlashOption<bool> _security;

    bool _onBufferA;
    uint32_t _pageBufferA;
    uint32_t _pageBufferB;
};

#endif // _FLASH_H
