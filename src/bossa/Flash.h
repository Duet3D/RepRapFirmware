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
    FlashOption() : _dirty(false) {}
    virtual ~FlashOption() {}
    void set(const T& value) { _value = value; _dirty = true; }
    const T& get() { return _value; }
    bool isDirty() { return _dirty; }

private:
    T    _value;
    bool _dirty;
};

class Flash
{
public:
    Flash(Samba& samba,
          const char* name,
          uint32_t addr,                 // Flash base address
          uint32_t pages,                // Number of pages
          uint32_t size,                 // Page size in bytes
          uint32_t planes,               // Number of flash planes
          uint32_t lockRegions,          // Number of flash lock regions
          uint32_t user,                 // Address in SRAM where the applet and buffers will be placed
          uint32_t stack);               // Address in SRAM where the applet stack will be placed
    virtual ~Flash() {}

    const char* name() { return _name; }

    virtual uint32_t address() { return _addr; }
    virtual uint32_t pageSize() { return _size; }
    virtual uint32_t numPages() { return _pages; }
    virtual uint32_t numPlanes() { return _planes; }
    virtual uint32_t totalSize() { return _size * _pages; }
    virtual uint32_t lockRegions() { return _lockRegions; }

    virtual void eraseAll(uint32_t offset) = 0;
    virtual void eraseAuto(bool enable) = 0;

    virtual Vector<bool, 16> getLockRegions() = 0;
    virtual void setLockRegions(const Vector<bool, 16>& regions);

    virtual bool getSecurity() = 0;
    virtual void setSecurity();

    virtual bool getBod() = 0;
    virtual void setBod(bool enable);
    virtual bool canBod() = 0;

    virtual bool getBor() = 0;
    virtual void setBor(bool enable);
    virtual bool canBor() = 0;

    virtual bool getBootFlash() = 0;
    virtual void setBootFlash(bool enable);
    virtual bool canBootFlash() = 0;

    virtual void writeOptions() = 0;

    virtual void writePage(uint32_t page) = 0;
    virtual void readPage(uint32_t page, uint8_t* data) = 0;

    virtual void writeBuffer(uint32_t dst_addr, uint32_t size);
    virtual void loadBuffer(const uint8_t* data, uint16_t size);

protected:
    Samba& _samba;
    const char* _name;
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
