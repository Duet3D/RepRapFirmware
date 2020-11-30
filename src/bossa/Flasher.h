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
#ifndef _FLASHER_H
#define _FLASHER_H

#include "Device.h"
#include "Flash.h"
#include "Samba.h"

#include "GCodes/GCodeException.h"

typedef GCodeException FlashOffsetError;
typedef GCodeException FileSizeError;

class FlasherObserver
{
public:
    FlasherObserver() {}
    virtual ~FlasherObserver() {}

    virtual void onStatus(const char *message, ...) = 0;
    virtual void onProgress(int num, int div) = 0;
};

class Flasher
{
public:
    Flasher(Samba& samba, Device& device, FlasherObserver& observer)
    	: _samba(samba), _flash(device.getFlash()), _observer(observer), pageNum(0), infile(nullptr), fileSize(0)
    {}
    virtual ~Flasher() {}

    void erase(uint32_t foffset);
    int GetNextChunk(char* buffer, const uint32_t amount, uint32_t& offset, const char* filename) noexcept;
    bool write(const char* filename, uint32_t& foffset);
    bool verify(const char* filename, uint32_t& pageErrors, uint32_t& totalErrors, uint32_t& foffset);
    void lock(/* std::string& regionArg, */ bool enable);

private:
    Samba& _samba;
    Flash* _flash;
    FlasherObserver& _observer;

    uint32_t pageNum;
    FileStore * infile;
    FilePosition fileSize;
};

#endif // _FLASHER_H
