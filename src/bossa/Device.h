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
#ifndef _DEVICE_H
#define _DEVICE_H

#include "Samba.h"
#include "BossaFlash.h"
#include "GCodes/GCodeException.h"

typedef GCodeException DeviceUnsupportedError;

class Device
{
public:
    enum Family {
        FAMILY_NONE,
#if ORIGINAL_BOSSA_CODE

        FAMILY_SAM7S,
        FAMILY_SAM7SE,
        FAMILY_SAM7X,
        FAMILY_SAM7XC,
        FAMILY_SAM7L,

        FAMILY_SAM3N,
        FAMILY_SAM3S,
        FAMILY_SAM3U,
        FAMILY_SAM3X,
        FAMILY_SAM3A,
#endif
        FAMILY_SAM4S,
#if ORIGINAL_BOSSA_CODE
        FAMILY_SAM4E,

        FAMILY_SAM9XE,

        FAMILY_SAMD21,
        FAMILY_SAMR21,
        FAMILY_SAML21,

        FAMILY_SAMD51,
        FAMILY_SAME51,
        FAMILY_SAME53,
        FAMILY_SAME54,

        FAMILY_SAME70,
        FAMILY_SAMS70,
        FAMILY_SAMV70,
        FAMILY_SAMV71,
#endif
    };

    explicit Device(Samba& samba) noexcept : _samba(samba), _flash(nullptr), _family(FAMILY_NONE) {}
    virtual ~Device() {  delete _flash; }

    void create() THROWS(GCodeException);

    Family getFamily() const noexcept { return _family; }

    BossaFlash *_ecv_from null getFlash() const noexcept { return _flash; }

    void reset() THROWS(GCodeException);

private:
    Samba& _samba;
    BossaFlash *_ecv_from null _flash;
    Family _family;

#if ORIGINAL_BOSSA_CODE
    void readChipId(uint32_t& chipId, uint32_t& extChipId);
#endif
};

#endif // _DEVICE_H

