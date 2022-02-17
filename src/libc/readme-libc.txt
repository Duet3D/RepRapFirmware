README file for libc folder
===========================

On ARM Cortex M7 processors, accesses to strongly-ordered memory such as our "nocache" RAM segment must be aligned. Therefore we compile with option -fno-unaligned-access.
Unfortunately, newlib (the standard C library) isn't compiled with -fno-unaligned-access, and memcpy in particular sometimes does unaligned accesses.
To fix this, we include our own copy of memcpy here. Similarly for other memory-related functions that might be used to access DMA buffers.

strptime.cpp in this folder is a cut-down version that doesn't need a locale, saving about 360 bytes of RAM. This is sufficient for RepRapFirmware because we don't
need to recognise month or day names.

DC 2020-01-10
