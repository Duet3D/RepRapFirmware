README file for libc folder
===========================

On ARM Cortex M7 processors, accesses to strongly-ordered memory such as our "nocache" RAM segment must be aligned. Therefore we compile with option -fno-unaligned-access.
Unfortunately, newlib (the standard C library) isn't compiled with -fno-unaligned-access, and memcpy in particular sometimes does unaligned accesses.
To fix this, we include our own copy of memcpy here. Similarly for other memory-related functions that might be used to access DMA buffers.

DC 2019-12-24
