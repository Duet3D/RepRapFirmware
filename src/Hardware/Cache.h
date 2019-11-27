/*
 * Cache.h
 *
 *  Created on: 22 Nov 2019
 *      Author: David
 */

#ifndef SRC_HARDWARE_CACHE_H_
#define SRC_HARDWARE_CACHE_H_

#include <RepRapFirmware.h>

namespace Cache
{
	void Init();
	void Enable();
	void Disable();
	void Flush(const volatile void *start, size_t length);
	void Invalidate(const volatile void *start, size_t length);

	inline void FlushBeforeDMAReceive(const volatile void *start, size_t length) { Flush(start, length); }
	inline void InvalidateAfterDMAReceive(const volatile void *start, size_t length) { Invalidate(start, length); }
	inline void FlushBeforeDMASend(const volatile void *start, size_t length) { Flush(start, length); }

#if SAM4E
	uint32_t GetHitCount();
#endif
};

// Entry points that can be called from ASF C code
extern "C" void CacheFlushBeforeDMAReceive(const volatile void *start, size_t length);
extern "C" void CacheInvalidateAfterDMAReceive(const volatile void *start, size_t length);
extern "C" void CacheFlushBeforeDMASend(const volatile void *start, size_t length);

#if USE_CACHE

# if SAM4E

// The SAM4E cache is write-through, so no need to flush it
inline void Cache::Flush(const volatile void *start, size_t length) { }

# endif

#else

inline void Cache::Init() {}
inline void Cache::Enable() {}
inline void Cache::Disable() {}
inline void Cache::Flush(const volatile void *start, size_t length) {}
inline void Cache::Invalidate(const volatile void *start, size_t length) {}

#endif

#endif /* SRC_HARDWARE_CACHE_H_ */
