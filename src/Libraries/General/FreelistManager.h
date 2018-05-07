/*
 * FreeListManager.h
 *
 *  Created on: 7 May 2018
 *      Author: David
 *
 *  Templated class to manage free lists, a different one for each object size
 */

#ifndef SRC_LIBRARIES_GENERAL_FREELISTMANAGER_H_
#define SRC_LIBRARIES_GENERAL_FREELISTMANAGER_H_

#include <cstddef>

// Free list manager
template<size_t Sz> class FreelistManager
{
public:
	static void *Allocate();
	static void Release(void *p);

private:
	static void *freelist;
};

template<size_t Sz> void *FreelistManager<Sz>::freelist = nullptr;

template<size_t Sz> void *FreelistManager<Sz>::Allocate()
{
	if (freelist != nullptr)
	{
		void * const p = freelist;
		freelist = *static_cast<void **>(p);
		return p;
	}
	return ::operator new(Sz);
}

template<size_t Sz> void FreelistManager<Sz>::Release(void *p)
{
	*static_cast<void **>(p) = freelist;
	freelist = p;
}

// Macro to return the size of objects of a given type rounded up to a multiple of 8 bytes.
// We use this to reduce the number of freelists that we need to keep.
#define ROUNDED_UP_SIZE(_type) ((sizeof(_type) + (8u - 1u)) & ~(8u - 1u))

// Operators new and delete for the classes that we want to use these freelists for should call the following functions
template<class T> inline void *Allocate()
{
	return FreelistManager<ROUNDED_UP_SIZE(T)>::Allocate();
}

template<class T> inline void Release(void *p)
{
	FreelistManager<ROUNDED_UP_SIZE(T)>::Release(p);
}

#endif /* SRC_LIBRARIES_GENERAL_FREELISTMANAGER_H_ */
