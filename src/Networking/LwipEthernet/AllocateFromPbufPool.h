/*
 * AllocateFromPbufPool.h
 *
 *  Created on: 2 Jul 2024
 *      Author: David
 */

#ifndef SRC_NETWORKING_LWIPETHERNET_ALLOCATEFROMPBUFPOOL_H_
#define SRC_NETWORKING_LWIPETHERNET_ALLOCATEFROMPBUFPOOL_H_

#include <cstddef>

// The following functions are defined in file LwipEthernetInterface.cpp because that uses the correct include files already

// Flag that allocation from the PBUF pool is allowed and set up the variables
extern void InitAllocationFromPbufPool() noexcept;

// Function to allocate memory from the LWIP PBUF pool. Use this only when Lwip Ethernet will definitely not be used.
// Return a pointer to the allocated memory, or nullptr if there was insufficient memory left.
extern void *AllocateFromPbufPool(size_t bytes) noexcept;

#endif /* SRC_NETWORKING_LWIPETHERNET_ALLOCATEFROMPBUFPOOL_H_ */
