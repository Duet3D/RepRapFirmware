/*
 * UniqueId.h
 *
 *  Created on: 4 Oct 2021
 *      Author: David
 *
 *  This class extends UniqueIdBase to add additional functions
 */

#ifndef SRC_PLATFORM_UNIQUEID_H_
#define SRC_PLATFORM_UNIQUEID_H_

#include <UniqueIdBase.h>
#include <Platform/OutputMemory.h>
#include <Networking/NetworkDefs.h>

// Unique ID class extended with some additional functions
class UniqueId : public UniqueIdBase
{
public:
	void AppendCharsToBuffer(OutputBuffer *buf) const noexcept;
	void GenerateMacAddress(MacAddress& addr) const noexcept;
};

#endif /* SRC_PLATFORM_UNIQUEID_H_ */
