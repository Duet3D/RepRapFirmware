/*
 * IP4String.h
 *
 *  Created on: 19 Sep 2017
 *      Author: David
 */

#ifndef SRC_LIBRARIES_GENERAL_IP4STRING_H_
#define SRC_LIBRARIES_GENERAL_IP4STRING_H_

#include <cstdint>

// Class to convert an IPv4 address to a string representation
class IP4String
{
public:
	IP4String(const uint8_t ip[4]);
	IP4String(uint32_t ip);
	const char *c_str() const { return buf; }

private:
	char buf[16];		// long enough for e.g. "255.255.255.255" including a null terminator
};

#endif /* SRC_LIBRARIES_GENERAL_IP4STRING_H_ */
