/*
 * mDNSResponder.h
 *
 *  Created on: 18 Sep 2019
 *      Author: Christian
 */

#ifndef SRC_NETWORKING_W5500ETHERNET_MDNSRESPONDER_H_
#define SRC_NETWORKING_W5500ETHERNET_MDNSRESPONDER_H_

#include <cstddef>
#include <cstdint>

#include <General/IPAddress.h>
#include <General/StringRef.h>

#include "Configuration.h"
#include "NetworkDefs.h"

class W5500Socket;

class MdnsResponder {
public:
	MdnsResponder(W5500Socket *sock);

	void UpdateServiceRecords();
	void Spin();
	void Announce();

private:
	W5500Socket *socket;
	uint32_t lastAnnouncement;

	void ProcessPacket(const uint8_t *data, size_t length) const;
	bool CheckHostname(const uint8_t *ptr, size_t maxLength, size_t *bytesProcessed) const;
	void SendARecord(uint16_t transaction) const;
};

#endif /* SRC_NETWORKING_W5500ETHERNET_MDNSRESPONDER_H_ */
