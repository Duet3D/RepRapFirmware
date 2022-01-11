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

#include <Config/Configuration.h>
#include "NetworkDefs.h"

class W5500Socket;

class MdnsResponder {
public:
	MdnsResponder(W5500Socket *sock) noexcept;

	void UpdateServiceRecords() noexcept;
	void Spin() noexcept;
	void Announce() noexcept;

private:
	W5500Socket *socket;
	uint32_t lastAnnouncement;

	void ProcessPacket(const uint8_t *data, size_t length) const noexcept;
	bool CheckHostname(const uint8_t *ptr, size_t maxLength, size_t *bytesProcessed) const noexcept;
	void SendARecord(uint16_t transaction) const noexcept;
};

#endif /* SRC_NETWORKING_W5500ETHERNET_MDNSRESPONDER_H_ */
