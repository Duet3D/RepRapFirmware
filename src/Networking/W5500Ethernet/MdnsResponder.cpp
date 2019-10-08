/*
 * mDNSResponder.cpp
 *
 *  Created on: 18 Sep 2019
 *      Author: Christian
 *
 *  This class is roughly based on the EthernetBonjour class for Arduino
 *  by Georg Kaindl (c) 2010 (licensed under the terms of the GPLv3)
 *
 */

#include "RepRap.h"
#include "RepRapFirmware.h"
#include "Platform.h"

#include "MdnsResponder.h"
#include "W5500Interface.h"
#include "W5500Socket.h"

constexpr size_t UdpHeaderLength = 8;
constexpr size_t DnsHeaderLength = 6 * sizeof(uint16_t);
constexpr size_t MinMdnsHeaderLength = DnsHeaderLength + 8;	// + extra space for at least one query

constexpr uint16_t MdnsTtl = 120;		// in seconds

MdnsResponder::MdnsResponder(W5500Socket *sock) : socket(sock), lastAnnouncement(0)
{
}

void MdnsResponder::UpdateServiceRecords()
{
	// TODO
}

void MdnsResponder::Spin()
{
	// Announce this host in regular intervals...
	if (millis() - lastAnnouncement > MdnsTtl * 1000)
	{
		Announce();
	}

	// Have we got a new query?
	const uint8_t *packetData;
	size_t bytesRead;
	if (socket->CanRead() && socket->ReadBuffer(packetData, bytesRead))
	{
		if (bytesRead >= UdpHeaderLength + MinMdnsHeaderLength)
		{
			// Process mDNS packet
			ProcessPacket(packetData + UdpHeaderLength, bytesRead - UdpHeaderLength);
		}
		socket->Taken(bytesRead);
	}
}

void MdnsResponder::ProcessPacket(const uint8_t *packet, size_t length) const
{
	size_t bytesProcessed = 0;

	// Try to read the DNS header
	const uint16_t transaction = __builtin_bswap16(*reinterpret_cast<const uint16_t*>(packet + bytesProcessed));
	bytesProcessed += sizeof(uint16_t);
	const uint16_t headerFlags = __builtin_bswap16(*reinterpret_cast<const uint16_t*>(packet + bytesProcessed));
	bytesProcessed += sizeof(uint16_t);
	const uint16_t numQueries = __builtin_bswap16(*reinterpret_cast<const uint16_t*>(packet + bytesProcessed));
	bytesProcessed += DnsHeaderLength - 2 * sizeof(uint16_t);

	if (headerFlags == 0)		// Query
	{
		for (uint16_t query = 0; query < numQueries; query++)
		{
			bool nameMatches = false;
			if (packet[bytesProcessed] == 0xC0)
			{
				// Deal with name compression
				bytesProcessed++;
				uint8_t offset = packet[bytesProcessed++];
				if (offset < length)
				{
					// Check if the compressed name matches
					nameMatches = CheckHostname(packet + offset, length - offset, nullptr);
				}
				else
				{
					//debugPrintf("mDNS compressed name overflow\n");
					break;
				}
			}
			else
			{
				// Deal with regular names
				if (bytesProcessed < length)
				{
					// Check if the regular name matches
					nameMatches = CheckHostname(packet + bytesProcessed, length - bytesProcessed, &bytesProcessed);
				}
				else
				{
					//debugPrintf("mDNS regular name overflow\n");
					break;
				}
			}

			// Check query type and class
			if (bytesProcessed + 2 * sizeof(uint16_t) <= length)
			{
				const uint16_t type = __builtin_bswap16(*reinterpret_cast<const uint16_t*>(packet + bytesProcessed));
				bytesProcessed += sizeof(uint16_t);
				const uint16_t flags = __builtin_bswap16(*reinterpret_cast<const uint16_t*>(packet + bytesProcessed));
				bytesProcessed += sizeof(uint16_t);
				if (flags == 1 && type == 1 && nameMatches)		// Class IN, A record
				{
					SendARecord(transaction);
				}
			}
			else
			{
				//debugPrintf("mDNS query flags overflow\n");
				break;
			}
		}
	}
	else
	{
		//debugPrintf("Unknown mDNS query\n");
	}
}

bool MdnsResponder::CheckHostname(const uint8_t *ptr, size_t maxLength, size_t *bytesProcessed) const
{
	const uint8_t *originalPtr = ptr;
	const char *hostname = reprap.GetNetwork().GetHostname();
	bool nameMatches = true;

	// Check the hostname
	const uint8_t nameLength = *ptr++;
	if (nameLength < maxLength)
	{
		if (nameLength == strlen(hostname))
		{
			for (unsigned int i = 0; i < nameLength; i++)
			{
				if (tolower(hostname[i]) != tolower(*ptr++))
				{
					nameMatches = false;
				}
			}
		}
		else
		{
			nameMatches = false;
			ptr += nameLength;
		}
	}
	else
	{
		//debugPrintf("mDNS name overflow\n");
		if (bytesProcessed != nullptr)
		{
			bytesProcessed += maxLength;
		}
		return false;
	}

	// Check the domain name
	const uint8_t domainLength = *ptr++;
	if (ptr + domainLength < originalPtr + maxLength)
	{
		if (nameMatches && domainLength == 5)
		{
			const char *domainName = reinterpret_cast<const char *>(ptr);
			nameMatches = strncmp(domainName, "local", 5) == 0;
		}
		else
		{
			nameMatches = false;
		}
		ptr += domainLength;
	}
	else
	{
		//debugPrintf("mDNS query domain overflow\n");
		if (bytesProcessed != nullptr)
		{
			bytesProcessed += maxLength;
		}
		return false;
	}

	// Check for terminating zero
	if (ptr < originalPtr + maxLength)
	{
		if (*ptr++ != 0)
		{
			//debugPrintf("mDNS FQDN not terminated\n");
			if (bytesProcessed != nullptr)
			{
				bytesProcessed += maxLength;
			}
			return false;
		}
	}
	else
	{
		//debugPrintf("mDNS FQDN overflow\n");
		if (bytesProcessed != nullptr)
		{
			bytesProcessed += maxLength;
		}
		return false;
	}

	// End
	if (bytesProcessed != nullptr)
	{
		*bytesProcessed += ptr - originalPtr;
	}
	return nameMatches;
}

void MdnsResponder::SendARecord(uint16_t transaction) const
{
	uint8_t buffer[256];
	size_t bytesWritten = 0;

	memset(buffer, 0, ARRAY_SIZE(buffer));

	// Write DNS header
	*reinterpret_cast<uint16_t*>(buffer) = transaction;
	bytesWritten += sizeof(uint16_t);
	*reinterpret_cast<uint16_t*>(buffer + bytesWritten) = __builtin_bswap16(0x8400);	// Standard response
	bytesWritten += sizeof(uint16_t);
	*reinterpret_cast<uint16_t*>(buffer + bytesWritten) = 0;							// No questions
	bytesWritten += sizeof(uint16_t);
	*reinterpret_cast<uint16_t*>(buffer + bytesWritten) = __builtin_bswap16(1);			// One answer
	bytesWritten += sizeof(uint16_t);
	*reinterpret_cast<uint16_t*>(buffer + bytesWritten) = 0;							// No authority RRs
	bytesWritten += sizeof(uint16_t);
	*reinterpret_cast<uint16_t*>(buffer + bytesWritten) = 0;							// No additional RRs
	bytesWritten += sizeof(uint16_t);

	// Write A record and start with the hostname
	const char *hostname = reprap.GetNetwork().GetHostname();
	size_t hostnameLength = strlen(hostname);
	buffer[bytesWritten++] = hostnameLength;
	strncpy(reinterpret_cast<char *>(buffer + bytesWritten), hostname, ARRAY_SIZE(buffer) - bytesWritten);
	bytesWritten += hostnameLength;

	// Write domain
	buffer[bytesWritten++] = 5;
	strncpy(reinterpret_cast<char *>(buffer + bytesWritten), "local", ARRAY_SIZE(buffer) - bytesWritten);
	bytesWritten += 5;

	// Write terminating zero
	buffer[bytesWritten++] = 0;

	// Write flags
	*reinterpret_cast<uint16_t*>(buffer + bytesWritten) = __builtin_bswap16(0x01);		// A record
	bytesWritten += sizeof(uint16_t);
	*reinterpret_cast<uint16_t*>(buffer + bytesWritten) = __builtin_bswap16(0x8001);	// Class IN, Cache Flush
	bytesWritten += sizeof(uint16_t);

	// Write TTL
	*reinterpret_cast<uint32_t*>(buffer + bytesWritten) = __builtin_bswap32(MdnsTtl);
	bytesWritten += sizeof(uint32_t);

	// Write data length
	*reinterpret_cast<uint16_t*>(buffer + bytesWritten) = __builtin_bswap16(4);			// 4 Bytes (IPv4)
	bytesWritten += sizeof(uint16_t);

	// Write IPv4 address
	socket->GetInterface()->GetIPAddress().UnpackV4(buffer + bytesWritten);
	bytesWritten += 4;

	// Send it to the mDNS address
	socket->Send(buffer, bytesWritten);
	socket->Send();
}

void MdnsResponder::Announce()
{
	if (!socket->CanSend())
	{
		return;
	}

	SendARecord(0);
	lastAnnouncement = millis();
}
