#include "Network.h"

static const uint8_t dummy_ipv4[4] = { 0, 0, 0, 0 };

const uint8_t *Network::GetIPAddress() const
{
	return dummy_ipv4;
}

void Network::ReportProtocols(StringRef& reply) const
{
	reply.copy("Networking is not supported on this hardware");
}

// End
