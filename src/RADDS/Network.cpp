#include "Network.h"
#include "OutputMemory.h"

static const uint8_t dummy_ipv4[4] = { 0, 0, 0, 0 };

const char *notSupportedText = "Networking is not supported on this hardware";

const uint8_t *Network::GetIPAddress() const
{
	return dummy_ipv4;
}

void Network::ReportProtocols(StringRef& reply) const
{
	reply.copy(notSupportedText);
}

void Network::Enable(int mode, StringRef& reply)
{
	reply.copy(notSupportedText);
}

bool Network::GetNetworkState(StringRef& reply)
{
	reply.copy(notSupportedText);
	return false;
}

void Network::HandleHttpGCodeReply(OutputBuffer *buf)
{
	OutputBuffer::ReleaseAll(buf);
}

void Network::HandleTelnetGCodeReply(OutputBuffer *buf)
{
	OutputBuffer::ReleaseAll(buf);
}

// End
