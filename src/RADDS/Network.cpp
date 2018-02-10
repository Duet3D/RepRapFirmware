#include "Network.h"
#include "OutputMemory.h"

const char * const notSupportedText = "Networking is not supported on this hardware";

GCodeResult Network::EnableProtocol(unsigned int interface, int protocol, int port, bool secure, const StringRef& reply)
{
	reply.copy(notSupportedText);
	return GCodeResult::error;
}

GCodeResult Network::DisableProtocol(unsigned int interface, int protocol, const StringRef& reply)
{
	reply.copy(notSupportedText);
	return GCodeResult::error;
}

GCodeResult Network::ReportProtocols(unsigned int interface, const StringRef& reply) const
{
	reply.copy(notSupportedText);
	return GCodeResult::error;
}

GCodeResult Network::EnableInterface(unsigned int interface, int mode, const StringRef& ssid, const StringRef& reply)
{
	reply.copy(notSupportedText);
	return GCodeResult::error;
}

GCodeResult Network::GetNetworkState(unsigned int interface, const StringRef& reply)
{
	reply.copy(notSupportedText);
	return GCodeResult::error;
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
