#ifndef WEBSERVER_H
#define WEBSERVER_H

#include "RepRapFirmware.h"
#include "MessageType.h"

// List of protocols that can execute G-Codes
enum class WebSource
{
	HTTP,
	Telnet
};

class Webserver
{   
public:
	Webserver(Platform* p, Network *n) { };
	void Init() const { };
	void Spin() const { };
	void Exit() const { };
	void Diagnostics(MessageType mtype) const { };

	uint32_t GetReplySeq() const { return (uint32_t)0; }

	void HandleGCodeReply(const WebSource source, OutputBuffer *reply) const;
	void HandleGCodeReply(const WebSource source, const char *reply) const { };
};

inline void Webserver::HandleGCodeReply(const WebSource source, OutputBuffer *reply) const
{
	if (reply != (OutputBuffer *)0)
	{
		OutputBuffer::ReleaseAll(reply);
	}
}

#endif
