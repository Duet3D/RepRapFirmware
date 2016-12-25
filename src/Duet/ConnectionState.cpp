/*
 * ConnectionState.cpp
 *
 *  Created on: 25 Dec 2016
 *      Author: David
 */

#include "ConnectionState.h"

extern "C" {
	#include "lwip/src/include/lwip/tcp.h"
}

//***************************************************************************************************
// ConnectionState class

void ConnectionState::Init(tcp_pcb *p)
{
	pcb = p;
	localPort = p->local_port;
	remoteIPAddress = p->remote_ip.addr;
	remotePort = p->remote_port;
	next = nullptr;
	sendingTransaction = nullptr;
	persistConnection = true;
	isTerminated = false;
}

void ConnectionState::Terminate()
{
	if (pcb != nullptr)
	{
		tcp_abort(pcb);
	}
}

// End
