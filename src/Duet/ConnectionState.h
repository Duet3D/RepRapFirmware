/*
 * ConnectionState.h
 *
 *  Created on: 25 Dec 2016
 *      Author: David
 */

#ifndef SRC_DUET_CONNECTIONSTATE_H_
#define SRC_DUET_CONNECTIONSTATE_H_

#include <NetworkDefs.h>
#include "RepRapFirmware.h"

class tcp_pcb;

// ConnectionState structure that we use to track TCP connections. It is usually combined with NetworkTransactions.
struct ConnectionState
{
	tcp_pcb *volatile pcb;								// Connection PCB
	uint16_t localPort, remotePort;						// Copy of the local and remote ports, because the PCB may be unavailable
	uint32_t remoteIPAddress;							// Same for the remote IP address
	NetworkTransaction * volatile sendingTransaction;	// NetworkTransaction that is currently sending via this connection
	ConnectionState * volatile next;					// Next ConnectionState in this list
	bool persistConnection;								// Do we expect this connection to stay alive?
	volatile bool isTerminated;							// Will be true if the connection has gone down unexpectedly (TCP RST)

	void Init(tcp_pcb *p);
	uint16_t GetLocalPort() const { return localPort; }
	uint32_t GetRemoteIP() const { return remoteIPAddress; }
	uint16_t GetRemotePort() const { return remotePort; }
	bool IsConnected() const { return pcb != nullptr; }
	bool IsTerminated() const { return isTerminated; }
	void Terminate();
};

#endif /* SRC_DUET_CONNECTIONSTATE_H_ */
