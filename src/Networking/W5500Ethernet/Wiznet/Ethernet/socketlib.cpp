//*****************************************************************************
//
//! \file socket.c
//! \brief SOCKET APIs Implements file.
//! \details SOCKET APIs like as Berkeley Socket APIs. 
//! \version 1.0.3
//! \date 2013/10/21
//! \par  Revision history
//!       <2015/02/05> Notice
//!        The version history is not updated after this point.
//!        Download the latest version directly from GitHub. Please visit the our GitHub repository for ioLibrary.
//!        >> https://github.com/Wiznet/ioLibrary_Driver
//!       <2014/05/01> V1.0.3. Refer to M20140501
//!         1. Implicit type casting -> Explicit type casting.
//!         2. replace 0x01 with PACK_REMAINED in recvfrom()
//!         3. Validation a destination ip in connect() & sendto(): 
//!            It occurs a fatal error on converting unint32 address if uint8* addr parameter is not aligned by 4byte address.
//!            Copy 4 byte addr value into temporary uint32 variable and then compares it.
//!       <2013/12/20> V1.0.2 Refer to M20131220
//!                    Remove Warning.
//!       <2013/11/04> V1.0.1 2nd Release. Refer to "20131104".
//!                    In sendto(), Add to clear timeout interrupt status (Sn_IR_TIMEOUT)
//!       <2013/10/21> 1st Release
//! \author MidnightCow
//! \copyright
//!
//! Copyright (c)  2013, WIZnet Co., LTD.
//! All rights reserved.
//! 
//! Redistribution and use in source and binary forms, with or without 
//! modification, are permitted provided that the following conditions 
//! are met: 
//! 
//!     * Redistributions of source code must retain the above copyright 
//! notice, this list of conditions and the following disclaimer. 
//!     * Redistributions in binary form must reproduce the above copyright
//! notice, this list of conditions and the following disclaimer in the
//! documentation and/or other materials provided with the distribution. 
//!     * Neither the name of the <ORGANIZATION> nor the names of its 
//! contributors may be used to endorse or promote products derived 
//! from this software without specific prior written permission. 
//! 
//! THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//! AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
//! IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//! ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
//! LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
//! CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
//! SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//! INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
//! CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
//! ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
//! THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************
#include "socketlib.h"

//#define _SOCKET_DEBUG_

#ifdef _SOCKET_DEBUG_
extern "C" void debugPrintf(const char *fmt, ...);
extern "C" void delay(uint32_t);
# define DEBUG_PRINTF(...) debugPrintf(__VA_ARGS__)
#else
# define DEBUG_PRINTF(_fmt, ...)
#endif

#define SOCK_ANY_PORT_NUM  0xC000

static uint16_t sock_any_port = SOCK_ANY_PORT_NUM;
static uint16_t sock_io_mode = 0;
static uint16_t sock_is_sending = 0;

static uint16_t sock_remained_size[_WIZCHIP_SOCK_NUM_] = {0,0,};

uint8_t sock_pack_info[_WIZCHIP_SOCK_NUM_] = {0,};

#define CHECK_SOCKMODE(mode)  \
   do{                     \
      if((getSn_MR(sn) & 0x0F) != mode) return SOCKERR_SOCKMODE;  \
   }while(0);              \

#define CHECK_SOCKINIT()   \
   do{                     \
      if((getSn_SR(sn) != SOCK_INIT)) return SOCKERR_SOCKINIT; \
   }while(0);              \

#define CHECK_SOCKDATA()   \
   do{                     \
      if(len == 0) return SOCKERR_DATALEN;   \
   }while(0);              \

bool IsSending(uint8_t sn)
{
	return (sock_is_sending & (1u << sn)) != 0;
}

void ExecCommand(uint8_t sn, uint8_t cmd)
{
	setSn_CR(sn, cmd);
	while(getSn_CR(sn) != 0) { }
}

int8_t socket(uint8_t sn, uint8_t protocol, uint16_t port, uint8_t flag)
{
	switch(protocol)
	{
	case Sn_MR_TCP:
		{
			uint32_t taddr;
			getSIPR((uint8_t*)&taddr);
			if (taddr == 0)
			{
				return SOCKERR_SOCKINIT;
			}
		}
		break;

	case Sn_MR_UDP:
	case Sn_MR_MACRAW:
		break;

	default:
		return SOCKERR_SOCKMODE;
	}

	if ((flag & 0x04) != 0)
	{
		return SOCKERR_SOCKFLAG;
	}

	if (flag != 0)
	{
		switch(protocol)
		{
		case Sn_MR_TCP:
			if ((flag & (SF_TCP_NODELAY|SF_IO_NONBLOCK)) == 0)
			{
				return SOCKERR_SOCKFLAG;
			}
			break;
		case Sn_MR_UDP:
			if (flag & SF_IGMP_VER2)
			{
				if ((flag & SF_MULTI_ENABLE) == 0)
				{
					return SOCKERR_SOCKFLAG;
				}
			}
			if (flag & SF_UNI_BLOCK)
			{
				if ((flag & SF_MULTI_ENABLE) == 0)
				{
					return SOCKERR_SOCKFLAG;
				}
			}
			break;
		default:
			break;
		}
	}
	close(sn);
	setSn_MR(sn, (protocol | (flag & 0xF0)));
	if (port == 0)
	{
		port = sock_any_port++;
		if (sock_any_port == 0xFFF0)
		{
			sock_any_port = SOCK_ANY_PORT_NUM;
		}
	}
	setSn_PORT(sn,port);
	ExecCommand(sn, Sn_CR_OPEN);

	// release the previous sock_io_mode
	sock_io_mode &= ~(1 <<sn);
	sock_io_mode |= ((flag & SF_IO_NONBLOCK) << sn);   
	sock_is_sending &= ~(1<<sn);
	sock_remained_size[sn] = 0;
	sock_pack_info[sn] = PACK_COMPLETED;
	while(getSn_SR(sn) == SOCK_CLOSED) { }
	return (int8_t)sn;
}	   

int8_t close(uint8_t sn)
{
	ExecCommand(sn, Sn_CR_CLOSE);

	// Clear all interrupt of the socket
	setSn_IR(sn, 0xFF);
	// Release the sock_io_mode of socket n.
	sock_io_mode &= ~(1<<sn);
	sock_is_sending &= ~(1<<sn);
	sock_remained_size[sn] = 0;
	sock_pack_info[sn] = 0;
	while(getSn_SR(sn) != SOCK_CLOSED) { }
	return SOCK_OK;
}

int8_t listen(uint8_t sn)
{
	CHECK_SOCKMODE(Sn_MR_TCP);
	CHECK_SOCKINIT();
	ExecCommand(sn, Sn_CR_LISTEN);
	while(getSn_SR(sn) != SOCK_LISTEN)
	{
		 close(sn);
		 return SOCKERR_SOCKCLOSED;
	}
	return SOCK_OK;
}

int8_t connect(uint8_t sn, uint8_t * addr, uint16_t port)
{
	CHECK_SOCKMODE(Sn_MR_TCP);
	CHECK_SOCKINIT();
	{
		uint32_t taddr = ((uint32_t)addr[0] & 0x000000FF);
		taddr = (taddr << 8) + ((uint32_t)addr[1] & 0x000000FF);
		taddr = (taddr << 8) + ((uint32_t)addr[2] & 0x000000FF);
		taddr = (taddr << 8) + ((uint32_t)addr[3] & 0x000000FF);
		if (taddr == 0xFFFFFFFF || taddr == 0)
		{
			return SOCKERR_IPINVALID;
		}
	}
	
	if (port == 0)
	{
		return SOCKERR_PORTZERO;
	}
	setSn_DIPR(sn,addr);
	setSn_DPORT(sn,port);
	ExecCommand(sn, Sn_CR_CONNECT);
	if (sock_io_mode & (1<<sn))
	{
		return SOCK_BUSY;
	}
	while(getSn_SR(sn) != SOCK_ESTABLISHED)
	{
		if (getSn_IR(sn) & Sn_IR_TIMEOUT)
		{
			setSn_IR(sn, Sn_IR_TIMEOUT);
			return SOCKERR_TIMEOUT;
		}

		if (getSn_SR(sn) == SOCK_CLOSED)
		{
			return SOCKERR_SOCKCLOSED;
		}
	}

	return SOCK_OK;
}

void disconnectNoWait(uint8_t sn)
{
	ExecCommand(sn, Sn_CR_DISCON);
}

int32_t sendto(uint8_t sn, const uint8_t * buf, uint16_t len, const uint8_t * addr, uint16_t port)
{
	switch(getSn_MR(sn) & 0x0F)
	{
	case Sn_MR_UDP:
	case Sn_MR_MACRAW:
		break;
	default:
		return SOCKERR_SOCKMODE;
	}

	CHECK_SOCKDATA();
	uint32_t taddr = ((uint32_t)addr[0]) & 0x000000FF;
	taddr = (taddr << 8) + ((uint32_t)addr[1] & 0x000000FF);
	taddr = (taddr << 8) + ((uint32_t)addr[2] & 0x000000FF);
	taddr = (taddr << 8) + ((uint32_t)addr[3] & 0x000000FF);
	if (taddr == 0)
	{
		return SOCKERR_IPINVALID;
	}
	if (port == 0)
	{
		return SOCKERR_PORTZERO;
	}
	const uint8_t tmp = getSn_SR(sn);
	if (tmp != SOCK_MACRAW && tmp != SOCK_UDP)
	{
		return SOCKERR_SOCKSTATUS;
	}

	setSn_DIPR(sn,addr);
	setSn_DPORT(sn,port);
	uint16_t freesize = getSn_TxMAX(sn);
	if (len > freesize)
	{
		len = freesize; // check size not to exceed MAX size.
	}

	freesize = getSn_TX_FSR(sn);
	if (getSn_SR(sn) == SOCK_CLOSED)
	{
		return SOCKERR_SOCKCLOSED;
	}
	if (len > freesize)
	{
		return SOCK_BUSY;
	}

	wiz_send_data(sn, buf, len);
	ExecCommand(sn, Sn_CR_SEND);
	sock_is_sending |= 1u << sn;
	return (int32_t)len;
}

int32_t CheckSendComplete(uint8_t sn)
{
	const uint8_t tmp = getSn_IR(sn);
	if (tmp & Sn_IR_SENDOK)
	{
		setSn_IR(sn, Sn_IR_SENDOK);
		sock_is_sending &= ~(1u << sn);
		return SOCK_OK;
	}
	else if(tmp & Sn_IR_TIMEOUT)
	{
		setSn_IR(sn, Sn_IR_TIMEOUT);
		sock_is_sending &= ~(1u << sn);
		return SOCKERR_TIMEOUT;
	}
	DEBUG_PRINTF("Socket %u waiting for send to complete, IR=%02x\n", sn, tmp);
	return SOCK_BUSY;
}

int32_t recvfrom(uint8_t sn, uint8_t * buf, uint16_t len, uint8_t * addr, uint16_t *port)
{
	const uint8_t mr = getSn_MR(sn);
	switch(mr & 0x0F)
	{
	case Sn_MR_UDP:
	case Sn_MR_MACRAW:
		break;
	default:
		return SOCKERR_SOCKMODE;
	}

	CHECK_SOCKDATA();
	uint16_t pack_len=0;
	if (sock_remained_size[sn] == 0)
	{
		while(1)
		{
			pack_len = getSn_RX_RSR(sn);
			if (getSn_SR(sn) == SOCK_CLOSED)
			{
				return SOCKERR_SOCKCLOSED;
			}
			if ( (sock_io_mode & (1<<sn)) && (pack_len == 0) )
			{
				return SOCK_BUSY;
			}
			if (pack_len != 0)
			{
				break;
			}
		}
	}

	switch (mr & 0x07)
	{
	case Sn_MR_UDP :
		if (sock_remained_size[sn] == 0)
		{
			uint8_t head[8];
			wiz_recv_data(sn, head, 8);
			ExecCommand(sn, Sn_CR_RECV);

			// Read peer's IP address, port number & packet length
			addr[0] = head[0];
			addr[1] = head[1];
			addr[2] = head[2];
			addr[3] = head[3];
			*port = head[4];
			*port = (*port << 8) + head[5];
			sock_remained_size[sn] = head[6];
			sock_remained_size[sn] = (sock_remained_size[sn] << 8) + head[7];
			sock_pack_info[sn] = PACK_FIRST;
		}
		if (len < sock_remained_size[sn])
		{
			pack_len = len;
		}
		else
		{
			pack_len = sock_remained_size[sn];
		}
		len = pack_len;
		//
		// Need to packet length check (default 1472)
		//
		wiz_recv_data(sn, buf, pack_len); // data copy.
		break;

	case Sn_MR_MACRAW :
		if (sock_remained_size[sn] == 0)
		{
			uint8_t head[8];
			wiz_recv_data(sn, head, 2);
			ExecCommand(sn, Sn_CR_RECV);

			// Read peer's IP address, port number & packet length
			sock_remained_size[sn] = head[0];
			sock_remained_size[sn] = (sock_remained_size[sn] <<8) + head[1];
			if (sock_remained_size[sn] > 1514)
			{
				close(sn);
				return SOCKFATAL_PACKLEN;
			}
			sock_pack_info[sn] = PACK_FIRST;
		}
		if (len < sock_remained_size[sn])
		{
			pack_len = len;
		}
		else
		{
			pack_len = sock_remained_size[sn];
		}
		wiz_recv_data(sn,buf,pack_len);
		break;

	default:
		wiz_recv_ignore(sn, pack_len); // data copy.
		sock_remained_size[sn] = pack_len;
		break;
	}
	ExecCommand(sn, Sn_CR_RECV);

	sock_remained_size[sn] -= pack_len;
	if (sock_remained_size[sn] != 0)
	{
		sock_pack_info[sn] |= PACK_REMAINED;
	}
	else
	{
		sock_pack_info[sn] = PACK_COMPLETED;
	}
	return (int32_t)pack_len;
}


int8_t  ctlsocket(uint8_t sn, ctlsock_type cstype, void* arg)
{
	switch(cstype)
	{
	case CS_SET_IOMODE:
		{
			const uint8_t tmp = *((uint8_t*)arg);
			if (tmp == SOCK_IO_NONBLOCK)
			{
				sock_io_mode |= (1<<sn);
			}
			else if (tmp == SOCK_IO_BLOCK)
			{
				sock_io_mode &= ~(1<<sn);
			}
			else
			{
				return SOCKERR_ARG;
			}
		}
		break;
	case CS_GET_IOMODE:
		*((uint8_t*)arg) = (uint8_t)((sock_io_mode >> sn) & 0x0001);
		break;
	case CS_GET_MAXTXBUF:
		*((uint16_t*)arg) = getSn_TxMAX(sn);
		break;
	case CS_GET_MAXRXBUF:
		*((uint16_t*)arg) = getSn_RxMAX(sn);
		break;
	case CS_CLR_INTERRUPT:
		if ((*(uint8_t*)arg) > SIK_ALL)
		{
			return SOCKERR_ARG;
		}
		setSn_IR(sn, *(uint8_t*)arg);
		break;
	case CS_GET_INTERRUPT:
		*((uint8_t*)arg) = getSn_IR(sn);
		break;
	case CS_SET_INTMASK:
		if ((*(uint8_t*)arg) > SIK_ALL)
		{
			return SOCKERR_ARG;
		}
		setSn_IMR(sn, *(uint8_t*)arg);
		break;
	case CS_GET_INTMASK:
		*((uint8_t*)arg) = getSn_IMR(sn);
		break;
	default:
		return SOCKERR_ARG;
	}
	return SOCK_OK;
}

int8_t  setsockopt(uint8_t sn, sockopt_type sotype, void* arg)
{
	switch(sotype)
	{
	case SO_TTL:
		setSn_TTL(sn,*(uint8_t*)arg);
		break;
	case SO_TOS:
		setSn_TOS(sn,*(uint8_t*)arg);
		break;
	case SO_MSS:
		setSn_MSSR(sn,*(uint16_t*)arg);
		break;
	case SO_DESTIP:
		setSn_DIPR(sn, (uint8_t*)arg);
		break;
	case SO_DESTPORT:
		setSn_DPORT(sn, *(uint16_t*)arg);
		break;
	case SO_KEEPALIVESEND:
		CHECK_SOCKMODE(Sn_MR_TCP);
		if (getSn_KPALVTR(sn) != 0)
		{
			return SOCKERR_SOCKOPT;
		}
		setSn_CR(sn, Sn_CR_SEND_KEEP);
		while(getSn_CR(sn) != 0)
		{
			if (getSn_IR(sn) & Sn_IR_TIMEOUT)
			{
				setSn_IR(sn, Sn_IR_TIMEOUT);
				return SOCKERR_TIMEOUT;
			}
		}
		break;
	case SO_KEEPALIVEAUTO:
		CHECK_SOCKMODE(Sn_MR_TCP);
		setSn_KPALVTR(sn,*(uint8_t*)arg);
		break;
	default:
		return SOCKERR_ARG;
	}
	return SOCK_OK;
}

int8_t getsockopt(uint8_t sn, sockopt_type sotype, void* arg)
{
   switch(sotype)
   {
      case SO_FLAG:
         *(uint8_t*)arg = getSn_MR(sn) & 0xF0;
         break;
      case SO_TTL:
         *(uint8_t*) arg = getSn_TTL(sn);
         break;
      case SO_TOS:
         *(uint8_t*) arg = getSn_TOS(sn);
         break;
      case SO_MSS:   
         *(uint8_t*) arg = getSn_MSSR(sn);
         break;
      case SO_DESTIP:
         getSn_DIPR(sn, (uint8_t*)arg);
         break;
      case SO_DESTPORT:  
         *(uint16_t*) arg = getSn_DPORT(sn);
         break;
      case SO_KEEPALIVEAUTO:
         CHECK_SOCKMODE(Sn_MR_TCP);
         *(uint16_t*) arg = getSn_KPALVTR(sn);
         break;
      case SO_SENDBUF:
         *(uint16_t*) arg = getSn_TX_FSR(sn);
         break;
      case SO_RECVBUF:
         *(uint16_t*) arg = getSn_RX_RSR(sn);
         break;
      case SO_STATUS:
         *(uint8_t*) arg = getSn_SR(sn);
         break;
      case SO_REMAINSIZE:
         if (getSn_MR(sn) == Sn_MR_TCP)
         {
            *(uint16_t*)arg = getSn_RX_RSR(sn);
         }
         else
         {
            *(uint16_t*)arg = sock_remained_size[sn];
         }
         break;
      case SO_PACKINFO:
         CHECK_SOCKMODE(Sn_MR_TCP);
         *(uint8_t*)arg = sock_pack_info[sn];
         break;
      default:
         return SOCKERR_SOCKOPT;
   }
   return SOCK_OK;
}

// End
