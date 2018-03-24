//*****************************************************************************
//
//! \file w5500.cpp
//! \brief W5500 HAL Interface.
//! \version 1.0.2
//! \date 2013/10/21
//! \par  Revision history
//!       <2015/02/05> Notice
//!        The version history is not updated after this point.
//!        Download the latest version directly from GitHub. Please visit the our GitHub repository for ioLibrary.
//!        >> https://github.com/Wiznet/ioLibrary_Driver
//!       <2014/05/01> V1.0.2
//!         1. Implicit type casting -> Explicit type casting. Refer to M20140501
//!            Fixed the problem on porting into under 32bit MCU
//!            Issued by Mathias ClauBen, wizwiki forum ID Think01 and bobh
//!            Thank for your interesting and serious advices.
//!       <2013/12/20> V1.0.1
//!         1. Remove warning
//!         2. WIZCHIP_READ_BUF WIZCHIP_WRITE_BUF in case _WIZCHIP_IO_MODE_SPI_FDM_
//!            for loop optimized(removed). refer to M20131220
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

#include "Networking/W5500Ethernet/Wiznet/Ethernet/W5500/w5500.h"
#include "Networking/W5500Ethernet/Wiznet/Ethernet/WizSpi.h"

#define _W5500_SPI_VDM_OP_          0x00
#define _W5500_SPI_FDM_OP_LEN1_     0x01
#define _W5500_SPI_FDM_OP_LEN2_     0x02
#define _W5500_SPI_FDM_OP_LEN4_     0x03

////////////////////////////////////////////////////

uint8_t WIZCHIP_READ(uint32_t AddrSel)
{
	WizSpi::AssertSS();
	WizSpi::SendAddress(AddrSel | (_W5500_SPI_READ_ | _W5500_SPI_VDM_OP_));
	const uint8_t ret = WizSpi::ReadByte();
	WizSpi::ReleaseSS();
	return ret;
}

void WIZCHIP_WRITE(uint32_t AddrSel, uint8_t wb )
{
	WizSpi::AssertSS();
	WizSpi::SendAddress(AddrSel | (_W5500_SPI_WRITE_ | _W5500_SPI_VDM_OP_));
	WizSpi::WriteByte(wb);
	WizSpi::ReleaseSS();
}
         
void WIZCHIP_READ_BUF (uint32_t AddrSel, uint8_t* pBuf, uint16_t len)
{
	WizSpi::AssertSS();
	WizSpi::SendAddress(AddrSel | (_W5500_SPI_READ_ | _W5500_SPI_VDM_OP_));
	WizSpi::ReadBurst(pBuf, len);
	WizSpi::ReleaseSS();
}

void WIZCHIP_WRITE_BUF(uint32_t AddrSel, const uint8_t* pBuf, uint16_t len)
{
	WizSpi::AssertSS();
	WizSpi::SendAddress(AddrSel | (_W5500_SPI_WRITE_ | _W5500_SPI_VDM_OP_));
	WizSpi::SendBurst(pBuf, len);
	WizSpi::ReleaseSS();
}


uint16_t getSn_TX_FSR(uint8_t sn)
{
	uint16_t val = 0, val1 = 0;
	do
	{
		val1 = WIZCHIP_READ(Sn_TX_FSR(sn));
		val1 = (val1 << 8) + WIZCHIP_READ(WIZCHIP_OFFSET_INC(Sn_TX_FSR(sn),1));
		if (val1 != 0)
		{
			val = WIZCHIP_READ(Sn_TX_FSR(sn));
			val = (val << 8) + WIZCHIP_READ(WIZCHIP_OFFSET_INC(Sn_TX_FSR(sn),1));
		}
	} while (val != val1);
	return val;
}

uint16_t getSn_RX_RSR(uint8_t sn)
{
	uint16_t val = 0, val1 = 0;
	do
	{
		val1 = WIZCHIP_READ(Sn_RX_RSR(sn));
		val1 = (val1 << 8) + WIZCHIP_READ(WIZCHIP_OFFSET_INC(Sn_RX_RSR(sn),1));
		if (val1 != 0)
		{
			val = WIZCHIP_READ(Sn_RX_RSR(sn));
			val = (val << 8) + WIZCHIP_READ(WIZCHIP_OFFSET_INC(Sn_RX_RSR(sn),1));
		}
	} while (val != val1);
	return val;
}

void wiz_send_data(uint8_t sn, const uint8_t *wizdata, uint16_t len)
{
	if (len != 0)
	{
		uint16_t ptr = getSn_TX_WR(sn);
		const uint32_t addrsel = ((uint32_t)ptr << 8) + (WIZCHIP_TXBUF_BLOCK(sn) << 3);
		WIZCHIP_WRITE_BUF(addrsel, wizdata, len);

		ptr += len;
		setSn_TX_WR(sn, ptr);
	}
}

// Function wiz_send_data doesn't work properly when we send TCP/IP data in multiple chunks,
// because the getSn_TX_WR call doesn't read back the incremented value after we wrote to it.
// So use this instead.
void wiz_send_data_at(uint8_t sn, const uint8_t *wizdata, uint16_t len, uint16_t ptr)
{
	if (len != 0)
	{
		const uint32_t addrsel = ((uint32_t)ptr << 8) + (WIZCHIP_TXBUF_BLOCK(sn) << 3);
		WIZCHIP_WRITE_BUF(addrsel, wizdata, len);
	}
}

void wiz_recv_data(uint8_t sn, uint8_t *wizdata, uint16_t len)
{
	if (len != 0)
	{
		uint16_t ptr = getSn_RX_RD(sn);
		const uint32_t addrsel = ((uint32_t)ptr << 8) + (WIZCHIP_RXBUF_BLOCK(sn) << 3);
		WIZCHIP_READ_BUF(addrsel, wizdata, len);
		ptr += len;
		setSn_RX_RD(sn, ptr);
	}
}

// Function wiz_recv_data only works if we read the entire received data.
// This function should get round that, but the caller will have to track the received buffer pointer.
void wiz_recv_data_at(uint8_t sn, uint8_t *wizdata, uint16_t len, uint16_t ptr)
{
	if (len != 0)
	{
		const uint32_t addrsel = ((uint32_t)ptr << 8) + (WIZCHIP_RXBUF_BLOCK(sn) << 3);
		WIZCHIP_READ_BUF(addrsel, wizdata, len);
	}
}

void wiz_recv_ignore(uint8_t sn, uint16_t len)
{
	uint16_t ptr = getSn_RX_RD(sn);
	ptr += len;
	setSn_RX_RD(sn,ptr);
}

// End
