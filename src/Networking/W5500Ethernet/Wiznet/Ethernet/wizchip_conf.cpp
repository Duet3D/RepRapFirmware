//****************************************************************************/ 
//!
//! \file wizchip_conf.c
//! \brief WIZCHIP Config Header File.
//! \version 1.0.1
//! \date 2013/10/21
//! \par  Revision history
//!       <2015/02/05> Notice
//!        The version history is not updated after this point.
//!        Download the latest version directly from GitHub. Please visit the our GitHub repository for ioLibrary.
//!        >> https://github.com/Wiznet/ioLibrary_Driver
//!       <2014/05/01> V1.0.1  Refer to M20140501
//!        1. Explicit type casting in wizchip_bus_readdata() & wizchip_bus_writedata()
//            Issued by Mathias ClauBen.
//!           uint32_t type converts into ptrdiff_t first. And then recoverting it into uint8_t*
//!           For remove the warning when pointer type size is not 32bit.
//!           If ptrdiff_t doesn't support in your complier, You should must replace ptrdiff_t into your suitable pointer type.
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
//*****************************************************************************/
//A20140501 : for use the type - ptrdiff_t

#include "wizchip_conf.h"
#include "WizSpi.h"
#include <cstddef>
#include <cstring>

/**
 * @\ref WIZCHIP instance data
 */

static const char WizchipId[6] = "w5500";

int8_t ctlwizchip(ctlwizchip_type cwtype, void* arg)
{
	switch(cwtype)
	{
	case CW_RESET_WIZCHIP:
		wizchip_sw_reset();
		break;
	case CW_INIT_WIZCHIP:
		{
			uint8_t* ptmp[2] = {0,0};
			if (arg != 0)
			{
				ptmp[0] = (uint8_t*)arg;
				ptmp[1] = ptmp[0] + _WIZCHIP_SOCK_NUM_;
			}
			return wizchip_init(ptmp[0], ptmp[1]);
		}
	case CW_CLR_INTERRUPT:
		wizchip_clrinterrupt(*((intr_kind*)arg));
		break;
	case CW_GET_INTERRUPT:
		*((intr_kind*)arg) = wizchip_getinterrupt();
		break;
	case CW_SET_INTRMASK:
		wizchip_setinterruptmask(*((intr_kind*)arg));
		break;
	case CW_GET_INTRMASK:
		*((intr_kind*)arg) = wizchip_getinterruptmask();
		break;
	case CW_SET_INTRTIME:
		setINTLEVEL(*(uint16_t*)arg);
		break;
	case CW_GET_INTRTIME:
		*(uint16_t*)arg = getINTLEVEL();
		break;
	case CW_GET_ID:
		memcpy(arg, WizchipId, sizeof(WizchipId));
		break;
	case CW_RESET_PHY:
		wizphy_reset();
		break;
	case CW_SET_PHYCONF:
		wizphy_setphyconf((wiz_PhyConf*)arg);
		break;
	case CW_GET_PHYCONF:
		wizphy_getphyconf((wiz_PhyConf*)arg);
		break;
	case CW_GET_PHYSTATUS:
		break;
	case CW_SET_PHYPOWMODE:
		return wizphy_setphypmode(*(uint8_t*)arg);
		case CW_GET_PHYPOWMODE:
		{
			uint8_t tmp = wizphy_getphypmode();
			if ((int8_t)tmp == -1)
			{
				return -1;
			}
			*(uint8_t*)arg = tmp;
		}
		break;
	case CW_GET_PHYLINK:
		{
			uint8_t tmp = wizphy_getphylink();
			if ((int8_t)tmp == -1)
			{
				return -1;
			}
			*(uint8_t*)arg = tmp;
		}
		break;
	default:
		return -1;
	}
	return 0;
}

void wizchip_sw_reset(void)
{
	IPAddress gw, sn, sip;
	uint8_t mac[6];
	getSHAR(mac);
	getGAR(gw);  getSUBR(sn);  getSIPR(sip);
	setMR(MR_RST);
	getMR(); // for delay
	setSHAR(mac);
	setGAR(gw);
	setSUBR(sn);
	setSIPR(sip);
}

int8_t wizchip_init(const uint8_t* txsize, const uint8_t* rxsize)
{
	WizSpi::Init();
	wizchip_sw_reset();
	if (txsize != nullptr)
	{
		int8_t tmp = 0;
		for (uint8_t i = 0 ; i < _WIZCHIP_SOCK_NUM_; i++)
		{
			tmp += txsize[i];
			if (tmp > 16)
			{
				return -1;
			}
		}
		for	(uint8_t i = 0 ; i < _WIZCHIP_SOCK_NUM_; i++)
		{
			setSn_TXBUF_SIZE(i, txsize[i]);
		}
	}
	if (rxsize != nullptr)
	{
		int8_t tmp = 0;
		for (uint8_t i = 0 ; i < _WIZCHIP_SOCK_NUM_; i++)
		{
			tmp += rxsize[i];
			if (tmp > 16)
			{
				return -1;
			}
		}

		for (uint8_t i = 0 ; i < _WIZCHIP_SOCK_NUM_; i++)
		{
			setSn_RXBUF_SIZE(i, rxsize[i]);
		}
	}
	return 0;
}

void wizchip_clrinterrupt(intr_kind intr)
{
	setIR((uint8_t)intr);
	setSIR((uint8_t)((uint16_t)intr >> 8));
}

intr_kind wizchip_getinterrupt(void)
{
	const uint8_t ir  = getIR();
	const uint8_t sir = getSIR();
	const uint16_t ret = ((uint16_t)sir << 8) | ir;
	return (intr_kind)ret;
}

void wizchip_setinterruptmask(intr_kind intr)
{
	setIMR((uint8_t)intr);
	setSIMR((uint8_t)((uint16_t)intr >> 8));
}

intr_kind wizchip_getinterruptmask(void)
{
	const uint8_t imr  = getIMR();
	const uint8_t simr = getSIMR();
	const uint16_t ret = ((uint16_t)simr << 8) | imr;
	return (intr_kind)ret;
}

int8_t wizphy_getphylink(void)
{
	return (getPHYCFGR() & PHYCFGR_LNK_ON) ? PHY_LINK_ON : PHY_LINK_OFF;
}

int8_t wizphy_getphypmode(void)
{
	return (getPHYCFGR() & PHYCFGR_OPMDC_PDOWN) ? PHY_POWER_DOWN : PHY_POWER_NORM;
}

void wizphy_reset(void)
{
	uint8_t tmp = getPHYCFGR();
	tmp &= PHYCFGR_RST;
	setPHYCFGR(tmp);
	tmp = getPHYCFGR();
	tmp |= ~PHYCFGR_RST;
	setPHYCFGR(tmp);
}

void wizphy_setphyconf(wiz_PhyConf* phyconf)
{
	uint8_t tmp = 0;
	if (phyconf->by == PHY_CONFBY_SW)
	{
		tmp |= PHYCFGR_OPMD;
	}
	else
	{
		tmp &= ~PHYCFGR_OPMD;
	}
	if (phyconf->mode == PHY_MODE_AUTONEGO)
	{
		tmp |= PHYCFGR_OPMDC_ALLA;
	}
	else
	{
		if (phyconf->duplex == PHY_DUPLEX_FULL)
		{
			if (phyconf->speed == PHY_SPEED_100)
			{
				tmp |= PHYCFGR_OPMDC_100F;
			}
			else
			{
				tmp |= PHYCFGR_OPMDC_10F;
			}
		}
		else
		{
			if (phyconf->speed == PHY_SPEED_100)
			{
				tmp |= PHYCFGR_OPMDC_100H;
			}
			else
			{
				tmp |= PHYCFGR_OPMDC_10H;
			}
		}
	}
	setPHYCFGR(tmp);
	wizphy_reset();
}

void wizphy_getphyconf(wiz_PhyConf* phyconf)
{
	const uint8_t tmp = getPHYCFGR();
	phyconf->by = (tmp & PHYCFGR_OPMD) ? PHY_CONFBY_SW : PHY_CONFBY_HW;
	switch(tmp & PHYCFGR_OPMDC_ALLA)
	{
	case PHYCFGR_OPMDC_ALLA:
	case PHYCFGR_OPMDC_100FA:
		phyconf->mode = PHY_MODE_AUTONEGO;
		break;
	default:
		phyconf->mode = PHY_MODE_MANUAL;
		break;
	}

	switch(tmp & PHYCFGR_OPMDC_ALLA)
	{
	case PHYCFGR_OPMDC_100FA:
	case PHYCFGR_OPMDC_100F:
	case PHYCFGR_OPMDC_100H:
		phyconf->speed = PHY_SPEED_100;
		break;
	default:
		phyconf->speed = PHY_SPEED_10;
		break;
	}

	switch(tmp & PHYCFGR_OPMDC_ALLA)
	{
	case PHYCFGR_OPMDC_100FA:
	case PHYCFGR_OPMDC_100F:
	case PHYCFGR_OPMDC_10F:
		phyconf->duplex = PHY_DUPLEX_FULL;
		break;
	default:
		phyconf->duplex = PHY_DUPLEX_HALF;
		break;
	}
}

void wizphy_getphystat(wiz_PhyConf* phyconf)
{
	const uint8_t tmp = getPHYCFGR();
	phyconf->duplex = (tmp & PHYCFGR_DPX_FULL) ? PHY_DUPLEX_FULL : PHY_DUPLEX_HALF;
	phyconf->speed  = (tmp & PHYCFGR_SPD_100) ? PHY_SPEED_100 : PHY_SPEED_10;
}

int8_t wizphy_setphypmode(uint8_t pmode)
{
	uint8_t tmp = getPHYCFGR();
	if ((tmp & PHYCFGR_OPMD)== 0)
	{
		return -1;
	}
	tmp &= ~PHYCFGR_OPMDC_ALLA;
	if (pmode == PHY_POWER_DOWN)
	{
		tmp |= PHYCFGR_OPMDC_PDOWN;
	}
	else
	{
		tmp |= PHYCFGR_OPMDC_ALLA;
	}
	setPHYCFGR(tmp);
	wizphy_reset();

	tmp = getPHYCFGR();
	if (pmode == PHY_POWER_DOWN)
	{
		if (tmp & PHYCFGR_OPMDC_PDOWN)
		{
			return 0;
		}
	}
	else
	{
		if (tmp & PHYCFGR_OPMDC_ALLA)
		{
			return 0;
		}
	}
	return -1;
}

int8_t wizchip_setnetmode(netmode_type netmode)
{
	if (netmode & ~(NM_WAKEONLAN | NM_PPPOE | NM_PINGBLOCK | NM_FORCEARP))
	{
		return -1;
	}
	uint8_t tmp = getMR();
	tmp |= (uint8_t)netmode;
	setMR(tmp);
	return 0;
}

netmode_type wizchip_getnetmode(void)
{
	return (netmode_type) getMR();
}

void wizchip_settimeout(wiz_NetTimeout* nettime)
{
	setRCR(nettime->retry_cnt);
	setRTR(nettime->time_100us);
}

void wizchip_gettimeout(wiz_NetTimeout* nettime)
{
	nettime->retry_cnt = getRCR();
	nettime->time_100us = getRTR();
}

// End
