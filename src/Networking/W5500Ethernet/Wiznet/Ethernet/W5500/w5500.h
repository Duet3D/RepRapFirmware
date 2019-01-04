//*****************************************************************************
//
//! \file w5500.h
//! \brief W5500 HAL Header File.
//! \version 1.0.0
//! \date 2013/10/21
//! \par  Revision history
//!       <2015/02/05> Notice
//!        The version history is not updated after this point.
//!        Download the latest version directly from GitHub. Please visit the our GitHub repository for ioLibrary.
//!        >> https://github.com/Wiznet/ioLibrary_Driver
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

//

#ifndef  _W5500_H_
#define  _W5500_H_

#include "Networking/W5500Ethernet/Wiznet/Ethernet/wizchip_conf.h"
#include <cstdint>
#include <cstddef>
#include "spi/spi.h"
#include <General/IPAddress.h>

// The following functions are defined in Network.cpp and used by this module
extern void SpiAssertSS();
extern void SpiReleaseSS();
extern void SpiSendAddress(uint32_t);
extern uint8_t SpiReadByte();
extern void SpiSendByte(uint8_t b);
extern spi_status_t SpiReadBurst(uint8_t* pBuf, size_t len);
extern spi_status_t SpiSendBurst(const uint8_t* pBuf, size_t len);


#define _W5500_IO_BASE_              0x00000000

#define _W5500_SPI_READ_			   (0x00 << 2) //< SPI interface Read operation in Control Phase
#define _W5500_SPI_WRITE_			   (0x01 << 2) //< SPI interface Write operation in Control Phase

#define WIZCHIP_CREG_BLOCK          0x00 	//< Common register block
#define WIZCHIP_SREG_BLOCK(N)       (1+4*N) //< Socket N register block
#define WIZCHIP_TXBUF_BLOCK(N)      (2+4*N) //< Socket N Tx buffer address block
#define WIZCHIP_RXBUF_BLOCK(N)      (3+4*N) //< Socket N Rx buffer address block

#define WIZCHIP_OFFSET_INC(ADDR, N)    (ADDR + (N<<8)) //< Increase offset address


///////////////////////////////////////
// Definition For Legacy Chip Driver //
///////////////////////////////////////
#define IINCHIP_READ(ADDR)                WIZCHIP_READ(ADDR)               ///< The defined for legacy chip driver
#define IINCHIP_WRITE(ADDR,VAL)           WIZCHIP_WRITE(ADDR,VAL)          ///< The defined for legacy chip driver
#define IINCHIP_READ_BUF(ADDR,BUF,LEN)    WIZCHIP_READ_BUF(ADDR,BUF,LEN)   ///< The defined for legacy chip driver
#define IINCHIP_WRITE_BUF(ADDR,BUF,LEN)   WIZCHIP_WRITE(ADDR,BUF,LEN)      ///< The defined for legacy chip driver

//////////////////////////////
//--------------------------  defgroup ---------------------------------
/**
 * @defgroup W5500 W5500
 *
 * @brief WHIZCHIP register defines and I/O functions of @b W5500.
 *
 * - @ref WIZCHIP_register : @ref Common_register_group and @ref Socket_register_group
 * - @ref WIZCHIP_IO_Functions : @ref Basic_IO_function, @ref Common_register_access_function and @ref Socket_register_access_function
 */
 
 
/**
 * @defgroup WIZCHIP_register WIZCHIP register
 * @ingroup W5500
 *
 * @brief WHIZCHIP register defines register group of @b W5500.
 *
 * - @ref Common_register_group : Common register group
 * - @ref Socket_register_group : \c SOCKET n register group
 */


/**
 * @defgroup WIZCHIP_IO_Functions WIZCHIP I/O functions
 * @ingroup W5500
 *
 * @brief This supports the basic I/O functions for @ref WIZCHIP_register.
 *
 * - <b> Basic I/O function </b> \n
 *   WIZCHIP_READ(), WIZCHIP_WRITE(), WIZCHIP_READ_BUF(), WIZCHIP_WRITE_BUF() \n\n
 *
 * - @ref Common_register_group <b>access functions</b> \n
 * 	-# @b Mode \n
 *    getMR(), setMR()
 * 	-# @b Interrupt \n
 *    getIR(), setIR(), getIMR(), setIMR(), getSIR(), setSIR(), getSIMR(), setSIMR(), getINTLEVEL(), setINTLEVEL()
 * 	-# <b> Network Information </b> \n
 *    getSHAR(), setSHAR(), getGAR(), setGAR(), getSUBR(), setSUBR(), getSIPR(), setSIPR()
 * 	-# @b Retransmission \n
 *    getRCR(), setRCR(), getRTR(), setRTR()
 * 	-# @b PPPoE \n
 *    getPTIMER(), setPTIMER(), getPMAGIC(), getPMAGIC(), getPSID(), setPSID(), getPHAR(), setPHAR(), getPMRU(), setPMRU()
 * 	-# <b> ICMP packet </b>\n
 *    getUIPR(), getUPORTR()
 * 	-# @b etc. \n
 *    getPHYCFGR(), setPHYCFGR(), getVERSIONR() \n\n
 *
 * - \ref Socket_register_group <b>access functions</b> \n
 *   -# <b> SOCKET control</b> \n
 *      getSn_MR(), setSn_MR(), getSn_CR(), setSn_CR(), getSn_IMR(), setSn_IMR(), getSn_IR(), setSn_IR()
 *   -# <b> SOCKET information</b> \n
 *      getSn_SR(), getSn_DHAR(), setSn_DHAR(), getSn_PORT(), setSn_PORT(), getSn_DIPR(), setSn_DIPR(), getSn_DPORT(), setSn_DPORT()
 *      getSn_MSSR(), setSn_MSSR()
 *   -# <b> SOCKET communication </b> \n
 *      getSn_RXBUF_SIZE(), setSn_RXBUF_SIZE(), getSn_TXBUF_SIZE(), setSn_TXBUF_SIZE() \n
 *      getSn_TX_RD(), getSn_TX_WR(), setSn_TX_WR() \n
 *      getSn_RX_RD(), setSn_RX_RD(), getSn_RX_WR() \n
 *      getSn_TX_FSR(), getSn_RX_RSR(), getSn_KPALVTR(), setSn_KPALVTR()
 *   -# <b> IP header field </b> \n
 *      getSn_FRAG(), setSn_FRAG(),  getSn_TOS(), setSn_TOS() \n
 *      getSn_TTL(), setSn_TTL()
 */



/**
 * @defgroup Common_register_group Common register
 * @ingroup WIZCHIP_register
 *
 * @brief Common register group\n
 * It set the basic for the networking\n
 * It set the configuration such as interrupt, network information, ICMP, etc.
 * @details
 * @sa MR : Mode register.
 * @sa GAR, SUBR, SHAR, SIPR
 * @sa INTLEVEL, IR, IMR, SIR, SIMR : Interrupt.
 * @sa _RTR_, _RCR_ : Data retransmission.
 * @sa PTIMER, PMAGIC, PHAR, PSID, PMRU : PPPoE.
 * @sa UIPR, UPORTR : ICMP message.
 * @sa PHYCFGR, VERSIONR : etc.
 */
 
  
 
/**
 * @defgroup Socket_register_group Socket register
 * @ingroup WIZCHIP_register
 *
 * @brief Socket register group.\n
 * Socket register configures and control SOCKETn which is necessary to data communication.
 * @details
 * @sa Sn_MR, Sn_CR, Sn_IR, Sn_IMR : SOCKETn Control
 * @sa Sn_SR, Sn_PORT, Sn_DHAR, Sn_DIPR, Sn_DPORT : SOCKETn Information
 * @sa Sn_MSSR, Sn_TOS, Sn_TTL, Sn_KPALVTR, Sn_FRAG : Internet protocol.
 * @sa Sn_RXBUF_SIZE, Sn_TXBUF_SIZE, Sn_TX_FSR, Sn_TX_RD, Sn_TX_WR, Sn_RX_RSR, Sn_RX_RD, Sn_RX_WR : Data communication
 */
 
 
 
 /**
 * @defgroup Basic_IO_function Basic I/O function
 * @ingroup WIZCHIP_IO_Functions
 * @brief These are basic input/output functions to read values from register or write values to register.
 */

/**
 * @defgroup Common_register_access_function Common register access functions
 * @ingroup WIZCHIP_IO_Functions
 * @brief These are functions to access <b>common registers</b>.
 */

/**
 * @defgroup Socket_register_access_function Socket register access functions
 * @ingroup WIZCHIP_IO_Functions
 * @brief These are functions to access <b>socket registers</b>.
 */
 
//------------------------------- defgroup end --------------------------------------------
//----------------------------- W5500 Common Registers IOMAP -----------------------------
/**
 * @ingroup Common_register_group
 * @brief Mode Register address(R/W)\n
 * @ref MR is used for S/W reset, ping block mode, PPPoE mode and etc.
 * @details Each bit of @ref MR defined as follows.
 * <table>
 * 		<tr>  <td>7</td> <td>6</td> <td>5</td> <td>4</td> <td>3</td> <td>2</td> <td>1</td> <td>0</td>   </tr>
 * 		<tr>  <td>RST</td> <td>Reserved</td> <td>WOL</td> <td>PB</td> <td>PPPoE</td> <td>Reserved</td> <td>FARP</td> <td>Reserved</td> </tr>
 * </table>
 * - \ref MR_RST		 	: Reset
 * - \ref MR_WOL       	: Wake on LAN
 * - \ref MR_PB         : Ping block
 * - \ref MR_PPPOE      : PPPoE mode
 * - \ref MR_FARP			: Force ARP mode
 */
const uint32_t MR = _W5500_IO_BASE_ + (0x0000 << 8) + (WIZCHIP_CREG_BLOCK << 3);

/**
 * @ingroup Common_register_group
 * @brief Gateway IP Register address(R/W)
 * @details @ref GAR configures the default gateway address.
 */
 const uint32_t GAR = _W5500_IO_BASE_ + (0x0001 << 8) + (WIZCHIP_CREG_BLOCK << 3);

/**
 * @ingroup Common_register_group
 * @brief Subnet mask Register address(R/W)
 * @details @ref SUBR configures the subnet mask address.
 */
 const uint32_t SUBR = _W5500_IO_BASE_ + (0x0005 << 8) + (WIZCHIP_CREG_BLOCK << 3);

/**
 * @ingroup Common_register_group
 * @brief Source MAC Register address(R/W)
 * @details @ref SHAR configures the source hardware address.
 */
 const uint32_t SHAR = _W5500_IO_BASE_ + (0x0009 << 8) + (WIZCHIP_CREG_BLOCK << 3);

/**
 * @ingroup Common_register_group
 * @brief Source IP Register address(R/W)
 * @details @ref SIPR configures the source IP address.
 */
 const uint32_t SIPR = _W5500_IO_BASE_ + (0x000F << 8) + (WIZCHIP_CREG_BLOCK << 3);

/**
 * @ingroup Common_register_group
 * @brief Set Interrupt low level timer register address(R/W)
 * @details @ref INTLEVEL configures the Interrupt Assert Time.
 */
 const uint32_t INTLEVEL = _W5500_IO_BASE_ + (0x0013 << 8) + (WIZCHIP_CREG_BLOCK << 3);

/**
 * @ingroup Common_register_group
 * @brief Interrupt Register(R/W)
 * @details @ref IR indicates the interrupt status. Each bit of @ref IR will be still until the bit will be written to by the host.
 * If @ref IR is not equal to x00 INTn PIN is asserted to low until it is x00\n\n
 * Each bit of @ref IR defined as follows.
 * <table>
 * 		<tr>  <td>7</td> <td>6</td> <td>5</td> <td>4</td> <td>3</td> <td>2</td> <td>1</td> <td>0</td>   </tr>
 * 		<tr>  <td>CONFLICT</td> <td>UNREACH</td> <td>PPPoE</td> <td>MP</td> <td>Reserved</td> <td>Reserved</td> <td>Reserved</td> <td>Reserved</td> </tr>
 * </table>
 * - \ref IR_CONFLICT : IP conflict
 * - \ref IR_UNREACH  : Destination unreachable
 * - \ref IR_PPPoE	  : PPPoE connection close
 * - \ref IR_MP		  : Magic packet
 */
 const uint32_t IR = _W5500_IO_BASE_ + (0x0015 << 8) + (WIZCHIP_CREG_BLOCK << 3);

/**
 * @ingroup Common_register_group
 * @brief Interrupt mask register(R/W)
 * @details @ref _IMR_ is used to mask interrupts. Each bit of @ref _IMR_ corresponds to each bit of @ref IR.
 * When a bit of @ref _IMR_ is and the corresponding bit of @ref IR is  an interrupt will be issued. In other words,
 * if a bit of @ref _IMR_ is  an interrupt will not be issued even if the corresponding bit of @ref IR is \n\n
 * Each bit of @ref _IMR_ defined as the following.
 * <table>
 * 		<tr>  <td>7</td> <td>6</td> <td>5</td> <td>4</td> <td>3</td> <td>2</td> <td>1</td> <td>0</td>   </tr>
 * 		<tr>  <td>IM_IR7</td> <td>IM_IR6</td> <td>IM_IR5</td> <td>IM_IR4</td> <td>Reserved</td> <td>Reserved</td> <td>Reserved</td> <td>Reserved</td> </tr>
 * </table>
 * - \ref IM_IR7 : IP Conflict Interrupt Mask
 * - \ref IM_IR6 : Destination unreachable Interrupt Mask
 * - \ref IM_IR5 : PPPoE Close Interrupt Mask
 * - \ref IM_IR4 : Magic Packet Interrupt Mask
 */
 const uint32_t _IMR_ = _W5500_IO_BASE_ + (0x0016 << 8) + (WIZCHIP_CREG_BLOCK << 3);

/**
 * @ingroup Common_register_group
 * @brief Socket Interrupt Register(R/W)
 * @details @ref SIR indicates the interrupt status of Socket.\n
 * Each bit of @ref SIR be still until @ref Sn_IR is cleared by the host.\n
 * If @ref Sn_IR is not equal to x00 the n-th bit of @ref SIR is and INTn PIN is asserted until @ref SIR is x00 */
 const uint32_t SIR = _W5500_IO_BASE_ + (0x0017 << 8) + (WIZCHIP_CREG_BLOCK << 3);

/**
 * @ingroup Common_register_group
 * @brief Socket Interrupt Mask Register(R/W)
 * @details Each bit of @ref SIMR corresponds to each bit of @ref SIR.
 * When a bit of @ref SIMR is and the corresponding bit of @ref SIR is  Interrupt will be issued.
 * In other words, if a bit of @ref SIMR is  an interrupt will be not issued even if the corresponding bit of @ref SIR is 
 */
 const uint32_t SIMR = _W5500_IO_BASE_ + (0x0018 << 8) + (WIZCHIP_CREG_BLOCK << 3);

/**
 * @ingroup Common_register_group
 * @brief Timeout register address( 1 is 100us )(R/W)
 * @details @ref _RTR_ configures the retransmission timeout period. The unit of timeout period is 100us and the default of @ref _RTR_ is x07D0.
 * And so the default timeout period is 200ms(100us X 2000). During the time configured by @ref _RTR_, W5500 waits for the peer response
 * to the packet that is transmitted by \ref Sn_CR (CONNECT, DISCON, CLOSE, SEND, SEND_MAC, SEND_KEEP command).
 * If the peer does not respond within the @ref _RTR_ time, W5500 retransmits the packet or issues timeout.
 */
 const uint32_t _RTR_ = _W5500_IO_BASE_ + (0x0019 << 8) + (WIZCHIP_CREG_BLOCK << 3);

/**
 * @ingroup Common_register_group
 * @brief Retry count register(R/W)
 * @details @ref _RCR_ configures the number of time of retransmission.
 * When retransmission occurs as many as ref _RCR_+1 Timeout interrupt is issued (@ref Sn_IR_TIMEOUT = '1').
 */
 const uint32_t _RCR_ = _W5500_IO_BASE_ + (0x001B << 8) + (WIZCHIP_CREG_BLOCK << 3);

/**
 * @ingroup Common_register_group
 * @brief PPP LCP Request Timer register  in PPPoE mode(R/W)
 * @details @ref PTIMER configures the time for sending LCP echo request. The unit of time is 25ms.
 */
 const uint32_t PTIMER = _W5500_IO_BASE_ + (0x001C << 8) + (WIZCHIP_CREG_BLOCK << 3);

/**
 * @ingroup Common_register_group
 * @brief PPP LCP Magic number register  in PPPoE mode(R/W)
 * @details @ref PMAGIC configures the 4bytes magic number to be used in LCP negotiation.
 */
 const uint32_t PMAGIC = _W5500_IO_BASE_ + (0x001D << 8) + (WIZCHIP_CREG_BLOCK << 3);

/**
 * @ingroup Common_register_group
 * @brief PPP Destination MAC Register address(R/W)
 * @details @ref PHAR configures the PPPoE server hardware address that is acquired during PPPoE connection process.
 */
 const uint32_t PHAR = _W5500_IO_BASE_ + (0x001E << 8) + (WIZCHIP_CREG_BLOCK << 3);

/**
 * @ingroup Common_register_group
 * @brief PPP Session Identification Register(R/W)
 * @details @ref PSID configures the PPPoE sever session ID acquired during PPPoE connection process.
 */
 const uint32_t PSID = _W5500_IO_BASE_ + (0x0024 << 8) + (WIZCHIP_CREG_BLOCK << 3);

/**
 * @ingroup Common_register_group
 * @brief PPP Maximum Segment Size(MSS) register(R/W)
 * @details @ref PMRU configures the maximum receive unit of PPPoE.
 */
 const uint32_t PMRU = _W5500_IO_BASE_ + (0x0026 << 8) + (WIZCHIP_CREG_BLOCK << 3);

/**
 * @ingroup Common_register_group
 * @brief Unreachable IP register address in UDP mode(R)
 * @details W5500 receives an ICMP packet(Destination port unreachable) when data is sent to a port number
 * which socket is not open and @ref IR_UNREACH bit of @ref IR becomes and @ref UIPR & @ref UPORTR indicates
 * the destination IP address & port number respectively.
 */
 const uint32_t UIPR = _W5500_IO_BASE_ + (0x0028 << 8) + (WIZCHIP_CREG_BLOCK << 3);

/**
 * @ingroup Common_register_group
 * @brief Unreachable Port register address in UDP mode(R)
 * @details W5500 receives an ICMP packet(Destination port unreachable) when data is sent to a port number
 * which socket is not open and @ref IR_UNREACH bit of @ref IR becomes and @ref UIPR & @ref UPORTR
 * indicates the destination IP address & port number respectively.
 */
 const uint32_t UPORTR = _W5500_IO_BASE_ + (0x002C << 8) + (WIZCHIP_CREG_BLOCK << 3);

/**
 * @ingroup Common_register_group
 * @brief PHY Status Register(R/W)
 * @details @ref PHYCFGR configures PHY operation mode and resets PHY. In addition, @ref PHYCFGR indicates the status of PHY such as duplex, Speed, Link.
 */
 const uint32_t PHYCFGR = _W5500_IO_BASE_ + (0x002E << 8) + (WIZCHIP_CREG_BLOCK << 3);

// Reserved			         (_W5500_IO_BASE_ + (0x002F << 8) + (WIZCHIP_CREG_BLOCK << 3))
// Reserved			         (_W5500_IO_BASE_ + (0x0030 << 8) + (WIZCHIP_CREG_BLOCK << 3))
// Reserved			         (_W5500_IO_BASE_ + (0x0031 << 8) + (WIZCHIP_CREG_BLOCK << 3))
// Reserved			         (_W5500_IO_BASE_ + (0x0032 << 8) + (WIZCHIP_CREG_BLOCK << 3))
// Reserved			         (_W5500_IO_BASE_ + (0x0033 << 8) + (WIZCHIP_CREG_BLOCK << 3))
// Reserved			         (_W5500_IO_BASE_ + (0x0034 << 8) + (WIZCHIP_CREG_BLOCK << 3))
// Reserved			         (_W5500_IO_BASE_ + (0x0035 << 8) + (WIZCHIP_CREG_BLOCK << 3))
// Reserved			         (_W5500_IO_BASE_ + (0x0036 << 8) + (WIZCHIP_CREG_BLOCK << 3))
// Reserved			         (_W5500_IO_BASE_ + (0x0037 << 8) + (WIZCHIP_CREG_BLOCK << 3))
// Reserved			         (_W5500_IO_BASE_ + (0x0038 << 8) + (WIZCHIP_CREG_BLOCK << 3))

/**
 * @ingroup Common_register_group
 * @brief chip version register address(R)
 * @details @ref VERSIONR always indicates the W5500 version as @b 0x04.
 */
 const uint32_t VERSIONR = _W5500_IO_BASE_ + (0x0039 << 8) + (WIZCHIP_CREG_BLOCK << 3);


//----------------------------- W5500 Socket Registers IOMAP -----------------------------
/**
 * @ingroup Socket_register_group
 * @brief socket Mode register(R/W)
 * @details @ref Sn_MR configures the option or protocol type of Socket n.\n\n
 * Each bit of @ref Sn_MR defined as the following.
 * <table>
 * 		<tr>  <td>7</td> <td>6</td> <td>5</td> <td>4</td> <td>3</td> <td>2</td> <td>1</td> <td>0</td>   </tr>
 * 		<tr>  <td>MULTI/MFEN</td> <td>BCASTB</td> <td>ND/MC/MMB</td> <td>UCASTB/MIP6B</td> <td>Protocol[3]</td> <td>Protocol[2]</td> <td>Protocol[1]</td> <td>Protocol[0]</td> </tr>
 * </table>
 * - @ref Sn_MR_MULTI	: Support UDP Multicasting
 * - @ref Sn_MR_BCASTB	: Broadcast block <b>in UDP Multicasting</b>
 * - @ref Sn_MR_ND		: No Delayed Ack(TCP) flag
 * - @ref Sn_MR_MC   	: IGMP version used <b>in UDP mulitcasting</b>
 * - @ref Sn_MR_MMB    	: Multicast Blocking <b>in @ref Sn_MR_MACRAW mode</b>
 * - @ref Sn_MR_UCASTB	: Unicast Block <b>in UDP Multicating</b>
 * - @ref Sn_MR_MIP6B   : IPv6 packet Blocking <b>in @ref Sn_MR_MACRAW mode</b>
 * - <b>Protocol</b>
 * <table>
 * 		<tr>   <td><b>Protocol[3]</b></td> <td><b>Protocol[2]</b></td> <td><b>Protocol[1]</b></td> <td><b>Protocol[0]</b></td> <td>@b Meaning</td>   </tr>
 * 		<tr>   <td>0</td> <td>0</td> <td>0</td> <td>0</td> <td>Closed</td>   </tr>
 * 		<tr>   <td>0</td> <td>0</td> <td>0</td> <td>1</td> <td>TCP</td>   </tr>
 * 		<tr>   <td>0</td> <td>0</td> <td>1</td> <td>0</td> <td>UDP</td>   </tr>
 * 		<tr>   <td>0</td> <td>1</td> <td>0</td> <td>0</td> <td>MACRAW</td>   </tr>
 * </table>
 *	- @ref Sn_MR_MACRAW	: MAC LAYER RAW SOCK \n
 *  - @ref Sn_MR_UDP		: UDP
 *  - @ref Sn_MR_TCP		: TCP
 *  - @ref Sn_MR_CLOSE	: Unused socket
 *  @note MACRAW mode should be only used in Socket 0.
 */
 static inline uint32_t Sn_MR(uint8_t N)
 {
	 return _W5500_IO_BASE_ + (0x0000 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3);
 }

/**
 * @ingroup Socket_register_group
 * @brief Socket command register(R/W)
 * @details This is used to set the command for Socket n such as OPEN, CLOSE, CONNECT, LISTEN, SEND, and RECEIVE.\n
 * After W5500 accepts the command, the @ref Sn_CR register is automatically cleared to 0x00.
 * Even though @ref Sn_CR is cleared to 0x00, the command is still being processed.\n
 * To check whether the command is completed or not, please check the @ref Sn_IR or @ref Sn_SR.
 * - @ref Sn_CR_OPEN 		: Initialize or open socket.
 * - @ref Sn_CR_LISTEN 		: Wait connection request in TCP mode(<b>Server mode</b>)
 * - @ref Sn_CR_CONNECT 	: Send connection request in TCP mode(<b>Client mode</b>)
 * - @ref Sn_CR_DISCON 		: Send closing request in TCP mode.
 * - @ref Sn_CR_CLOSE   	: Close socket.
 * - @ref Sn_CR_SEND    	: Update TX buffer pointer and send data.
 * - @ref Sn_CR_SEND_MAC	: Send data with MAC address, so without ARP process.
 * - @ref Sn_CR_SEND_KEEP 	: Send keep alive message.
 * - @ref Sn_CR_RECV		: Update RX buffer pointer and receive data.
 */
 static inline uint32_t Sn_CR(uint8_t N)
 {
	 return _W5500_IO_BASE_ + (0x0001 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3);
 }

/**
 * @ingroup Socket_register_group
 * @brief Socket interrupt register(R)
 * @details @ref Sn_IR indicates the status of Socket Interrupt such as establishment, termination, receiving data, timeout).\n
 * When an interrupt occurs and the corresponding bit of @ref Sn_IMR is  the corresponding bit of @ref Sn_IR becomes \n
 * In order to clear the @ref Sn_IR bit, the host should write the bit to \n
 * <table>
 * 		<tr>  <td>7</td> <td>6</td> <td>5</td> <td>4</td> <td>3</td> <td>2</td> <td>1</td> <td>0</td>   </tr>
 * 		<tr>  <td>Reserved</td> <td>Reserved</td> <td>Reserved</td> <td>SEND_OK</td> <td>TIMEOUT</td> <td>RECV</td> <td>DISCON</td> <td>CON</td> </tr>
 * </table>
 * - \ref Sn_IR_SENDOK : <b>SEND_OK Interrupt</b>
 * - \ref Sn_IR_TIMEOUT : <b>TIMEOUT Interrupt</b>
 * - \ref Sn_IR_RECV : <b>RECV Interrupt</b>
 * - \ref Sn_IR_DISCON : <b>DISCON Interrupt</b>
 * - \ref Sn_IR_CON : <b>CON Interrupt</b>
 */
 static inline uint32_t Sn_IR(uint8_t N)
 {
	 return _W5500_IO_BASE_ + (0x0002 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3);
 }

/**
 * @ingroup Socket_register_group
 * @brief Socket status register(R)
 * @details @ref Sn_SR indicates the status of Socket n.\n
 * The status of Socket n is changed by @ref Sn_CR or some special control packet as SYN, FIN packet in TCP.
 * @par Normal status
 * - @ref SOCK_CLOSED 		: Closed
 * - @ref SOCK_INIT   		: Initiate state
 * - @ref SOCK_LISTEN    	: Listen state
 * - @ref SOCK_ESTABLISHED 	: Success to connect
 * - @ref SOCK_CLOSE_WAIT   : Closing state
 * - @ref SOCK_UDP   		: UDP socket
 * - @ref SOCK_MACRAW  		: MAC raw mode socket
 *@par Temporary status during changing the status of Socket n.
 * - @ref SOCK_SYNSENT   	: This indicates Socket n sent the connect-request packet (SYN packet) to a peer.
 * - @ref SOCK_SYNRECV    	: It indicates Socket n successfully received the connect-request packet (SYN packet) from a peer.
 * - @ref SOCK_FIN_WAIT		: Connection state
 * - @ref SOCK_CLOSING		: Closing state
 * - @ref SOCK_TIME_WAIT	: Closing state
 * - @ref SOCK_LAST_ACK 	: Closing state
 */
 static inline uint32_t Sn_SR(uint8_t N)
 {
	 return _W5500_IO_BASE_ + (0x0003 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3);
 }

/**
 * @ingroup Socket_register_group
 * @brief source port register(R/W)
 * @details @ref Sn_PORT configures the source port number of Socket n.
 * It is valid when Socket n is used in TCP/UDP mode. It should be set before OPEN command is ordered.
 */
 static inline uint32_t Sn_PORT(uint8_t N)
 {
	 return _W5500_IO_BASE_ + (0x0004 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3);
 }

/**
 * @ingroup Socket_register_group
 * @brief Peer MAC register address(R/W)
 * @details @ref Sn_DHAR configures the destination hardware address of Socket n when using SEND_MAC command in UDP mode or
 * it indicates that it is acquired in ARP-process by CONNECT/SEND command.
 */
 static inline uint32_t Sn_DHAR(uint8_t N)
 {
	 return _W5500_IO_BASE_ + (0x0006 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3);
 }

/**
 * @ingroup Socket_register_group
 * @brief Peer IP register address(R/W)
 * @details @ref Sn_DIPR configures or indicates the destination IP address of Socket n. It is valid when Socket n is used in TCP/UDP mode.
 * In TCP client mode, it configures an IP address of TCP serverbefore CONNECT command.
 * In TCP server mode, it indicates an IP address of TCP clientafter successfully establishing connection.
 * In UDP mode, it configures an IP address of peer to be received the UDP packet by SEND or SEND_MAC command.
 */
 static inline uint32_t  Sn_DIPR(uint8_t N)
 {
	 return _W5500_IO_BASE_ + (0x000C << 8) + (WIZCHIP_SREG_BLOCK(N) << 3);
 }

/**
 * @ingroup Socket_register_group
 * @brief Peer port register address(R/W)
 * @details @ref Sn_DPORT configures or indicates the destination port number of Socket n. It is valid when Socket n is used in TCP/UDP mode.
 * In TCP clientmode, it configures the listen port number of TCP serverbefore CONNECT command.
 * In TCP Servermode, it indicates the port number of TCP client after successfully establishing connection.
 * In UDP mode, it configures the port number of peer to be transmitted the UDP packet by SEND/SEND_MAC command.
 */
 static inline uint32_t  Sn_DPORT(uint8_t N)
 {
	 return _W5500_IO_BASE_ + (0x0010 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3);
 }

/**
 * @ingroup Socket_register_group
 * @brief Maximum Segment Size(Sn_MSSR0) register address(R/W)
 * @details @ref Sn_MSSR configures or indicates the MTU(Maximum Transfer Unit) of Socket n.
 */
 static inline uint32_t  Sn_MSSR(uint8_t N)
 {
	 return _W5500_IO_BASE_ + (0x0012 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3);
 }

// Reserved			         (_W5500_IO_BASE_ + (0x0014 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

/**
 * @ingroup Socket_register_group
 * @brief IP Type of Service(TOS) Register(R/W)
 * @details @ref Sn_TOS configures the TOS(Type Of Service field in IP Header) of Socket n.
 * It is set before OPEN command.
 */
static inline uint32_t  Sn_TOS(uint8_t N)
{
	return _W5500_IO_BASE_ + (0x0015 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3);
}

/**
 * @ingroup Socket_register_group
 * @brief IP Time to live(TTL) Register(R/W)
 * @details @ref Sn_TTL configures the TTL(Time To Live field in IP header) of Socket n.
 * It is set before OPEN command.
 */
static inline uint32_t  Sn_TTL(uint8_t N)
{
	return _W5500_IO_BASE_ + (0x0016 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3);
}

// Reserved			         (_W5500_IO_BASE_ + (0x0017 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))
// Reserved			         (_W5500_IO_BASE_ + (0x0018 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3)) 
// Reserved			         (_W5500_IO_BASE_ + (0x0019 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))
// Reserved			         (_W5500_IO_BASE_ + (0x001A << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))
// Reserved			         (_W5500_IO_BASE_ + (0x001B << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))
// Reserved			         (_W5500_IO_BASE_ + (0x001C << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))
// Reserved			         (_W5500_IO_BASE_ + (0x001D << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))

/**
 * @ingroup Socket_register_group
 * @brief Receive memory size register(R/W)
 * @details @ref Sn_RXBUF_SIZE configures the RX buffer block size of Socket n.
 * Socket n RX Buffer Block size can be configured with 1,2,4,8, and 16 Kbytes.
 * If a different size is configured, the data cannot be normally received from a peer.
 * Although Socket n RX Buffer Block size is initially configured to 2Kbytes,
 * user can re-configure its size using @ref Sn_RXBUF_SIZE. The total sum of @ref Sn_RXBUF_SIZE can not be exceed 16Kbytes.
 * When exceeded, the data reception error is occurred.
 */
static inline uint32_t  Sn_RXBUF_SIZE(uint8_t N)
{
	 return _W5500_IO_BASE_ + (0x001E << 8) + (WIZCHIP_SREG_BLOCK(N) << 3);
}

/**
 * @ingroup Socket_register_group
 * @brief Transmit memory size register(R/W)
 * @details @ref Sn_TXBUF_SIZE configures the TX buffer block size of Socket n. Socket n TX Buffer Block size can be configured with 1,2,4,8, and 16 Kbytes.
 * If a different size is configured, the data can�t be normally transmitted to a peer.
 * Although Socket n TX Buffer Block size is initially configured to 2Kbytes,
 * user can be re-configure its size using @ref Sn_TXBUF_SIZE. The total sum of @ref Sn_TXBUF_SIZE can not be exceed 16Kbytes.
 * When exceeded, the data transmission error is occurred.
 */
static inline uint32_t  Sn_TXBUF_SIZE(uint8_t N)
{
	 return _W5500_IO_BASE_ + (0x001F << 8) + (WIZCHIP_SREG_BLOCK(N) << 3);
}

/**
 * @ingroup Socket_register_group
 * @brief Transmit free memory size register(R)
 * @details @ref Sn_TX_FSR indicates the free size of Socket n TX Buffer Block. It is initialized to the configured size by @ref Sn_TXBUF_SIZE.
 * Data bigger than @ref Sn_TX_FSR should not be saved in the Socket n TX Buffer because the bigger data overwrites the previous saved data not yet sent.
 * Therefore, check before saving the data to the Socket n TX Buffer, and if data is equal or smaller than its checked size,
 * transmit the data with SEND/SEND_MAC command after saving the data in Socket n TX buffer. But, if data is bigger than its checked size,
 * transmit the data after dividing into the checked size and saving in the Socket n TX buffer.
 */
static inline uint32_t  Sn_TX_FSR(uint8_t N)
{
	 return _W5500_IO_BASE_ + (0x0020 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3);
}

/**
 * @ingroup Socket_register_group
 * @brief Transmit memory read pointer register address(R)
 * @details @ref Sn_TX_RD is initialized by OPEN command. However, if Sn_MR(P[3:0]) is TCP mode(001, it is re-initialized while connecting with TCP.
 * After its initialization, it is auto-increased by SEND command.
 * SEND command transmits the saved data from the current @ref Sn_TX_RD to the @ref Sn_TX_WR in the Socket n TX Buffer.
 * After transmitting the saved data, the SEND command increases the @ref Sn_TX_RD as same as the @ref Sn_TX_WR.
 * If its increment value exceeds the maximum value 0xFFFF, (greater than 0x10000 and the carry bit occurs),
 * then the carry bit is ignored and will automatically update with the lower 16bits value.
 */
static inline uint32_t  Sn_TX_RD(uint8_t N)
{
	 return _W5500_IO_BASE_ + (0x0022 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3);
}

/**
 * @ingroup Socket_register_group
 * @brief Transmit memory write pointer register address(R/W)
 * @details @ref Sn_TX_WR is initialized by OPEN command. However, if Sn_MR(P[3:0]) is TCP mode(001, it is re-initialized while connecting with TCP.\n
 * It should be read or be updated like as follows.\n
 * 1. Read the starting address for saving the transmitting data.\n
 * 2. Save the transmitting data from the starting address of Socket n TX buffer.\n
 * 3. After saving the transmitting data, update @ref Sn_TX_WR to the increased value as many as transmitting data size.
 * If the increment value exceeds the maximum value 0xFFFF(greater than 0x10000 and the carry bit occurs),
 * then the carry bit is ignored and will automatically update with the lower 16bits value.\n
 * 4. Transmit the saved data in Socket n TX Buffer by using SEND/SEND command
 */
static inline uint32_t  Sn_TX_WR(uint8_t N)
{
	 return _W5500_IO_BASE_ + (0x0024 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3);
}

/**
 * @ingroup Socket_register_group
 * @brief Received data size register(R)
 * @details @ref Sn_RX_RSR indicates the data size received and saved in Socket n RX Buffer.
 * @ref Sn_RX_RSR does not exceed the @ref Sn_RXBUF_SIZE and is calculated as the difference between
 * Socket n RX Write Pointer (@ref Sn_RX_WR)and Socket n RX Read Pointer (@ref Sn_RX_RD)
 */
static inline uint32_t  Sn_RX_RSR(uint8_t N)
{
	 return _W5500_IO_BASE_ + (0x0026 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3);
}

/**
 * @ingroup Socket_register_group
 * @brief Read point of Receive memory(R/W)
 * @details @ref Sn_RX_RD is initialized by OPEN command. Make sure to be read or updated as follows.\n
 * 1. Read the starting save address of the received data.\n
 * 2. Read data from the starting address of Socket n RX Buffer.\n
 * 3. After reading the received data, Update @ref Sn_RX_RD to the increased value as many as the reading size.
 * If the increment value exceeds the maximum value 0xFFFF, that is, is greater than 0x10000 and the carry bit occurs,
 * update with the lower 16bits value ignored the carry bit.\n
 * 4. Order RECV command is for notifying the updated @ref Sn_RX_RD to W5500.
 */
static inline uint32_t  Sn_RX_RD(uint8_t N)
{
	 return _W5500_IO_BASE_ + (0x0028 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3);
}

/**
 * @ingroup Socket_register_group
 * @brief Write point of Receive memory(R)
 * @details @ref Sn_RX_WR is initialized by OPEN command and it is auto-increased by the data reception.
 * If the increased value exceeds the maximum value 0xFFFF, (greater than 0x10000 and the carry bit occurs),
 * then the carry bit is ignored and will automatically update with the lower 16bits value.
 */
static inline uint32_t Sn_RX_WR(uint8_t N)
{
	 return _W5500_IO_BASE_ + (0x002A << 8) + (WIZCHIP_SREG_BLOCK(N) << 3);
}

/**
 * @ingroup Socket_register_group
 * @brief socket interrupt mask register(R)
 * @details @ref Sn_IMR masks the interrupt of Socket n.
 * Each bit corresponds to each bit of @ref Sn_IR. When a Socket n Interrupt is occurred and the corresponding bit of @ref Sn_IMR is 
 * the corresponding bit of @ref Sn_IR becomes  When both the corresponding bit of @ref Sn_IMR and @ref Sn_IR are and the n-th bit of @ref IR is 
 * Host is interrupted by asserted INTn PIN to low.
 */
static inline uint32_t Sn_IMR(uint8_t N)
{
	 return _W5500_IO_BASE_ + (0x002C << 8) + (WIZCHIP_SREG_BLOCK(N) << 3);
}

/**
 * @ingroup Socket_register_group
 * @brief Fragment field value in IP header register(R/W)
 * @details @ref Sn_FRAG configures the FRAG(Fragment field in IP header).
 */
 static inline uint32_t Sn_FRAG(uint8_t N)
 {
	 return _W5500_IO_BASE_ + (0x002D << 8) + (WIZCHIP_SREG_BLOCK(N) << 3);
 }

/**
 * @ingroup Socket_register_group
 * @brief Keep Alive Timer register(R/W)
 * @details @ref Sn_KPALVTR configures the transmitting timer of �KEEP ALIVE(KA)packet of SOCKETn. It is valid only in TCP mode,
 * and ignored in other modes. The time unit is 5s.
 * KA packet is transmittable after @ref Sn_SR is changed to SOCK_ESTABLISHED and after the data is transmitted or received to/from a peer at least once.
 * In case of '@ref Sn_KPALVTR > 0', W5500 automatically transmits KA packet after time-period for checking the TCP connection (Auto-keepalive-process).
 * In case of '@ref Sn_KPALVTR = 0', Auto-keep-alive-process will not operate,
 * and KA packet can be transmitted by SEND_KEEP command by the host (Manual-keep-alive-process).
 * Manual-keep-alive-process is ignored in case of '@ref Sn_KPALVTR > 0'.
 */
static inline uint32_t Sn_KPALVTR(uint8_t N)
{
	return _W5500_IO_BASE_ + (0x002F << 8) + (WIZCHIP_SREG_BLOCK(N) << 3);
}

//#define Sn_TSR(N)          (_W5500_IO_BASE_ + (0x0030 << 8) + (WIZCHIP_SREG_BLOCK(N) << 3))


//----------------------------- W5500 Register values  -----------------------------

/* MODE register values */
/**
 * @brief Reset
 * @details If this bit is  All internal registers will be initialized. It will be automatically cleared as after S/W reset.
 */
const uint8_t MR_RST = 0x80;

/**
 * @brief Wake on LAN
 * @details 0 : Disable WOL mode\n
 * 1 : Enable WOL mode\n
 * If WOL mode is enabled and the received magic packet over UDP has been normally processed, the Interrupt PIN (INTn) asserts to low.
 * When using WOL mode, the UDP Socket should be opened with any source port number. (Refer to Socket n Mode Register (@ref Sn_MR) for opening Socket.)
 * @note The magic packet over UDP supported by W5500 consists of 6 bytes synchronization stream (xFFFFFFFFFFFF and
 * 16 times Target MAC address stream in UDP payload. The options such like password are ignored. You can use any UDP source port number for WOL mode.
 */
const uint8_t MR_WOL = 0x20;

/**
 * @brief Ping block
 * @details 0 : Disable Ping block\n
 * 1 : Enable Ping block\n
 * If the bit is  it blocks the response to a ping request.
 */
const uint8_t MR_PB = 0x10;

/**
 * @brief Enable PPPoE
 * @details 0 : DisablePPPoE mode\n
 * 1 : EnablePPPoE mode\n
 * If you use ADSL, this bit should be 
 */
const uint8_t MR_PPPOE = 0x08;

/**
 * @brief Enable UDP_FORCE_ARP CHECHK
 * @details 0 : Disable Force ARP mode\n
 * 1 : Enable Force ARP mode\n
 * In Force ARP mode, It forces on sending ARP Request whenever data is sent.
 */
const uint8_t MR_FARP = 0x02;

/* IR register values */
/**
 * @brief Check IP conflict.
 * @details Bit is set as when own source IP address is same with the sender IP address in the received ARP request.
 */
const uint8_t IR_CONFLICT = 0x80;

/**
 * @brief Get the destination unreachable message in UDP sending.
 * @details When receiving the ICMP (Destination port unreachable) packet, this bit is set as 
 * When this bit is  Destination Information such as IP address and Port number may be checked with the corresponding @ref UIPR & @ref UPORTR.
 */
const uint8_t IR_UNREACH = 0x40;

/**
 * @brief Get the PPPoE close message.
 * @details When PPPoE is disconnected during PPPoE mode, this bit is set.
 */
const uint8_t IR_PPPoE = 0x20;

/**
 * @brief Get the magic packet interrupt.
 * @details When WOL mode is enabled and receives the magic packet over UDP, this bit is set.
 */
const uint8_t IR_MP = 0x10;


/* PHYCFGR register value */
const uint8_t PHYCFGR_RST			= (uint8_t)~(1<<7);		//< For PHY reset, must operate AND mask.
const uint8_t PHYCFGR_OPMD			= (1<<6);   			// Configre PHY with OPMDC value
const uint8_t PHYCFGR_OPMDC_ALLA	= (7<<3);
const uint8_t PHYCFGR_OPMDC_PDOWN	= (6<<3);
const uint8_t PHYCFGR_OPMDC_NA		= (5<<3);
const uint8_t PHYCFGR_OPMDC_100FA	= (4<<3);
const uint8_t PHYCFGR_OPMDC_100F	= (3<<3);
const uint8_t PHYCFGR_OPMDC_100H	= (2<<3);
const uint8_t PHYCFGR_OPMDC_10F		= (1<<3);
const uint8_t PHYCFGR_OPMDC_10H		= (0<<3);
const uint8_t PHYCFGR_DPX_FULL		= (1<<2);
const uint8_t PHYCFGR_DPX_HALF		= (0<<2);
const uint8_t PHYCFGR_SPD_100		= (1<<1);
const uint8_t PHYCFGR_SPD_10		= (0<<1);
const uint8_t PHYCFGR_LNK_ON		= (1<<0);
const uint8_t PHYCFGR_LNK_OFF		= (0<<0);

/* IMR register values */
/**
 * @brief IP Conflict Interrupt Mask.
 * @details 0: Disable IP Conflict Interrupt\n
 * 1: Enable IP Conflict Interrupt
 */
const uint8_t IM_IR7 = 0x80;

/**
 * @brief Destination unreachable Interrupt Mask.
 * @details 0: Disable Destination unreachable Interrupt\n
 * 1: Enable Destination unreachable Interrupt
 */
const uint8_t IM_IR6 = 0x40;

/**
 * @brief PPPoE Close Interrupt Mask.
 * @details 0: Disable PPPoE Close Interrupt\n
 * 1: Enable PPPoE Close Interrupt
 */
const uint8_t IM_IR5 = 0x20;

/**
 * @brief Magic Packet Interrupt Mask.
 * @details 0: Disable Magic Packet Interrupt\n
 * 1: Enable Magic Packet Interrupt
 */
const uint8_t IM_IR4 = 0x10;

/* Sn_MR Default values */
/**
 * @brief Support UDP Multicasting
 * @details 0 : disable Multicasting\n
 * 1 : enable Multicasting\n
 * This bit is applied only during UDP mode(P[3:0] = 010.\n
 * To use multicasting, @ref Sn_DIPR & @ref Sn_DPORT should be respectively configured with the multicast group IP address & port number
 * before Socket n is opened by OPEN command of @ref Sn_CR.
 */
const uint8_t Sn_MR_MULTI = 0x80;

/**
 * @brief Broadcast block in UDP Multicasting.
 * @details 0 : disable Broadcast Blocking\n
 * 1 : enable Broadcast Blocking\n
 * This bit blocks to receive broadcasting packet during UDP mode(P[3:0] = 010.\m
 * In addition, This bit does when MACRAW mode(P[3:0] = 100
 */
const uint8_t Sn_MR_BCASTB = 0x40;

/**
 * @brief No Delayed Ack(TCP), Multicast flag
 * @details 0 : Disable No Delayed ACK option\n
 * 1 : Enable No Delayed ACK option\n
 * This bit is applied only during TCP mode (P[3:0] = 001.\n
 * When this bit is  It sends the ACK packet without delay as soon as a Data packet is received from a peer.\n
 * When this bit is  It sends the ACK packet after waiting for the timeout time configured by @ref _RTR_.
 */
const uint8_t Sn_MR_ND = 0x20;

/**
 * @brief Unicast Block in UDP Multicasting
 * @details 0 : disable Unicast Blocking\n
 * 1 : enable Unicast Blocking\n
 * This bit blocks receiving the unicast packet during UDP mode(P[3:0] = 010 and MULTI = 
 */
const uint8_t Sn_MR_UCASTB = 0x10;

/**
 * @brief MAC LAYER RAW SOCK
 * @details This configures the protocol mode of Socket n.
 * @note MACRAW mode should be only used in Socket 0.
 */
const uint8_t Sn_MR_MACRAW = 0x04;

//#define Sn_MR_IPRAW                  0x03     /**< IP LAYER RAW SOCK */

/**
 * @brief UDP
 * @details This configures the protocol mode of Socket n.
 */
const uint8_t Sn_MR_UDP = 0x02;

/**
 * @brief TCP
 * @details This configures the protocol mode of Socket n.
 */
const uint8_t Sn_MR_TCP = 0x01;

/**
 * @brief Unused socket
 * @details This configures the protocol mode of Socket n.
 */
const uint8_t Sn_MR_CLOSE = 0x00;

/* Sn_MR values used with Sn_MR_MACRAW */
/**
 * @brief MAC filter enable in @ref Sn_MR_MACRAW mode
 * @details 0 : disable MAC Filtering\n
 * 1 : enable MAC Filtering\n
 * This bit is applied only during MACRAW mode(P[3:0] = 100.\n
 * When set as  W5500 can only receive broadcasting packet or packet sent to itself.
 * When this bit is  W5500 can receive all packets on Ethernet.
 * If user wants to implement Hybrid TCP/IP stack,
 * it is recommended that this bit is set as for reducing host overhead to process the all received packets.
 */
const uint8_t Sn_MR_MFEN = Sn_MR_MULTI;

/**
 * @brief Multicast Blocking in @ref Sn_MR_MACRAW mode
 * @details 0 : using IGMP version 2\n
 * 1 : using IGMP version 1\n
 * This bit is applied only during UDP mode(P[3:0] = 010 and MULTI = 
 * It configures the version for IGMP messages (Join/Leave/Report).
 */
const uint8_t Sn_MR_MMB = Sn_MR_ND;

/**
 * @brief IPv6 packet Blocking in @ref Sn_MR_MACRAW mode
 * @details 0 : disable IPv6 Blocking\n
 * 1 : enable IPv6 Blocking\n
 * This bit is applied only during MACRAW mode (P[3:0] = 100. It blocks to receiving the IPv6 packet.
 */
const uint8_t Sn_MR_MIP6B = Sn_MR_UCASTB;

/* Sn_MR value used with Sn_MR_UDP & Sn_MR_MULTI */
/**
 * @brief IGMP version used in UDP mulitcasting
 * @details 0 : disable Multicast Blocking\n
 * 1 : enable Multicast Blocking\n
 * This bit is applied only when MACRAW mode(P[3:0] = 100. It blocks to receive the packet with multicast MAC address.
 */
const uint8_t Sn_MR_MC = Sn_MR_ND;

/* Sn_MR alternate values */
/**
 * @brief For Berkeley Socket API
 */
const uint8_t SOCK_STREAM = Sn_MR_TCP;

/**
 * @brief For Berkeley Socket API
 */
const uint8_t SOCK_DGRAM = Sn_MR_UDP;


/* Sn_CR values */
/**
 * @brief Initialize or open socket
 * @details Socket n is initialized and opened according to the protocol selected in Sn_MR(P3:P0).
 * The table below shows the value of @ref Sn_SR corresponding to @ref Sn_MR.\n
 * <table>
 *   <tr>  <td>\b Sn_MR (P[3:0])</td> <td>\b Sn_SR</td>            		 </tr>
 *   <tr>  <td>Sn_MR_CLOSE  (000)</td> <td></td>         	   		 </tr>
 *   <tr>  <td>Sn_MR_TCP  (001)</td> <td>SOCK_INIT (0x13)</td>  		 </tr>
 *   <tr>  <td>Sn_MR_UDP  (010)</td>  <td>SOCK_UDP (0x22)</td>  		 </tr>
 *   <tr>  <td>S0_MR_MACRAW  (100)</td>  <td>SOCK_MACRAW (0x02)</td>  </tr>
 * </table>
 */
const uint8_t Sn_CR_OPEN = 0x01;

/**
 * @brief Wait connection request in TCP mode(Server mode)
 * @details This is valid only in TCP mode (\ref Sn_MR(P3:P0) = \ref Sn_MR_TCP).
 * In this mode, Socket n operates as a TCP serverand waits for  connection-request (SYN packet) from any TCP client
 * The @ref Sn_SR changes the state from \ref SOCK_INIT to \ref SOCKET_LISTEN.
 * When a TCP clientconnection request is successfully established,
 * the @ref Sn_SR changes from SOCK_LISTEN to SOCK_ESTABLISHED and the @ref Sn_IR(0) becomes 
 * But when a TCP clientconnection request is failed, @ref Sn_IR(3) becomes and the status of @ref Sn_SR changes to SOCK_CLOSED.
 */
const uint8_t Sn_CR_LISTEN = 0x02;

/**
 * @brief Send connection request in TCP mode(Client mode)
 * @details  To connect, a connect-request (SYN packet) is sent to <b>TCP server</b>configured by @ref Sn_DIPR & Sn_DPORT(destination address & port).
 * If the connect-request is successful, the @ref Sn_SR is changed to @ref SOCK_ESTABLISHED and the Sn_IR(0) becomes \n\n
 * The connect-request fails in the following three cases.\n
 * 1. When a @b ARPTO occurs (@ref Sn_IR[3] =  ) because destination hardware address is not acquired through the ARP-process.\n
 * 2. When a @b SYN/ACK packet is not received and @b TCPTO (Sn_IR(3) =  )\n
 * 3. When a @b RST packet is received instead of a @b SYN/ACK packet. In these cases, @ref Sn_SR is changed to @ref SOCK_CLOSED.
 * @note This is valid only in TCP mode and operates when Socket n acts as <b>TCP client</b>
 */
const uint8_t Sn_CR_CONNECT = 0x04;

/**
 * @brief Send closing request in TCP mode
 * @details Regardless of <b>TCP server</b>or <b>TCP client</b> the DISCON command processes the disconnect-process (b>Active close</b>or <b>Passive close</b>.\n
 * @par Active close
 * it transmits disconnect-request(FIN packet) to the connected peer\n
 * @par Passive close
 * When FIN packet is received from peer, a FIN packet is replied back to the peer.\n
 * @details When the disconnect-process is successful (that is, FIN/ACK packet is received successfully), @ref Sn_SR is changed to @ref SOCK_CLOSED.\n
 * Otherwise, TCPTO occurs (\ref Sn_IR(3)='1') and then @ref Sn_SR is changed to @ref SOCK_CLOSED.
 * @note Valid only in TCP mode.
 */
const uint8_t Sn_CR_DISCON = 0x08;

/**
 * @brief Close socket
 * @details Sn_SR is changed to @ref SOCK_CLOSED.
 */
const uint8_t Sn_CR_CLOSE = 0x10;

/**
 * @brief Update TX buffer pointer and send data
 * @details SEND transmits all the data in the Socket n TX buffer.\n
 * For more details, please refer to Socket n TX Free Size Register (@ref Sn_TX_FSR), Socket n,
 * TX Write Pointer Register(@ref Sn_TX_WR), and Socket n TX Read Pointer Register(@ref Sn_TX_RD).
 */
const uint8_t Sn_CR_SEND = 0x20;

/**
 * @brief Send data with MAC address, so without ARP process
 * @details The basic operation is same as SEND.\n
 * Normally SEND transmits data after destination hardware address is acquired by the automatic ARP-process(Address Resolution Protocol).\n
 * But SEND_MAC transmits data without the automatic ARP-process.\n
 * In this case, the destination hardware address is acquired from @ref Sn_DHAR configured by host, instead of APR-process.
 * @note Valid only in UDP mode.
 */
const uint8_t Sn_CR_SEND_MAC = 0x21;

/**
 * @brief Send keep alive message
 * @details It checks the connection status by sending 1byte keep-alive packet.\n
 * If the peer can not respond to the keep-alive packet during timeout time, the connection is terminated and the timeout interrupt will occur.
 * @note Valid only in TCP mode.
 */
const uint8_t Sn_CR_SEND_KEEP = 0x22;

/**
 * @brief Update RX buffer pointer and receive data
 * @details RECV completes the processing of the received data in Socket n RX Buffer by using a RX read pointer register (@ref Sn_RX_RD).\n
 * For more details, refer to Socket n RX Received Size Register (@ref Sn_RX_RSR), Socket n RX Write Pointer Register (@ref Sn_RX_WR),
 * and Socket n RX Read Pointer Register (@ref Sn_RX_RD).
 */
const uint8_t Sn_CR_RECV = 0x40;

/* Sn_IR values */
/**
 * @brief SEND_OK Interrupt
 * @details This is issued when SEND command is completed.
 */
const uint8_t Sn_IR_SENDOK = 0x10;

/**
 * @brief TIMEOUT Interrupt
 * @details This is issued when ARPTO or TCPTO occurs.
 */
const uint8_t Sn_IR_TIMEOUT = 0x08;

/**
 * @brief RECV Interrupt
 * @details This is issued whenever data is received from a peer.
 */
const uint8_t Sn_IR_RECV = 0x04;

/**
 * @brief DISCON Interrupt
 * @details This is issued when FIN or FIN/ACK packet is received from a peer.
 */
const uint8_t Sn_IR_DISCON = 0x02;

/**
 * @brief CON Interrupt
 * @details This is issued one time when the connection with peer is successful and then @ref Sn_SR is changed to @ref SOCK_ESTABLISHED.
 */
const uint8_t Sn_IR_CON = 0x01;

/* Sn_SR values */
/**
 * @brief Closed
 * @details This indicates that Socket n is released.\n
 * When DICON, CLOSE command is ordered, or when a timeout occurs, it is changed to @ref SOCK_CLOSED regardless of previous status.
 */
const uint8_t SOCK_CLOSED = 0x00;

/**
 * @brief Initiate state
 * @details This indicates Socket n is opened with TCP mode.\n
 * It is changed to @ref SOCK_INIT when @ref Sn_MR(P[3:0]) = 001 and OPEN command is ordered.\n
 * After @ref SOCK_INIT, user can use LISTEN /CONNECT command.
 */
const uint8_t SOCK_INIT = 0x13;

/**
 * @brief Listen state
 * @details This indicates Socket n is operating as <b>TCP server</b>mode and waiting for connection-request (SYN packet) from a peer <b>TCP client</b>.\n
 * It will change to @ref SOCK_ESTALBLISHED when the connection-request is successfully accepted.\n
 * Otherwise it will change to @ref SOCK_CLOSED after TCPTO @ref Sn_IR(TIMEOUT) = '1') is occurred.
 */
const uint8_t SOCK_LISTEN = 0x14;

/**
 * @brief Connection state
 * @details This indicates Socket n sent the connect-request packet (SYN packet) to a peer.\n
 * It is temporarily shown when @ref Sn_SR is changed from @ref SOCK_INIT to @ref SOCK_ESTABLISHED by CONNECT command.\n
 * If connect-accept(SYN/ACK packet) is received from the peer at SOCK_SYNSENT, it changes to @ref SOCK_ESTABLISHED.\n
 * Otherwise, it changes to @ref SOCK_CLOSED after TCPTO (@ref Sn_IR[TIMEOUT] = '1') is occurred.
 */
const uint8_t SOCK_SYNSENT = 0x15;

/**
 * @brief Connection state
 * @details It indicates Socket n successfully received the connect-request packet (SYN packet) from a peer.\n
 * If socket n sends the response (SYN/ACK  packet) to the peer successfully,  it changes to @ref SOCK_ESTABLISHED. \n
 * If not, it changes to @ref SOCK_CLOSED after timeout (@ref Sn_IR[TIMEOUT] = '1') is occurred.
 */
const uint8_t SOCK_SYNRECV = 0x16;

/**
 * @brief Success to connect
 * @details This indicates the status of the connection of Socket n.\n
 * It changes to @ref SOCK_ESTABLISHED when the <b>TCP SERVER</b>processed the SYN packet from the <b>TCP CLIENT</b>during @ref SOCK_LISTEN, or
 * when the CONNECT command is successful.\n
 * During @ref SOCK_ESTABLISHED, DATA packet can be transferred using SEND or RECV command.
 */
const uint8_t SOCK_ESTABLISHED = 0x17;

/**
 * @brief Closing state
 * @details These indicate Socket n is closing.\n
 * These are shown in disconnect-process such as active-close and passive-close.\n
 * When Disconnect-process is successfully completed, or when timeout occurs, these change to @ref SOCK_CLOSED.
 */
const uint8_t SOCK_FIN_WAIT = 0x18;

/**
 * @brief Closing state
 * @details These indicate Socket n is closing.\n
 * These are shown in disconnect-process such as active-close and passive-close.\n
 * When Disconnect-process is successfully completed, or when timeout occurs, these change to @ref SOCK_CLOSED.
 */
const uint8_t SOCK_CLOSING = 0x1A;

/**
 * @brief Closing state
 * @details These indicate Socket n is closing.\n
 * These are shown in disconnect-process such as active-close and passive-close.\n
 * When Disconnect-process is successfully completed, or when timeout occurs, these change to @ref SOCK_CLOSED.
 */
const uint8_t SOCK_TIME_WAIT = 0x1B;

/**
 * @brief Closing state
 * @details This indicates Socket n received the disconnect-request (FIN packet) from the connected peer.\n
 * This is half-closing status, and data can be transferred.\n
 * For full-closing, DISCON command is used. But For just-closing, CLOSE command is used.
 */
const uint8_t SOCK_CLOSE_WAIT = 0x1C;

/**
 * @brief Closing state
 * @details This indicates Socket n is waiting for the response (FIN/ACK packet) to the disconnect-request (FIN packet) by passive-close.\n
 * It changes to @ref SOCK_CLOSED when Socket n received the response successfully, or when timeout(@ref Sn_IR[TIMEOUT] = '1') is occurred.
 */
const uint8_t SOCK_LAST_ACK = 0x1D;

/**
 * @brief UDP socket
 * @details This indicates Socket n is opened in UDP mode(@ref Sn_MR(P[3:0]) = '010').\n
 * It changes to SOCK_UDP when @ref Sn_MR(P[3:0]) = '010' and @ref Sn_CR_OPEN command is ordered.\n
 * Unlike TCP mode, data can be transfered without the connection-process.
 */
const uint8_t SOCK_UDP = 0x22;

//#define SOCK_IPRAW                   0x32     /**< IP raw mode socket */

/**
 * @brief MAC raw mode socket
 * @details This indicates Socket 0 is opened in MACRAW mode (S0_MR(P[3:0]) = 100and is valid only in Socket 0.\n
 * It changes to SOCK_MACRAW when S0_MR(P[3:0] = 100and OPEN command is ordered.\n
 * Like UDP mode socket, MACRAW mode Socket 0 can transfer a MAC packet (Ethernet frame) without the connection-process.
 */
const uint8_t SOCK_MACRAW = 0x42;

//#define SOCK_PPPOE                   0x5F

/* IP PROTOCOL */
#define IPPROTO_IP                   0        //< Dummy for IP 
#define IPPROTO_ICMP                 1        //< Control message protocol
#define IPPROTO_IGMP                 2        //< Internet group management protocol
#define IPPROTO_GGP                  3        //< Gateway^2 (deprecated)
#define IPPROTO_TCP                  6        //< TCP
#define IPPROTO_PUP                  12       //< PUP
#define IPPROTO_UDP                  17       //< UDP
#define IPPROTO_IDP                  22       //< XNS idp
#define IPPROTO_ND                   77       //< UNOFFICIAL net disk protocol
#define IPPROTO_RAW                  255      //< Raw IP packet


/**
 * @brief Enter a critical section
 *
 * @details It is provided to protect your shared code which are executed without distribution. \n \n
 *
 * In non-OS environment, It can be just implemented by disabling whole interrupt.\n
 * In OS environment, You can replace it to critical section api supported by OS.
 *
 * \sa WIZCHIP_READ(), WIZCHIP_WRITE(), WIZCHIP_READ_BUF(), WIZCHIP_WRITE_BUF()
 * \sa WIZCHIP_CRITICAL_EXIT()
 */
static inline void WIZCHIP_CRITICAL_ENTER() {}

/**
 * @brief Exit a critical section
 *
 * @details It is provided to protect your shared code which are executed without distribution. \n\n
 *
 * In non-OS environment, It can be just implemented by disabling whole interrupt. \n
 * In OS environment, You can replace it to critical section api supported by OS.
 *
 * @sa WIZCHIP_READ(), WIZCHIP_WRITE(), WIZCHIP_READ_BUF(), WIZCHIP_WRITE_BUF()
 * @sa WIZCHIP_CRITICAL_ENTER()
 */
static inline void WIZCHIP_CRITICAL_EXIT() { }


////////////////////////
// Basic I/O Function //
////////////////////////

/**
 * @ingroup Basic_IO_function
 * @brief It reads 1 byte value from a register.
 * @param AddrSel Register address
 * @return The value of register
 */
uint8_t  WIZCHIP_READ (uint32_t AddrSel);

/**
 * @ingroup Basic_IO_function
 * @brief It writes 1 byte value to a register.
 * @param AddrSel Register address
 * @param wb Write data
 * @return void
 */
void     WIZCHIP_WRITE(uint32_t AddrSel, uint8_t wb );

/**
 * @ingroup Basic_IO_function
 * @brief It reads sequence data from registers.
 * @param AddrSel Register address
 * @param pBuf Pointer buffer to read data
 * @param len Data length
 */
void     WIZCHIP_READ_BUF (uint32_t AddrSel, uint8_t* pBuf, uint16_t len);

/**
 * @ingroup Basic_IO_function
 * @brief It writes sequence data to registers.
 * @param AddrSel Register address
 * @param pBuf Pointer buffer to write data
 * @param len Data length
 */
void     WIZCHIP_WRITE_BUF(uint32_t AddrSel, const uint8_t* pBuf, uint16_t len);

// Read into an IPAddress
void WIZCHIP_READ_IP(uint32_t AddrSel, IPAddress& ip);

// Write to an IPAddress
void WIZCHIP_WRITE_IP(uint32_t AddrSel, const IPAddress& ip);

/////////////////////////////////
// Common Register I/O function //
/////////////////////////////////
/**
 * @ingroup Common_register_access_function
 * @brief Set Mode Register
 * @param (uint8_t)mr The value to be set.
 * @sa getMR()
 */
static inline void setMR(uint8_t mr)
{
	WIZCHIP_WRITE(MR, mr);
}

/**
 * @ingroup Common_register_access_function
 * @brief Get Mode Register
 * @return uint8_t. The value of Mode register.
 * @sa setMR()
 */
static inline uint8_t getMR()
{
	return WIZCHIP_READ(MR);
}


/**
 * @ingroup Common_register_access_function
 * @brief Set gateway IP address
 * @param (uint8_t*)gar Pointer variable to set gateway IP address. It should be allocated 4 bytes.
 * @sa getGAR()
 */
static inline void setGAR(const IPAddress& gar)
{
	WIZCHIP_WRITE_IP(GAR, gar);
}

/**
 * @ingroup Common_register_access_function
 * @brief Get gateway IP address
 * @param (uint8_t*)gar Pointer variable to get gateway IP address. It should be allocated 4 bytes.
 * @sa setGAR()
 */
static inline void getGAR(IPAddress& gar)
{
	WIZCHIP_READ_IP(GAR, gar);
}

/**
 * @ingroup Common_register_access_function
 * @brief Set subnet mask address
 * @param (uint8_t*)subr Pointer variable to set subnet mask address. It should be allocated 4 bytes.
 * @sa getSUBR()
 */
static inline void setSUBR(const IPAddress& subr)
{
	WIZCHIP_WRITE_IP(SUBR, subr);
}

/**
 * @ingroup Common_register_access_function
 * @brief Get subnet mask address
 * @param (uint8_t*)subr Pointer variable to get subnet mask address. It should be allocated 4 bytes.
 * @sa setSUBR()
 */
static inline void getSUBR(IPAddress& subr)
{
	WIZCHIP_READ_IP(SUBR, subr);
}

/**
 * @ingroup Common_register_access_function
 * @brief Set local MAC address
 * @param (uint8_t*)shar Pointer variable to set local MAC address. It should be allocated 6 bytes.
 * @sa getSHAR()
 */
static inline void setSHAR(const uint8_t *shar)
{
	WIZCHIP_WRITE_BUF(SHAR, shar, 6);
}

/**
 * @ingroup Common_register_access_function
 * @brief Get local MAC address
 * @param (uint8_t*)shar Pointer variable to get local MAC address. It should be allocated 6 bytes.
 * @sa setSHAR()
 */
static inline void getSHAR(uint8_t *shar)
{
	WIZCHIP_READ_BUF(SHAR, shar, 6);
}

/**
 * @ingroup Common_register_access_function
 * @brief Set local IP address
 * @param (uint8_t*)sipr Pointer variable to set local IP address. It should be allocated 4 bytes.
 * @sa getSIPR()
 */
static inline void setSIPR(const IPAddress& sipr)
{
	WIZCHIP_WRITE_IP(SIPR, sipr);
}

/**
 * @ingroup Common_register_access_function
 * @brief Get local IP address
 * @param (uint8_t*)sipr Pointer variable to get local IP address. It should be allocated 4 bytes.
 * @sa setSIPR()
 */
static inline void getSIPR(IPAddress& sipr)
{
	WIZCHIP_READ_IP(SIPR, sipr);
}

/**
 * @ingroup Common_register_access_function
 * @brief Set INTLEVEL register
 * @param (uint16_t)intlevel Value to set @ref INTLEVEL register.
 * @sa getINTLEVEL()
 */
static inline void  setINTLEVEL(uint16_t intlevel)
{
	WIZCHIP_WRITE(INTLEVEL, (uint8_t)(intlevel >> 8));
	WIZCHIP_WRITE(WIZCHIP_OFFSET_INC(INTLEVEL,1), (uint8_t) intlevel);
}

/**
 * @ingroup Common_register_access_function
 * @brief Get INTLEVEL register
 * @return uint16_t. Value of @ref INTLEVEL register.
 * @sa setINTLEVEL()
 */
static inline uint16_t getINTLEVEL()
{
	const uint8_t msb = WIZCHIP_READ(INTLEVEL);
	return ((uint16_t)msb << 8) | WIZCHIP_READ(WIZCHIP_OFFSET_INC(INTLEVEL, 1));
}

/**
 * @ingroup Common_register_access_function
 * @brief Set @ref IR register
 * @param (uint8_t)ir Value to set @ref IR register.
 * @sa getIR()
 */
static inline void setIR(uint8_t ir)
{
	WIZCHIP_WRITE(IR, (ir & 0xF0));
}

/**
 * @ingroup Common_register_access_function
 * @brief Get @ref IR register
 * @return uint8_t. Value of @ref IR register.
 * @sa setIR()
 */
static inline uint8_t getIR()
{
	return WIZCHIP_READ(IR) & 0xF0;
}

/**
 * @ingroup Common_register_access_function
 * @brief Set @ref _IMR_ register
 * @param (uint8_t)imr Value to set @ref _IMR_ register.
 * @sa getIMR()
 */
static inline void setIMR(uint8_t imr)
{
	WIZCHIP_WRITE(_IMR_, imr);
}

/**
 * @ingroup Common_register_access_function
 * @brief Get @ref _IMR_ register
 * @return uint8_t. Value of @ref _IMR_ register.
 * @sa setIMR()
 */
static inline uint8_t getIMR()
{
	return WIZCHIP_READ(_IMR_);
}

/**
 * @ingroup Common_register_access_function
 * @brief Set @ref SIR register
 * @param (uint8_t)sir Value to set @ref SIR register.
 * @sa getSIR()
 */
static inline void setSIR(uint8_t sir)
{
	WIZCHIP_WRITE(SIR, sir);
}

/**
 * @ingroup Common_register_access_function
 * @brief Get @ref SIR register
 * @return uint8_t. Value of @ref SIR register.
 * @sa setSIR()
 */
static inline uint8_t getSIR()
{
	return WIZCHIP_READ(SIR);
}

/**
 * @ingroup Common_register_access_function
 * @brief Set @ref SIMR register
 * @param (uint8_t)simr Value to set @ref SIMR register.
 * @sa getSIMR()
 */
static inline void setSIMR(uint8_t simr)
{
	WIZCHIP_WRITE(SIMR, simr);
}

/**
 * @ingroup Common_register_access_function
 * @brief Get @ref SIMR register
 * @return uint8_t. Value of @ref SIMR register.
 * @sa setSIMR()
 */
static inline uint8_t getSIMR()
{
	return WIZCHIP_READ(SIMR);
}

/**
 * @ingroup Common_register_access_function
 * @brief Set @ref _RTR_ register
 * @param (uint16_t)rtr Value to set @ref _RTR_ register.
 * @sa getRTR()
 */
static inline void setRTR(uint16_t rtr)
{
	WIZCHIP_WRITE(_RTR_, (uint8_t)(rtr >> 8));
	WIZCHIP_WRITE(WIZCHIP_OFFSET_INC(_RTR_,1), (uint8_t) rtr);
}

/**
 * @ingroup Common_register_access_function
 * @brief Get @ref _RTR_ register
 * @return uint16_t. Value of @ref _RTR_ register.
 * @sa setRTR()
 */
static inline uint16_t getRTR()
{
	const uint8_t msb = WIZCHIP_READ(_RTR_);
	return ((uint16_t)msb << 8) + WIZCHIP_READ(WIZCHIP_OFFSET_INC(_RTR_, 1));
}


/**
 * @ingroup Common_register_access_function
 * @brief Set @ref _RCR_ register
 * @param (uint8_t)rcr Value to set @ref _RCR_ register.
 * @sa getRCR()
 */
static inline void setRCR(uint8_t rcr)
{
	WIZCHIP_WRITE(_RCR_, rcr);
}

/**
 * @ingroup Common_register_access_function
 * @brief Get @ref _RCR_ register
 * @return uint8_t. Value of @ref _RCR_ register.
 * @sa setRCR()
 */
static inline uint8_t getRCR()
{
	return WIZCHIP_READ(_RCR_);
}

//================================================== test done ===========================================================

/**
 * @ingroup Common_register_access_function
 * @brief Set @ref PTIMER register
 * @param (uint8_t)ptimer Value to set @ref PTIMER register.
 * @sa getPTIMER()
 */
static inline void setPTIMER(uint8_t ptimer)
{
	WIZCHIP_WRITE(PTIMER, ptimer);
}

/**
 * @ingroup Common_register_access_function
 * @brief Get @ref PTIMER register
 * @return uint8_t. Value of @ref PTIMER register.
 * @sa setPTIMER()
 */
static inline uint8_t getPTIMER()
{
	return WIZCHIP_READ(PTIMER);
}

/**
 * @ingroup Common_register_access_function
 * @brief Set @ref PMAGIC register
 * @param (uint8_t)pmagic Value to set @ref PMAGIC register.
 * @sa getPMAGIC()
 */
static inline void setPMAGIC(uint8_t pmagic)
{
	WIZCHIP_WRITE(PMAGIC, pmagic);
}

/**
 * @ingroup Common_register_access_function
 * @brief Get @ref PMAGIC register
 * @return uint8_t. Value of @ref PMAGIC register.
 * @sa setPMAGIC()
 */
static inline uint8_t getPMAGIC()
{
	return WIZCHIP_READ(PMAGIC);
}

/**
 * @ingroup Common_register_access_function
 * @brief Set @ref PHAR address
 * @param (uint8_t*)phar Pointer variable to set PPP destination MAC register address. It should be allocated 6 bytes.
 * @sa getPHAR()
 */
static inline void setPHAR(const uint8_t *phar)
{
	WIZCHIP_WRITE_BUF(PHAR, phar, 6);
}

/**
 * @ingroup Common_register_access_function
 * @brief Get @ref PHAR address
 * @param (uint8_t*)phar Pointer variable to PPP destination MAC register address. It should be allocated 6 bytes.
 * @sa setPHAR()
 */
static inline void getPHAR(uint8_t *phar)
{
	WIZCHIP_READ_BUF(PHAR, phar, 6);
}

/**
 * @ingroup Common_register_access_function
 * @brief Set @ref PSID register
 * @param (uint16_t)psid Value to set @ref PSID register.
 * @sa getPSID()
 */
static inline void setPSID(uint16_t psid)
{
	WIZCHIP_WRITE(PSID, (uint8_t)(psid >> 8));
	WIZCHIP_WRITE(WIZCHIP_OFFSET_INC(PSID,1), (uint8_t) psid);
}

/**
 * @ingroup Common_register_access_function
 * @brief Get @ref PSID register
 * @return uint16_t. Value of @ref PSID register.
 * @sa setPSID()
 */
static inline uint16_t getPSID()
{
	const uint8_t msb = WIZCHIP_READ(PSID);
	return ((uint16_t)msb << 8) | WIZCHIP_READ(WIZCHIP_OFFSET_INC(PSID,1));
}

/**
 * @ingroup Common_register_access_function
 * @brief Set @ref PMRU register
 * @param (uint16_t)pmru Value to set @ref PMRU register.
 * @sa getPMRU()
 */
static inline void setPMRU(uint16_t pmru)
{
	WIZCHIP_WRITE(PMRU, (uint8_t)(pmru>>8));
	WIZCHIP_WRITE(WIZCHIP_OFFSET_INC(PMRU,1), (uint8_t) pmru);
}

/**
 * @ingroup Common_register_access_function
 * @brief Get @ref PMRU register
 * @return uint16_t. Value of @ref PMRU register.
 * @sa setPMRU()
 */
static inline uint16_t getPMRU()
{
	const uint8_t msb = WIZCHIP_READ(PMRU);
	return ((uint16_t)msb << 8) + WIZCHIP_READ(WIZCHIP_OFFSET_INC(PMRU, 1));
}

/**
 * @ingroup Common_register_access_function
 * @brief Get unreachable IP address
 * @param (uint8_t*)uipr Pointer variable to get unreachable IP address. It should be allocated 4 bytes.
 */
static inline void getUIPR(uint8_t *uipr)
{
	WIZCHIP_READ_BUF(UIPR, uipr, 4);
}

/**
 * @ingroup Common_register_access_function
 * @brief Get @ref UPORTR register
 * @return uint16_t. Value of @ref UPORTR register.
 */
static inline uint16_t getUPORTR()
{
	const uint8_t msb = WIZCHIP_READ(UPORTR);
	return ((uint16_t)msb << 8) + WIZCHIP_READ(WIZCHIP_OFFSET_INC(UPORTR, 1));
}

/**
 * @ingroup Common_register_access_function
 * @brief Set @ref PHYCFGR register
 * @param (uint8_t)phycfgr Value to set @ref PHYCFGR register.
 * @sa getPHYCFGR()
 */
static inline void setPHYCFGR(uint8_t phycfgr)
{
	WIZCHIP_WRITE(PHYCFGR, phycfgr);
}

/**
 * @ingroup Common_register_access_function
 * @brief Get @ref PHYCFGR register
 * @return uint8_t. Value of @ref PHYCFGR register.
 * @sa setPHYCFGR()
 */
static inline uint8_t getPHYCFGR()
{
	return WIZCHIP_READ(PHYCFGR);
}

/**
 * @ingroup Common_register_access_function
 * @brief Get @ref VERSIONR register
 * @return uint8_t. Value of @ref VERSIONR register.
 */
static inline uint8_t getVERSIONR()
{
	return WIZCHIP_READ(VERSIONR);
}

/////////////////////////////////////

///////////////////////////////////
// Socket N register I/O function //
///////////////////////////////////
/**
 * @ingroup Socket_register_access_function
 * @brief Set @ref Sn_MR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint8_t)mr Value to set @ref Sn_MR
 * @sa getSn_MR()
 */
static inline void setSn_MR(uint8_t sn, uint8_t mr)
{
	WIZCHIP_WRITE(Sn_MR(sn), mr);
}

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_MR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint8_t. Value of @ref Sn_MR.
 * @sa setSn_MR()
 */
static inline uint8_t getSn_MR(uint8_t sn)
{
	return WIZCHIP_READ(Sn_MR(sn));
}

/**
 * @ingroup Socket_register_access_function
 * @brief Set @ref Sn_CR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint8_t)cr Value to set @ref Sn_CR
 * @sa getSn_CR()
 */
static inline void setSn_CR(uint8_t sn, uint8_t cr)
{
	WIZCHIP_WRITE(Sn_CR(sn), cr);
}

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_CR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint8_t. Value of @ref Sn_CR.
 * @sa setSn_CR()
 */
static inline uint8_t getSn_CR(uint8_t sn)
{
	return WIZCHIP_READ(Sn_CR(sn));
}

/**
 * @ingroup Socket_register_access_function
 * @brief Set @ref Sn_IR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint8_t)ir Value to set @ref Sn_IR
 * @sa getSn_IR()
 */
static inline void setSn_IR(uint8_t sn, uint8_t ir)
{
	WIZCHIP_WRITE(Sn_IR(sn), (ir & 0x1F));
}

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_IR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint8_t. Value of @ref Sn_IR.
 * @sa setSn_IR()
 */
static inline uint8_t getSn_IR(uint8_t sn)
{
	return WIZCHIP_READ(Sn_IR(sn)) & 0x1F;
}

/**
 * @ingroup Socket_register_access_function
 * @brief Set @ref Sn_IMR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint8_t)imr Value to set @ref Sn_IMR
 * @sa getSn_IMR()
 */
static inline void setSn_IMR(uint8_t sn, uint8_t imr)
{
	WIZCHIP_WRITE(Sn_IMR(sn), (imr & 0x1F));
}

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_IMR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint8_t. Value of @ref Sn_IMR.
 * @sa setSn_IMR()
 */
static inline uint8_t getSn_IMR(uint8_t sn)
{
	 return WIZCHIP_READ(Sn_IMR(sn)) & 0x1F;
}

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_SR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint8_t. Value of @ref Sn_SR.
 */
static inline uint8_t getSn_SR(uint8_t sn)
{
	return WIZCHIP_READ(Sn_SR(sn));
}

/**
 * @ingroup Socket_register_access_function
 * @brief Set @ref Sn_PORT register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint16_t)port Value to set @ref Sn_PORT.
 * @sa getSn_PORT()
 */
static inline void setSn_PORT(uint8_t sn, uint16_t port)
{
	WIZCHIP_WRITE(Sn_PORT(sn), (uint8_t)(port >> 8));
	WIZCHIP_WRITE(WIZCHIP_OFFSET_INC(Sn_PORT(sn), 1), (uint8_t) port);
}

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_PORT register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint16_t. Value of @ref Sn_PORT.
 * @sa setSn_PORT()
 */
static inline uint16_t getSn_PORT(uint8_t sn)
{
	const uint8_t msb = WIZCHIP_READ(Sn_PORT(sn));
	return ((uint16_t)msb << 8) + WIZCHIP_READ(WIZCHIP_OFFSET_INC(Sn_PORT(sn), 1));
}

/**
 * @ingroup Socket_register_access_function
 * @brief Set @ref Sn_DHAR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint8_t*)dhar Pointer variable to set socket n destination hardware address. It should be allocated 6 bytes.
 * @sa getSn_DHAR()
 */
static inline void setSn_DHAR(uint8_t sn, const uint8_t *dhar)
{
	WIZCHIP_WRITE_BUF(Sn_DHAR(sn), dhar, 6);
}

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_MR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint8_t*)dhar Pointer variable to get socket n destination hardware address. It should be allocated 6 bytes.
 * @sa setSn_DHAR()
 */
static inline void getSn_DHAR(uint8_t sn, uint8_t *dhar)
{
	WIZCHIP_READ_BUF(Sn_DHAR(sn), dhar, 6);
}

/**
 * @ingroup Socket_register_access_function
 * @brief Set @ref Sn_DIPR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint8_t*)dipr Pointer variable to set socket n destination IP address. It should be allocated 4 bytes.
 * @sa getSn_DIPR()
 */
static inline void setSn_DIPR(uint8_t sn, const IPAddress& dipr)
{
	WIZCHIP_WRITE_IP(Sn_DIPR(sn), dipr);
}

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_DIPR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint8_t*)dipr Pointer variable to get socket n destination IP address. It should be allocated 4 bytes.
 * @sa setSn_DIPR()
 */
static inline void getSn_DIPR(uint8_t sn, IPAddress& dipr)
{
	WIZCHIP_READ_IP(Sn_DIPR(sn), dipr);
}

/**
 * @ingroup Socket_register_access_function
 * @brief Set @ref Sn_DPORT register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint16_t)dport Value to set @ref Sn_DPORT
 * @sa getSn_DPORT()
 */
static inline void setSn_DPORT(uint8_t sn, uint16_t dport)
{
	WIZCHIP_WRITE(Sn_DPORT(sn), (uint8_t) (dport >> 8));
	WIZCHIP_WRITE(WIZCHIP_OFFSET_INC(Sn_DPORT(sn), 1), (uint8_t)  dport);
}

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_DPORT register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint16_t. Value of @ref Sn_DPORT.
 * @sa setSn_DPORT()
 */
static inline uint16_t getSn_DPORT(uint8_t sn)
{
	const uint8_t msb = WIZCHIP_READ(Sn_DPORT(sn));
	return ((uint16_t)msb << 8) + WIZCHIP_READ(WIZCHIP_OFFSET_INC(Sn_DPORT(sn),1));
}

/**
 * @ingroup Socket_register_access_function
 * @brief Set @ref Sn_MSSR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint16_t)mss Value to set @ref Sn_MSSR
 * @sa setSn_MSSR()
 */
static inline void setSn_MSSR(uint8_t sn, uint16_t mss)
{
	WIZCHIP_WRITE(Sn_MSSR(sn), (uint8_t)(mss >> 8));
	WIZCHIP_WRITE(WIZCHIP_OFFSET_INC(Sn_MSSR(sn), 1), (uint8_t) mss);
}

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_MSSR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint16_t. Value of @ref Sn_MSSR.
 * @sa setSn_MSSR()
 */
static inline uint16_t getSn_MSSR(uint8_t sn)
{
	const uint8_t msb = WIZCHIP_READ(Sn_MSSR(sn));
	return ((uint16_t)msb << 8) + WIZCHIP_READ(WIZCHIP_OFFSET_INC(Sn_MSSR(sn), 1));
}

/**
 * @ingroup Socket_register_access_function
 * @brief Set @ref Sn_TOS register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint8_t)tos Value to set @ref Sn_TOS
 * @sa getSn_TOS()
 */
static inline void setSn_TOS(uint8_t sn, uint8_t tos)
{
	WIZCHIP_WRITE(Sn_TOS(sn), tos);
}

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_TOS register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint8_t. Value of Sn_TOS.
 * @sa setSn_TOS()
 */
static inline uint8_t getSn_TOS(uint8_t sn)
{
	return WIZCHIP_READ(Sn_TOS(sn));
}

/**
 * @ingroup Socket_register_access_function
 * @brief Set @ref Sn_TTL register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint8_t)ttl Value to set @ref Sn_TTL
 * @sa getSn_TTL()
 */
static inline void setSn_TTL(uint8_t sn, uint8_t ttl)
{
	WIZCHIP_WRITE(Sn_TTL(sn), ttl);
}


/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_TTL register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint8_t. Value of @ref Sn_TTL.
 * @sa setSn_TTL()
 */
static inline uint8_t getSn_TTL(uint8_t sn)
{
	return WIZCHIP_READ(Sn_TTL(sn));
}


/**
 * @ingroup Socket_register_access_function
 * @brief Set @ref Sn_RXBUF_SIZE register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint8_t)rxbufsize Value to set @ref Sn_RXBUF_SIZE
 * @sa getSn_RXBUF_SIZE()
 */
static inline void setSn_RXBUF_SIZE(uint8_t sn, uint8_t rxbufsize)
{
	WIZCHIP_WRITE(Sn_RXBUF_SIZE(sn),rxbufsize);
}


/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_RXBUF_SIZE register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint8_t. Value of @ref Sn_RXBUF_SIZE.
 * @sa setSn_RXBUF_SIZE()
 */
static inline uint8_t getSn_RXBUF_SIZE(uint8_t sn)
{
	return WIZCHIP_READ(Sn_RXBUF_SIZE(sn));
}

/**
 * @ingroup Socket_register_access_function
 * @brief Set @ref Sn_TXBUF_SIZE register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint8_t)txbufsize Value to set @ref Sn_TXBUF_SIZE
 * @sa getSn_TXBUF_SIZE()
 */
static inline void setSn_TXBUF_SIZE(uint8_t sn, uint8_t txbufsize)
{
	WIZCHIP_WRITE(Sn_TXBUF_SIZE(sn), txbufsize);
}

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_TXBUF_SIZE register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint8_t. Value of @ref Sn_TXBUF_SIZE.
 * @sa setSn_TXBUF_SIZE()
 */
static inline uint8_t getSn_TXBUF_SIZE(uint8_t sn)
{
	return WIZCHIP_READ(Sn_TXBUF_SIZE(sn));
}

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_TX_FSR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint16_t. Value of @ref Sn_TX_FSR.
 */
uint16_t getSn_TX_FSR(uint8_t sn);

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_TX_RD register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint16_t. Value of @ref Sn_TX_RD.
 */
static inline uint16_t getSn_TX_RD(uint8_t sn)
{
	const uint8_t msb = WIZCHIP_READ(Sn_TX_RD(sn));
	return ((uint16_t)msb << 8) + WIZCHIP_READ(WIZCHIP_OFFSET_INC(Sn_TX_RD(sn),1));
}

/**
 * @ingroup Socket_register_access_function
 * @brief Set @ref Sn_TX_WR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint16_t)txwr Value to set @ref Sn_TX_WR
 * @sa GetSn_TX_WR()
 */
static inline void setSn_TX_WR(uint8_t sn, uint16_t txwr)
{
	WIZCHIP_WRITE(Sn_TX_WR(sn), (uint8_t)(txwr >> 8));
	WIZCHIP_WRITE(WIZCHIP_OFFSET_INC(Sn_TX_WR(sn), 1), (uint8_t) txwr);
}

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_TX_WR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint16_t. Value of @ref Sn_TX_WR.
 * @sa setSn_TX_WR()
 */
static inline uint16_t getSn_TX_WR(uint8_t sn)
{
	const uint8_t msb = WIZCHIP_READ(Sn_TX_WR(sn));
	return ((uint16_t)msb << 8) + WIZCHIP_READ(WIZCHIP_OFFSET_INC(Sn_TX_WR(sn),1));
}


/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_RX_RSR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint16_t. Value of @ref Sn_RX_RSR.
 */
uint16_t getSn_RX_RSR(uint8_t sn);


/**
 * @ingroup Socket_register_access_function
 * @brief Set @ref Sn_RX_RD register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint16_t)rxrd Value to set @ref Sn_RX_RD
 * @sa getSn_RX_RD()
 */
static inline void setSn_RX_RD(uint8_t sn, uint16_t rxrd)
{
	WIZCHIP_WRITE(Sn_RX_RD(sn), (uint8_t)(rxrd >> 8));
	WIZCHIP_WRITE(WIZCHIP_OFFSET_INC(Sn_RX_RD(sn),1), (uint8_t) rxrd);
}

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_RX_RD register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint16_t. Value of @ref Sn_RX_RD.
 * @sa setSn_RX_RD()
 */
static inline uint16_t getSn_RX_RD(uint8_t sn)
{
	const uint8_t msb = WIZCHIP_READ(Sn_RX_RD(sn));
	return ((uint16_t)msb << 8) + WIZCHIP_READ(WIZCHIP_OFFSET_INC(Sn_RX_RD(sn), 1));
}

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_RX_WR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint16_t. Value of @ref Sn_RX_WR.
 */
static inline uint16_t getSn_RX_WR(uint8_t sn)
{
	const uint8_t msb = WIZCHIP_READ(Sn_RX_WR(sn));
	return ((uint16_t)msb << 8) | WIZCHIP_READ(WIZCHIP_OFFSET_INC(Sn_RX_WR(sn), 1));
}

/**
 * @ingroup Socket_register_access_function
 * @brief Set @ref Sn_FRAG register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint16_t)frag Value to set @ref Sn_FRAG
 * @sa getSn_FRAD()
 */
static inline void setSn_FRAG(uint8_t sn, uint16_t frag)
{
	WIZCHIP_WRITE(Sn_FRAG(sn),  (uint8_t)(frag >> 8));
	WIZCHIP_WRITE(WIZCHIP_OFFSET_INC(Sn_FRAG(sn),1), (uint8_t) frag);
}

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_FRAG register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint16_t. Value of @ref Sn_FRAG.
 * @sa setSn_FRAG()
 */
static inline uint16_t getSn_FRAG(uint8_t sn)
{
    const uint8_t msb = WIZCHIP_READ(Sn_FRAG(sn));
    return ((uint16_t)msb << 8) | WIZCHIP_READ(WIZCHIP_OFFSET_INC(Sn_FRAG(sn),1));
}

/**
 * @ingroup Socket_register_access_function
 * @brief Set @ref Sn_KPALVTR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param (uint8_t)kpalvt Value to set @ref Sn_KPALVTR
 * @sa getSn_KPALVTR()
 */
static inline void setSn_KPALVTR(uint8_t sn, uint8_t kpalvt)
{
	WIZCHIP_WRITE(Sn_KPALVTR(sn), kpalvt);
}

/**
 * @ingroup Socket_register_access_function
 * @brief Get @ref Sn_KPALVTR register
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint8_t. Value of @ref Sn_KPALVTR.
 * @sa setSn_KPALVTR()
 */
static inline uint8_t getSn_KPALVTR(uint8_t sn)
{
	return WIZCHIP_READ(Sn_KPALVTR(sn));
}

//////////////////////////////////////

/////////////////////////////////////
// Sn_TXBUF & Sn_RXBUF IO function //
/////////////////////////////////////
/**  
 * @brief Socket_register_access_function
 * @brief Gets the max buffer size of socket sn passed as parameter.
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint16_t. Value of Socket n RX max buffer size.
 */
static inline uint16_t getSn_RxMAX(uint8_t sn)
{
	return ((uint16_t)getSn_RXBUF_SIZE(sn)) << 10;
}

/**  
 * @brief Socket_register_access_function
 * @brief Gets the max buffer size of socket sn passed as parameters.
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @return uint16_t. Value of Socket n TX max buffer size.
 */
static inline uint16_t getSn_TxMAX(uint8_t sn)
{
	return ((uint16_t)getSn_TXBUF_SIZE(sn)) << 10;
}

/**
 * @ingroup Basic_IO_function
 * @brief It copies data to internal TX memory
 *
 * @details This function reads the Tx write pointer register and after that,
 * it copies the <i>wizdata(pointer buffer)</i> of the length of <i>len(variable)</i> bytes to internal TX memory
 * and updates the Tx write pointer register.
 * This function is being called by send() and sendto() function also.
 *
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param wizdata Pointer buffer to write data
 * @param len Data length
 * @sa wiz_recv_data()
 */
void wiz_send_data(uint8_t sn, const uint8_t *wizdata, uint16_t len);

// Alternative to wiz_send_data to work around an apparent bug
void wiz_send_data_at(uint8_t sn, const uint8_t *wizdata, uint16_t len, uint16_t ptr);

/**
 * @ingroup Basic_IO_function
 * @brief It copies data to your buffer from internal RX memory
 *
 * @details This function read the Rx read pointer register and after that,
 * it copies the received data from internal RX memory
 * to <i>wizdata(pointer variable)</i> of the length of <i>len(variable)</i> bytes.
 * This function is being called by recv() also.
 *
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param wizdata Pointer buffer to read data
 * @param len Data length
 * @sa wiz_send_data()
 */
void wiz_recv_data(uint8_t sn, uint8_t *wizdata, uint16_t len);

// Alternative to wiz_recv_data to work around an apparent bug
void wiz_recv_data_at(uint8_t sn, uint8_t *wizdata, uint16_t len, uint16_t ptr);

/**
 * @ingroup Basic_IO_function
 * @brief It discard the received data in RX memory.
 * @details It discards the data of the length of <i>len(variable)</i> bytes in internal RX memory.
 * @param (uint8_t)sn Socket number. It should be <b>0 ~ 7</b>.
 * @param len Data length
 */
void wiz_recv_ignore(uint8_t sn, uint16_t len);

#endif   // _W5500_H_
