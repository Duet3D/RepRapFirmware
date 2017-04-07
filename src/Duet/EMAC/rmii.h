/**
 * \file
 *
 * \brief API driver for KSZ8051RNL PHY based on driver for DM9161A PHY PHY.
 * Adapted from the mii.h file provided as part of LWIP under the license below:
 * Only designed to work with the KSZ8051RNL PHY, not yet fully tested.
 *
 * Copyright (c) 2012 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#ifndef RMII_H_INCLUDED
#define RMII_H_INCLUDED

/** \addtogroup eth_phy_rmii
  @{*/

/** \addtogroup rmii_registers PHY registers Addresses
    @{*/
#define MII_BMCR        0x0 /**< Basic Mode Control Register */
#define MII_BMSR        0x1 /**< Basic Mode Status Register */
#define MII_PHYID1      0x2 /**< PHY Identifier Register 1 */
#define MII_PHYID2      0x3 /**< PHY Identifier Register 2 */
#define MII_ANAR        0x4 /**< Auto_negotiation Advertisement Register */
#define MII_ANLPAR      0x5 /**< Auto_negotiation Link Partner Ability Register */
#define MII_ANER        0x6 /**< Auto-negotiation Expansion Register */
#define MII_ANNP        0x7 /**< Auto-Negotiation Next Page */
#define MII_LPNP        0x8 /**< Link Partner Next Page Ability */
#define MII_AFE        0x11 /**< AFE Control 1*/
#define MII_RXER       0x15 /**< RXER Counter*/
#define MII_OMSO       0x16 /**< Operation Mode Strap Override */
#define MII_OMSS       0x17 /**< Operation Mode Strap Status */
#define MII_EXPC       0x18 /**< Expanded Control */
#define MII_INTCS      0x1B /**< Interrupt Control/Status */
#define MII_LMDCS      0x1D /**< LinkMD Control/Status */
#define MII_PC1        0x1E /**< PHY Control 1*/
#define MII_PC2        0x1F /**< PHY Control 2*/
/** @}*/

/** \addtogroup phy_bmcr Basic Mode Control Register (BMCR, 0)
    List Bit definitions: \ref MII_BMCR
    @{*/
#define MII_RESET             (1u << 15) /**< 1= Software Reset; 0=Normal Operation */
#define MII_LOOPBACK          (1u << 14) /**< 1=loopback Enabled; 0=Normal Operation */
#define MII_SPEED_SELECT      (1u << 13) /**< 1=100Mbps; 0=10Mbps */
#define MII_AUTONEG           (1u << 12) /**< Auto-negotiation Enable */
#define MII_POWER_DOWN        (1u << 11) /**< 1=Power down 0=Normal operation */
#define MII_ISOLATE           (1u << 10) /**< 1 = Isolate 0 = Normal operation */
#define MII_RESTART_AUTONEG   (1u << 9)  /**< 1 = Restart auto-negotiation 0 = Normal operation */
#define MII_DUPLEX_MODE       (1u << 8)  /**< 1 = Full duplex operation 0 = Normal operation */
#define MII_COLLISION_TEST    (1u << 7)  /**< 1 = Collision test enabled 0 = Normal operation */
/** Reserved bits: 6 to 0, Read as 0, ignore on write */
/** @}*/

/** \addtogroup phy_bmsr Basic Mode Status Register (BMSR, 1)
    List Bit definitions: \ref MII_BMSR
    @{*/
#define MII_100BASE_T4        (1u << 15) /**< 100BASE-T4 Capable */
#define MII_100BASE_TX_FD     (1u << 14) /**< 100BASE-TX Full Duplex Capable */
#define MII_100BASE_TX_HD     (1u << 13) /**< 100BASE-TX Half Duplex Capable */
#define MII_10BASE_T_FD       (1u << 12) /**< 10BASE-T Full Duplex Capable */
#define MII_10BASE_T_HD       (1u << 11) /**< 10BASE-T Half Duplex Capable */
/** Reserved bits: 10 to 7, Read as 0, ignore on write */
#define MII_MF_PREAMB_SUPPR   (1u << 6) /**< MII Frame Preamble Suppression */
#define MII_AUTONEG_COMP      (1u << 5) /**< Auto-negotiation is completed */
#define MII_REMOTE_FAULT      (1u << 4) /**< Remote Fault */
#define MII_AUTONEG_ABILITY   (1u << 3) /**< Auto Configuration Ability */
#define MII_LINK_STATUS       (1u << 2) /**< Link Status */
#define MII_JABBER_DETECT     (1u << 1) /**< Jabber Detect */
#define MII_EXTEND_CAPAB      (1u << 0) /**< Extended Capability */
/** @}*/

/** \addtogroup phy_id PHY ID Identifier Register (PHYID, 2,3)
    List definitions: \ref MII_PHYID1, \ref MII_PHYID2
    @{*/
/**<Assigned to the 3rd through 18th bits of the
Organizationally Unique Identifier (OUI).
Kendin Communication’s OUI is 0010A1 (hex)*/
#define MII_OUI1            0x22
 /**<Assigned to the 19th through 24th bits of the
Organizationally Unique Identifier (OUI).
Kendin Communication’s OUI is 0010A1 (hex)*/
#define MII_OUI2            0x5
#define MII_MODELN            0x15  /**<Six bit manufacturer’s model number*/
/** @}*/

/** \addtogroup phy_neg Auto-negotiation (ANAR, 4; ANLPAR, 5; ANER, 6)
    - Auto-negotiation Advertisement Register (ANAR)
    - Auto-negotiation Link Partner Ability Register (ANLPAR)
	- Auto-negotiation Expansion Register (ANER)
    List Bit definitions: \ref MII_ANAR, \ref MII_ANLPAR \ref MII_ANER
    @{*/
#define MII_NP               (1u << 15) /**< Next page Indication */
#define MII_ACK              (1u << 14) /**< Acknowledge from Link Partner ability register (0x5)*/
#define MII_RF               (1u << 13) /**< Remote Fault */
#define MII_NO_PAUSE         0x0 /**<  [00] = No PAUSE*/
#define MII_A_PAUSE          0x2 /**<  [10] = Asymmetric PAUSE*/
#define MII_S_PAUSE          0x1 /**<  [01] = Symmetric PAUSE[11] = Asymmetric & Symmetric PAUSE */
#define MII_AS_PAUSE         0x3 /**<  [11] = Asymmetric & Symmetric PAUSE */
#define MII_T4               (1u << 9)  /**< 100BASE-T4 Support */
#define MII_TX_FDX           (1u << 8)  /**< 100BASE-TX Full Duplex Support */
#define MII_TX_HDX           (1u << 7)  /**< 100BASE-TX Support */
#define MII_10_FDX           (1u << 6)  /**< 10BASE-T Full Duplex Support */
#define MII_10_HDX           (1u << 5)  /**< 10BASE-T Support */
#define MII_PDF              (1u << 4) /**< Local Device Parallel Detection Fault */
#define MII_LP_NP_ABLE       (1u << 3) /**< Link Partner Next Page Able */
#define MII_NP_ABLE          (1u << 2) /**< Local Device Next Page Able */
#define MII_PAGE_RX          (1u << 1) /**< New Page Received */
#define MII_LP_AN_ABLE       (1u << 0) /**< Link Partner Auto-negotiation Able */
/** Selector: 4 to 0, Protocol Selection Bits */
#define MII_AN_IEEE_802_3      0x1
/** @}*/

/** \addtogroup phy_neg_exp Auto-Negotiation Next Page (ANNP, 7, LPNP,8)
    List Bit definitions: \ref MII_ANNP
    List Bit definitions: \ref MII_LPNP
    @{*/
#define MII_ANMP              (1u << 13) /**< Message Page */
#define MII_ACK2              (1u << 12) /**< Acknowledge2 */
#define MII_TOGGLE            (1u << 11) /**< Toggle */
#define MII_ANNP_MSG          0x001 /**< 11-bit wide field to encode 2048 messages */
/** @}*/

/** \addtogroup phy_neg_exp Auto-Negotiation Next Page (AFE, 11)
    List Bit definitions: \ref MII_AFE
    @{*/
#define MII_SOME              (1u << 5) /**< Slow Oscillator Mode Enable */
/** @}*/

/** \addtogroup phy_neg_exp Auto-Negotiation Next Page (RXER, 15)
    List Bit definitions: \ref MII_RXER
    @{*/
#define MII_RXERC             0x0000 /**< receive error counter for Symbol Error frames */
/** @}*/

/** \addtogroup phy_omso Specified Configuration Register (OMSO, 16)
    List Bit definitions: \ref MII_OMSO
    @{*/
#define MII_BCO              (1u << 9)  /**< B-CAST_OFF override */
#define MII_MB2BO            (1u << 7)  /**< MII B-to-B override */
#define MII_RB2BO            (1u << 6)  /**< RMII B-to-B override */
#define MII_NTO              (1u << 5)  /**< NAND Tree override*/
#define MII_RMIIO            (1u << 1)  /**< RMII override */
#define MII_MIIO            (1u << 0)  /**< MII override */
/** @}*/

/** \addtogroup phy_omss Specified Configuration and Status Register (OMSS, 17)
    List Bit definitions: \ref MII_OMSS
    @{*/
#define MII_PHYADD2           (1u << 15) /**< PHYAD[2] strap-in status */
#define MII_PHYADD1           (1u << 14) /**< PHYAD[1] strap-in status  */
#define MII_PHYADD0           (1u << 13) /**< PHYAD[0] strap-in status */
#define MII_BCOSS             (1u << 9) /**< B-CAST_OFF strap-in status */
#define MII_MB2BSS            (1u << 7) /**< MII B-to-B strap-in status */
#define MII_RB2BSS            (1u << 6) /**< RMII B-to-B strap-in status */
#define MII_NTSS              (1u << 5) /**< NAND Tree strap-in status */
#define MII_RMIISS            (1u << 1) /**< RMII strap-in status */
#define MII_MIISS             (1u << 0) /**< MII strap-in status */
/** @}*/


/** \addtogroup phy_expc Expanded Control (EXPC, 18)
    List Bit definitions: \ref MII_EXPC
    @{*/
#define MII_EDPDD            (1u << 11) /**< EDPD Disabled */
#define MII_100PR            (1u << 10) /**< 100Base-TX Preamble Restore */
#define MII_10PR             (1u << 6) /**< 100Base-TX Preamble Restore */
/** @}*/

/** \addtogroup phy_intcs Interrupt Control/Status (INTCS, 1B)
    List Bit definitions: \ref MII_INTCS
    @{*/
#define MII_JIE              (1u << 15) /**< Jabber Interrupt Enable*/
#define MII_REIE             (1u << 14) /**< Receive Error Interrupt Enable */
#define MII_PRIE             (1u << 13) /**< Page Received Interrupt Enable */
#define MII_PDFIE            (1u << 12) /**< Parallel Detect Fault Interrupt Enable */
#define MII_LPAIE            (1u << 11) /**< Link Partner Ack Interrupt Enable */
#define MII_LDIE             (1u << 10) /**< Link Down Interrupt Enable */
#define MII_RFIE             (1u << 9)  /**< Remote Fault Interrupt Enable */
#define MII_LUIE             (1u << 8)  /**< Link Up Interrupt Enable */
#define MII_JABI             (1u << 7)  /**< Jabber Interrupt*/
#define MII_REI              (1u << 6)  /**< Receive Error Interrupt*/
#define MII_PRI              (1u << 5)  /**< Page Received Interrupt*/
#define MII_PDFI             (1u << 4) /**< Parallel Detect Fault Interrupt */
#define MII_LPAI             (1u << 3) /**< Link Partner Ack Interrupt  */
#define MII_LDI              (1u << 2) /**< Link Down Interrupt*/
#define MII_RFI              (1u << 1)  /**< Remote Fault Interrupt */
#define MII_LUI              (1u << 0)  /**< Link Up Interrupt */
/** @}*/

/** \addtogroup phy_lmdcs LinkMD Control/Status (LMDCS, 1D)
    List Bit definitions: \ref MII_LMDCS
    @{*/
#define MII_CDTE             (1u << 15) /**< Cable Diagnostic Test Enable*/
#define MII_CDTR1            (1u << 14)  /**< Cable Diagnostic Test Result BIT 1 */
#define MII_CDTR0            (1u << 14) /**< Cable Diagnostic Test Result BIT 0 */
#define MII_SCI              (1u << 12) /**< Short Cable Indicator */
#define MII_CFC              (1u << 0)  /**< Cable Fault Counter */
/** @}*/

/** \addtogroup phy_pc PHY Control 1 Register (PC1, 1E)
    List Bit definitions: \ref MII_PC1
    @{*/
#define MII_ENPFC            (1u << 9)  /**< Enable Pause(Flow Control)*/
#define MII_LNKS             (1u << 8)  /**< Link Status */
#define MII_POLS             (1u << 7)  /**< Polarity Status*/
#define MII_MDIXS            (1u << 5)  /**< MDI/MDI-X State*/
#define MII_ENERGYD          (1u << 4) /**< Energy Detect */
#define MII_PHYISO           (1u << 3) /**< PHY Isolate  */
#define MII_OMI_10_HD		 0x1  /**< [001] = 10Base-T half-duplex*/
#define MII_OMI_100_HD		 0x2  /**< [010] = 100Base-TX half-duplex*/
#define MII_OMI_10_FD		 0x5  /**< [101] = 10Base-T full-duplex*/
#define MII_OMI_100_FD		 0x6  /**< [110] = 100Base-TX full-duplex*/

/** @}*/

/** \addtogroup phy_pc PHY Control 2 Register (PC2, 1F)
    List Bit definitions: \ref MII_PC2
    @{*/
#define MII_HP_MDIX          (1u << 15) /**< HP_MDIX*/
#define MII_MDIX_SELECT      (1u << 14) /**< MDI/MDI-X Select */
#define MII_PSD              (1u << 13)  /**< Pair Swap Disable */
#define MII_FORCE_LINK       (1u << 11) /**< Force Link */
#define MII_POWERSAVE        (1u << 10) /**< Power Saving */
#define MII_INT_LEVEL        (1u << 9)  /**< Interrupt Level*/
#define MII_ENJAB            (1u << 8)  /**< Enable Jabber */
#define MII_RMII_REF_CLK     (1u << 7)  /**< RMII Reference Clock Select*/
#define MII_LED1             (1u << 5)  /**< LED mode BIT 1*/
#define MII_LED0             (1u << 4)  /**< LED mode BIT 0*/
#define MII_TXDIS            (1u << 3) /**< Disable Transmitter  */
#define MII_RMLB             (1u << 2) /**< Remote Loop-back*/
#define MII_SQET             (1u << 1) /**< Enable SQE Test*/
#define MII_DDS              (1u << 0) /**< Disable Data Scrambling*/
/** @}*/

/**@}*/
#endif /* RMII_H_INCLUDED */
