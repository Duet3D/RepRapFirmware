/**
 * \file
 *
 * \brief SAM Control Area Network (MCAN) Low Level Driver
 *
 * Copyright (c) 2015-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */


#ifndef SRC_HARDWARE_CANDRIVER_H_
#define SRC_HARDWARE_CANDRIVER_H_

#include <RepRapFirmware.h>

#if SUPPORT_CAN_EXPANSION

#include <compiler.h>
#include <status_codes.h>

/** The value should be 8/12/16/20/24/32/48/64. */
#define CONF_MCAN_ELEMENT_DATA_SIZE         64

/* -------- MCAN_RX_ELEMENT_R0 : (MCAN RX element: 0x00) (R/W 32) Rx Element R0 Configuration -------- */
typedef union {
	struct {
		/* bit:  0..28  Identifier */
		uint32_t ID:29;
		/* bit:  29     Remote Transmission Request */
		uint32_t RTR:1;
		/* bit:  30     Extended Identifier */
		uint32_t XTD:1;
		/* bit:  31     Error State Indicator */
		uint32_t ESI:1;
	} bit;
	/* Type used for register access */
	uint32_t reg;
} MCAN_RX_ELEMENT_R0_Type;

#define MCAN_RX_ELEMENT_R0_ID_Pos          0
#define MCAN_RX_ELEMENT_R0_ID_Msk          (0x1FFFFFFFul << MCAN_RX_ELEMENT_R0_ID_Pos)
#define MCAN_RX_ELEMENT_R0_ID(value)       ((MCAN_RX_ELEMENT_R0_ID_Msk & ((value) << MCAN_RX_ELEMENT_R0_ID_Pos)))
#define MCAN_RX_ELEMENT_R0_RTR_Pos         29
#define MCAN_RX_ELEMENT_R0_RTR             (0x1ul << MCAN_RX_ELEMENT_R0_RTR_Pos)
#define MCAN_RX_ELEMENT_R0_XTD_Pos         30
#define MCAN_RX_ELEMENT_R0_XTD             (0x1ul << MCAN_RX_ELEMENT_R0_XTD_Pos)
#define MCAN_RX_ELEMENT_R0_ESI_Pos         31
#define MCAN_RX_ELEMENT_R0_ESI             (0x1ul << MCAN_RX_ELEMENT_R0_ESI_Pos)

/* -------- MCAN_RX_ELEMENT_R1 : (MCAN RX element: 0x01) (R/W 32) Rx Element R1 Configuration -------- */
typedef union {
	struct {
		/* bit: 0..15   Rx Timestamp */
		uint32_t RXTS:16;
		/* bit: 16..19  Data Length Code */
		uint32_t DLC:4;
		/* bit: 20      Bit Rate Switch */
		uint32_t BRS:1;
		/* bit: 21      FD Format */
		uint32_t EDL:1;
		/* bit: 22..23  Reserved */
		uint32_t :2;
		/* bit: 24..30  Filter Index */
		uint32_t FIDX:7;
		/* bit: 31      Accepted Non-matching Frame */
		uint32_t ANMF:1;
	} bit;
	 /* Type used for register access */
	uint32_t reg;
} MCAN_RX_ELEMENT_R1_Type;

#define MCAN_RX_ELEMENT_R1_RXTS_Pos        0
#define MCAN_RX_ELEMENT_R1_RXTS_Msk        (0xFFFFul << MCAN_RX_ELEMENT_R1_RXTS_Pos)
#define MCAN_RX_ELEMENT_R1_RXTS(value)     ((MCAN_RX_ELEMENT_R1_RXTS_Msk & ((value) << MCAN_RX_ELEMENT_R1_RXTS_Pos)))
#define MCAN_RX_ELEMENT_R1_DLC_Pos         16
#define MCAN_RX_ELEMENT_R1_DLC_Msk         (0xFul << MCAN_RX_ELEMENT_R1_DLC_Pos)
#define MCAN_RX_ELEMENT_R1_DLC(value)      ((MCAN_RX_ELEMENT_R1_DLC_Msk & ((value) << MCAN_RX_ELEMENT_R1_DLC_Pos)))
#define MCAN_RX_ELEMENT_R1_BRS_Pos         20
#define MCAN_RX_ELEMENT_R1_BRS             (0x1ul << MCAN_RX_ELEMENT_R1_BRS_Pos)
#define MCAN_RX_ELEMENT_R1_FDF_Pos         21
#define MCAN_RX_ELEMENT_R1_FDF             (0x1ul << MCAN_RX_ELEMENT_R1_FDF_Pos)
#define MCAN_RX_ELEMENT_R1_FIDX_Pos        24
#define MCAN_RX_ELEMENT_R1_FIDX_Msk        (0x7Ful << MCAN_RX_ELEMENT_R1_FIDX_Pos)
#define MCAN_RX_ELEMENT_R1_FIDX(value)     ((MCAN_RX_ELEMENT_R1_FIDX_Msk & ((value) << MCAN_RX_ELEMENT_R1_FIDX_Pos)))
#define MCAN_RX_ELEMENT_R1_ANMF_Pos        31
#define MCAN_RX_ELEMENT_R1_ANMF            (0x1ul << MCAN_RX_ELEMENT_R1_ANMF_Pos)

//TODO there is no need for these 3 structs to be different
/**
 * \brief MCAN receive element structure for buffer.
 */
struct mcan_rx_element {
	__IO MCAN_RX_ELEMENT_R0_Type R0;
	__IO MCAN_RX_ELEMENT_R1_Type R1;
	uint8_t data[CONF_MCAN_ELEMENT_DATA_SIZE];

	mcan_rx_element& operator=(const mcan_rx_element& other)
	{
		R0.reg = other.R0.reg;
		R1.reg = other.R1.reg;
		memcpy(data, other.data, sizeof(data));
		return *this;
	}
};

/* -------- MCAN_TX_ELEMENT_T0 : (MCAN TX element: 0x00) (R/W 32) Tx Element T0 Configuration -------- */
typedef union {
	struct {
		/* bit:  0..28  Identifier */
		uint32_t ID:29;
		/* bit:  29     Remote Transmission Request */
		uint32_t RTR:1;
		/* bit:  30     Extended Identifier */
		uint32_t XTD:1;
#if (SAMV71B || SAME70B || SAMV70B)
		/* bit:  31     Error State Indicator */
		uint32_t ESI:1;
#else
		/* bit:  31     Reserved */
		uint32_t :1;
#endif
	} bit;
	/* Type used for register access */
  uint32_t reg;
} MCAN_TX_ELEMENT_T0_Type;

#define MCAN_TX_ELEMENT_T0_EXTENDED_ID_Pos          0
#define MCAN_TX_ELEMENT_T0_EXTENDED_ID_Msk          (0x1FFFFFFFul << MCAN_TX_ELEMENT_T0_EXTENDED_ID_Pos)
#define MCAN_TX_ELEMENT_T0_EXTENDED_ID(value)       ((MCAN_TX_ELEMENT_T0_EXTENDED_ID_Msk & ((value) << MCAN_TX_ELEMENT_T0_EXTENDED_ID_Pos)))
#define MCAN_TX_ELEMENT_T0_STANDARD_ID_Pos          18
#define MCAN_TX_ELEMENT_T0_STANDARD_ID_Msk          (0x7FFul << MCAN_TX_ELEMENT_T0_STANDARD_ID_Pos)
#define MCAN_TX_ELEMENT_T0_STANDARD_ID(value)       ((MCAN_TX_ELEMENT_T0_STANDARD_ID_Msk & ((value) << MCAN_TX_ELEMENT_T0_STANDARD_ID_Pos)))
#define MCAN_TX_ELEMENT_T0_RTR_Pos         29
#define MCAN_TX_ELEMENT_T0_RTR             (0x1ul << MCAN_TX_ELEMENT_T0_RTR_Pos)
#define MCAN_TX_ELEMENT_T0_XTD_Pos         30
#define MCAN_TX_ELEMENT_T0_XTD             (0x1ul << MCAN_TX_ELEMENT_T0_XTD_Pos)
#if (SAMV71B || SAME70B || SAMV70B)
#define MCAN_TX_ELEMENT_T0_ESI_Pos         31
#define MCAN_TX_ELEMENT_T0_ESI             (0x1ul << MCAN_TX_ELEMENT_T0_ESI_Pos)
#endif

/* -------- MCAN_TX_ELEMENT_T1 : (MCAN TX element: 0x01) (R/W 32) Tx Element T1 Configuration -------- */
typedef union {
	struct {
		/* bit: 0..15   Reserved */
		uint32_t :16;
		/* bit: 16..19  Data Length Code */
		uint32_t DLC:4;
#if (SAMV71B || SAME70B || SAMV70B)
		/* bit: 20      Bit Rate Switch */
		uint32_t BRS:1;
		/* bit: 21      FD Format */
		uint32_t FDF:1;
        /* bit: 22  Reserved */
        uint32_t :1;
#else
		/* bit: 20..22  Reserved */
		uint32_t :3;
#endif
    	/* bit: 23      Event FIFO Control */
		uint32_t EFCC:1;
		/* bit: 24..31  Message Marker */
		uint32_t MM:8;
	} bit;
	/* Type used for register access */
	uint32_t reg;
} MCAN_TX_ELEMENT_T1_Type;

#define MCAN_TX_ELEMENT_T1_DLC_Pos         16
#define MCAN_TX_ELEMENT_T1_DLC_Msk         (0xFul << MCAN_TX_ELEMENT_T1_DLC_Pos)
#define MCAN_TX_ELEMENT_T1_DLC(value)      ((MCAN_TX_ELEMENT_T1_DLC_Msk & ((value) << MCAN_TX_ELEMENT_T1_DLC_Pos)))
/**< \brief (MCAN_RXESC) 8 byte data field */
#define MCAN_TX_ELEMENT_T1_DLC_DATA8_Val        0x8ul
/**< \brief (MCAN_RXESC) 12 byte data field */
#define MCAN_TX_ELEMENT_T1_DLC_DATA12_Val       0x9ul
/**< \brief (MCAN_RXESC) 16 byte data field */
#define MCAN_TX_ELEMENT_T1_DLC_DATA16_Val       0xAul
/**< \brief (MCAN_RXESC) 20 byte data field */
#define MCAN_TX_ELEMENT_T1_DLC_DATA20_Val       0xBul
/**< \brief (MCAN_RXESC) 24 byte data field */
#define MCAN_TX_ELEMENT_T1_DLC_DATA24_Val       0xCul
/**< \brief (MCAN_RXESC) 32 byte data field */
#define MCAN_TX_ELEMENT_T1_DLC_DATA32_Val       0xDul
/**< \brief (MCAN_RXESC) 48 byte data field */
#define MCAN_TX_ELEMENT_T1_DLC_DATA48_Val       0xEul
/**< \brief (MCAN_RXESC) 64 byte data field */
#define MCAN_TX_ELEMENT_T1_DLC_DATA64_Val       0xFul
#if (SAMV71B || SAME70B || SAMV70B)
#define MCAN_TX_ELEMENT_T1_BRS_Pos         20
#define MCAN_TX_ELEMENT_T1_BRS             (0x1ul << MCAN_TX_ELEMENT_T1_BRS_Pos)
#define MCAN_TX_ELEMENT_T1_FDF_Pos         21
#define MCAN_TX_ELEMENT_T1_FDF             (0x1ul << MCAN_TX_ELEMENT_T1_FDF_Pos)
#endif
#define MCAN_TX_ELEMENT_T1_EFC_Pos         23
#define MCAN_TX_ELEMENT_T1_EFC             (0x1ul << MCAN_TX_ELEMENT_T1_EFC_Pos)
#define MCAN_TX_ELEMENT_T1_MM_Pos          24
#define MCAN_TX_ELEMENT_T1_MM_Msk          (0xFFul << MCAN_TX_ELEMENT_T1_MM_Pos)
#define MCAN_TX_ELEMENT_T1_MM(value)       ((MCAN_TX_ELEMENT_T1_MM_Msk & ((value) << MCAN_TX_ELEMENT_T1_MM_Pos)))

/**
 * \brief MCAN transfer element structure.
 *
 *  Common element structure for transfer buffer and FIFO/QUEUE.
 */
struct mcan_tx_element {
	__IO MCAN_TX_ELEMENT_T0_Type T0;
	__IO MCAN_TX_ELEMENT_T1_Type T1;
	uint8_t data[CONF_MCAN_ELEMENT_DATA_SIZE];
};

/* -------- MCAN_TX_EVENT_ELEMENT_E0 : (MCAN TX event element: 0x00) (R/W 32) Tx Event Element E0 Configuration -------- */
typedef union {
	struct {
		/* bit: 0..28  Identifier */
		uint32_t ID:29;
		/* bit: 29     Remote Transmission Request */
		uint32_t RTR:1;
		/* bit: 30     Extended Identifier */
		uint32_t XTD:1;
		/* bit: 31     Error State Indicator */
		uint32_t ESI:1;
	} bit;
	/* Type used for register access */
	uint32_t reg;
} MCAN_TX_EVENT_ELEMENT_E0_Type;

#define MCAN_TX_EVENT_ELEMENT_E0_ID_Pos          0
#define MCAN_TX_EVENT_ELEMENT_E0_ID_Msk          (0x1FFFFFFFul << MCAN_TX_EVENT_ELEMENT_E0_ID_Pos)
#define MCAN_TX_EVENT_ELEMENT_E0_ID(value)       ((MCAN_TX_EVENT_ELEMENT_E0_ID_Msk & ((value) << MCAN_TX_EVENT_ELEMENT_E0_ID_Pos)))
#define MCAN_TX_EVENT_ELEMENT_E0_RTR_Pos         29
#define MCAN_TX_EVENT_ELEMENT_E0_RTR             (0x1ul << MCAN_TX_EVENT_ELEMENT_E0_RTR_Pos)
#define MCAN_TX_EVENT_ELEMENT_E0_XTD_Pos         30
#define MCAN_TX_EVENT_ELEMENT_E0_XTD             (0x1ul << MCAN_TX_EVENT_ELEMENT_E0_XTD_Pos)
#define MCAN_TX_EVENT_ELEMENT_E0_ESI_Pos         31
#define MCAN_TX_EVENT_ELEMENT_E0_ESI             (0x1ul << MCAN_TX_EVENT_ELEMENT_E0_ESI_Pos)

/* -------- MCAN_TX_EVENT_ELEMENT_E1 : (MCAN TX event element: 0x01) (R/W 32) Tx Event Element E1 Configuration -------- */
typedef union {
	struct {
		/* bit: 0..15   Tx Timestamp */
		uint32_t TXTS:16;
		/* bit: 16..19  Data Length Code */
		uint32_t DLC:4;
		/* bit: 20      Bit Rate Switch */
		uint32_t BRS:1;
		/* bit: 21      FD Format */
		uint32_t EDL:1;
		/* bit: 22..23  Event Type */
		uint32_t ET:2;
		/* bit: 24..31  Message Marker */
		uint32_t MM:8;
	} bit;
	/* Type used for register access */
	uint32_t reg;
} MCAN_TX_EVENT_ELEMENT_E1_Type;

#define MCAN_TX_EVENT_ELEMENT_E1_TXTS_Pos        0
#define MCAN_TX_EVENT_ELEMENT_E1_TXTS_Msk        (0xFFFFul << MCAN_TX_EVENT_ELEMENT_E1_TXTS_Pos)
#define MCAN_TX_EVENT_ELEMENT_E1_TXTS(value)     ((MCAN_TX_EVENT_ELEMENT_E1_TXTS_Msk & ((value) << MCAN_TX_EVENT_ELEMENT_E1_TXTS_Pos)))
#define MCAN_TX_EVENT_ELEMENT_E1_DLC_Pos         16
#define MCAN_TX_EVENT_ELEMENT_E1_DLC_Msk         (0xFul << MCAN_TX_EVENT_ELEMENT_E1_DLC_Pos)
#define MCAN_TX_EVENT_ELEMENT_E1_DLC(value)      ((MCAN_TX_EVENT_ELEMENT_E1_DLC_Msk & ((value) << MCAN_TX_EVENT_ELEMENT_E1_DLC_Pos)))
#define MCAN_TX_EVENT_ELEMENT_E1_BRS_Pos         20
#define MCAN_TX_EVENT_ELEMENT_E1_BRS             (0x1ul << MCAN_TX_EVENT_ELEMENT_E1_BRS_Pos)
#define MCAN_TX_EVENT_ELEMENT_E1_FDF_Pos         21
#define MCAN_TX_EVENT_ELEMENT_E1_FDF             (0x1ul << MCAN_TX_EVENT_ELEMENT_E1_FDF_Pos)
#define MCAN_TX_EVENT_ELEMENT_E1_ET_Pos          22
#define MCAN_TX_EVENT_ELEMENT_E1_ET_Msk          (0x3ul << MCAN_TX_EVENT_ELEMENT_E1_ET_Pos)
#define MCAN_TX_EVENT_ELEMENT_E1_ET(value)       ((MCAN_TX_EVENT_ELEMENT_E1_ET_Msk & ((value) << MCAN_TX_EVENT_ELEMENT_E1_ET_Pos)))
#define MCAN_TX_EVENT_ELEMENT_E1_MM_Pos          24
#define MCAN_TX_EVENT_ELEMENT_E1_MM_Msk          (0xFFul << MCAN_TX_EVENT_ELEMENT_E1_MM_Pos)
#define MCAN_TX_EVENT_ELEMENT_E1_MM(value)       ((MCAN_TX_EVENT_ELEMENT_E1_MM_Msk & ((value) << MCAN_TX_EVENT_ELEMENT_E1_MM_Pos)))

/**
 * \brief MCAN transfer event  FIFO element structure.
 *
 *  Common element structure for transfer event FIFO.
 */
struct mcan_tx_event_element {
	__IO MCAN_TX_EVENT_ELEMENT_E0_Type E0;
	__IO MCAN_TX_EVENT_ELEMENT_E1_Type E1;
};

/* -------- MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0 : (MCAN standard message ID filter element: 0x00) (R/W 32) Standard Message ID Filter Element S0 Configuration -------- */
typedef union {
	struct {
		/* bit: 0..10   Standard Filter ID 2 */
		uint32_t SFID2:11;
		/* bit: 11..15  Reserved */
		uint32_t :5;
		/* bit: 16..26  Standard Filter ID 1 */
		uint32_t SFID1:11;
		/* bit: 27..29  Standard Filter Element Configuration */
		uint32_t SFEC:3;
		/* bit: 30..31  Standard Filter Type */
		uint32_t SFT:2;
	} bit;
	/* Type used for register access */
	uint32_t reg;
} MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_Type;

#define MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFID2_Pos          0
#define MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFID2_Msk          (0x7FFul << MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFID2_Pos)
#define MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFID2(value)       ((MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFID2_Msk & ((value) << MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFID2_Pos)))
#define MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFID1_Pos          16
#define MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFID1_Msk          (0x7FFul << MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFID1_Pos)
#define MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFID1(value)       ((MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFID1_Msk & ((value) << MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFID1_Pos)))
#define MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFEC_Pos           27
#define MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFEC_Msk           (0x7ul << MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFEC_Pos)
#define MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFEC(value)        ((MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFEC_Msk & ((value) << MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFEC_Pos)))
#define MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFEC_DISABLE_Val     0
#define MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFEC_STF0M_Val       1
#define MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFEC_STF1M_Val       2
#define MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFEC_REJECT_Val      3
#define MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFEC_PRIORITY_Val    4
#define MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFEC_PRIF0M_Val      5
#define MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFEC_PRIF1M_Val      6
#define MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFEC_STRXBUF_Val     7
#define MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFT_Pos            30
#define MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFT_Msk            (0x3ul << MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFT_Pos)
#define MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFT(value)         ((MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFT_Msk & ((value) << MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFT_Pos)))
#define MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFT_RANGE          MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFT(0)
#define MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFT_DUAL           MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFT(1)
#define MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFT_CLASSIC        MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFT(2)

/**
 * \brief MCAN standard message ID filter element structure.
 *
 *  Common element structure for standard message ID filter element.
 */
struct mcan_standard_message_filter_element {
	__IO MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_Type S0;
};

/* -------- MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0 : (MCAN extended message ID filter element: 0x00) (R/W 32) Extended Message ID Filter Element F0 Configuration -------- */
typedef union {
	struct {
		/* bit: 0..28   Extended Filter ID 1 */
		uint32_t EFID1:29;
		/* bit: 29..31  Extended Filter Element Configuration */
		uint32_t EFEC:3;
	} bit;
	/* Type used for register access */
	uint32_t reg;
} MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0_Type;

#define MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0_EFID1_Pos          0
#define MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0_EFID1_Msk          (0x1FFFFFFFul << MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0_EFID1_Pos)
#define MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0_EFID1(value)       ((MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0_EFID1_Msk & ((value) << MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0_EFID1_Pos)))
#define MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0_EFEC_Pos           29
#define MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0_EFEC_Msk           (0x7ul << MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0_EFEC_Pos)
#define MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0_EFEC(value)        ((MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0_EFEC_Msk & ((value) << MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0_EFEC_Pos)))
#define MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0_EFEC_DISABLE_Val       0
#define MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0_EFEC_STF0M_Val         1
#define MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0_EFEC_STF1M_Val         2
#define MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0_EFEC_REJECT_Val        3
#define MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0_EFEC_PRIORITY_Val      4
#define MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0_EFEC_PRIF0M_Val        5
#define MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0_EFEC_PRIF1M_Val        6
#define MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0_EFEC_STRXBUF_Val       7

/* -------- MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F1 : (MCAN extended message ID filter element: 0x01) (R/W 32) Extended Message ID Filter Element F1 Configuration -------- */
typedef union {
	struct {
		/* bit: 0..28  Extended Filter ID 2 */
		uint32_t EFID2:29;
		/* bit: 29     Reserved */
		uint32_t :1;
		/* bit: 30..31 Extended Filter Type */
		uint32_t EFT:2;
	} bit;
	/* Type used for register access */
	uint32_t reg;
} MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F1_Type;

#define MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F1_EFID2_Pos          0
#define MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F1_EFID2_Msk          (0x1FFFFFFFul << MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F1_EFID2_Pos)
#define MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F1_EFID2(value)       ((MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F1_EFID2_Msk & ((value) << MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F1_EFID2_Pos)))
#define MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F1_EFT_Pos            30
#define MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F1_EFT_Msk            (0x3ul << MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F1_EFT_Pos)
#define MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F1_EFT(value)         ((MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F1_EFT_Msk & ((value) << MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F1_EFT_Pos)))
#define MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F1_EFT_RANGEM       MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F1_EFT(0)
#define MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F1_EFT_DUAL         MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F1_EFT(1)
#define MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F1_EFT_CLASSIC      MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F1_EFT(2)
#define MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F1_EFT_RANGE        MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F1_EFT(3)

/**
 * \brief MCAN extended message ID filter element structure.
 *
 *  Common element structure for extended message ID filter element.
 */
struct mcan_extended_message_filter_element {
	__IO MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0_Type F0;
	__IO MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F1_Type F1;
};
/** @} */

/**
 * \defgroup asfdoc_sam_mcan_group SAM Control Area Network (MCAN) Low Level Driver
 *
 * This driver for Atmel® | SMART SAM devices provides an low level
 * interface for the configuration and management of the device's
 * Control Area Network functionality.
 *
 * \note  Since "The Control Area Network (CAN) performs communication according
 * to ISO 11898-1 (Bosch CAN specification 2.0 part A,B) and to Bosch CAN FD
 * specification V1.0", the driver is focus on the MAC layer and try to offer
 * the APIs which can be used by upper application layer.
 *
 * For storage of Rx/Tx messages and for storage of the filter configuration,
 * a message RAM is needed to the CAN module. In this driver, the message RAM
 * is static allocated, the related setting is defined and can be changed in
 * the module configuration file "conf_mcan.h".
 *
 * The following peripherals are used by this module:
 *  - CAN (Control Area Network)
 *
 * The following devices can use this module:
 *  - SAMV71
 *
 * The outline of this documentation is as follows:
 *  - \ref asfdoc_sam_mcan_prerequisites
 *  - \ref asfdoc_sam_mcan_module_overview
 *  - \ref asfdoc_sam_mcan_special_considerations
 *  - \ref asfdoc_sam_mcan_extra_info
 *  - \ref asfdoc_sam_mcan_examples
 *  - \ref asfdoc_sam_mcan_api_overview
 *
 *
 * \section asfdoc_sam_mcan_prerequisites Prerequisites
 *
 * There are no prerequisites for this module.
 *
 *
 * \section asfdoc_sam_mcan_module_overview Module Overview
 *
 * This driver provides an interface for the Control Area Network Controller
 * functions on the device.
 *
 *
 * \section asfdoc_sam_mcan_special_considerations Special Considerations
 *
 * There are no special considerations for this module.
 *
 *
 * \section asfdoc_sam_mcan_extra_info Extra Information
 *
 * For extra information see \ref asfdoc_sam_mcan_extra. This includes:
 *  - \ref asfdoc_sam_mcan_extra_acronyms
 *  - \ref asfdoc_sam_mcan_extra_dependencies
 *  - \ref asfdoc_sam_mcan_extra_errata
 *  - \ref asfdoc_sam_mcan_extra_history
 *
 *
 * \section asfdoc_sam_mcan_examples Examples
 *
 * For a list of examples related to this driver, see
 * \ref asfdoc_sam_mcan_exqsg.
 *
 *
 * \section asfdoc_sam_mcan_api_overview API Overview
 * @{
 */

/**
 * \name Module Setting
 * @{
 */

/**
 * \brief Can time out modes.
 */
enum mcan_timeout_mode {
	/** Continuous operation. */
	MCAN_TIMEOUT_CONTINUES = MCAN_TOCC_TOS_CONTINUOUS,
	/** Timeout controlled by TX Event FIFO. */
	MCAN_TIMEOUT_TX_EVEN_FIFO = MCAN_TOCC_TOS_TX_EV_TIMEOUT,
	/** Timeout controlled by Rx FIFO 0. */
	MCAN_TIMEOUT_RX_FIFO_0 = MCAN_TOCC_TOS_RX0_EV_TIMEOUT,
	/** Timeout controlled by Rx FIFO 1. */
	MCAN_TIMEOUT_RX_FIFO_1 = MCAN_TOCC_TOS_RX1_EV_TIMEOUT,
};

/**
 * \brief Can nonmatching frames action.
 */
enum mcan_nonmatching_frames_action {
	/** Accept in Rx FIFO 0. */
	MCAN_NONMATCHING_FRAMES_FIFO_0,
	/** Accept in Rx FIFO 1. */
	MCAN_NONMATCHING_FRAMES_FIFO_1,
	/** Reject. */
	MCAN_NONMATCHING_FRAMES_REJECT,
};

/**
 * \brief MCAN software device instance structure.
 *
 * MCAN software instance structure, used to retain software state information
 * of an associated hardware module instance.
 *
 * \note The fields of this structure should not be altered by the user
 *       application; they are reserved for module-internal use only.
 */
struct mcan_module {
	Mcan *hw;
};

/**
 * \brief MCAN configuration structure.
 *
 * Configuration structure for an MCAN instance. This structure should be
 * initialized by the \ref mcan_get_config_defaults()
 * function before being modified by the user application.
 */
struct mcan_config {
	/** MCAN run in standby control. */
	bool run_in_standby;
	/** Start value of the Message RAM Watchdog Counter */
	uint8_t watchdog_configuration;
	/** Transmit Pause. */
	bool transmit_pause;
	/** Edge Filtering during Bus Integration. */
	bool edge_filtering;
	/** Protocol Exception Handling. */
	bool protocol_exception_handling;
	/** Automatic Retransmission. */
	bool automatic_retransmission;
	/** Clock Stop Request. */
	bool clock_stop_request;
	/** Clock Stop Acknowledge. */
	bool clock_stop_acknowledge;
	/** Timestamp Counter Prescaler: 0x0-0xF */
	uint8_t timestamp_prescaler;
	/** Timeout Period. */
	uint16_t timeout_period;
	/** Timeout Mode. */
	enum mcan_timeout_mode timeout_mode;
	/** Timeout enable. */
	bool timeout_enable;
	/** Transceiver Delay Compensation enable. */
	bool tdc_enable;
	/** Transmitter Delay Compensation Offset : 0x0-0x7F */
	uint8_t delay_compensation_offset;
#if (SAMV71B || SAME70B || SAMV70B)
	/** Transmitter Delay Compensation Filter Window Length : 0x0-0x7F */
	uint8_t delay_compensation_filter_window_length;
#endif
	/** Nonmatching frames action for standard frames. */
	enum mcan_nonmatching_frames_action nonmatching_frames_action_standard;
	/** Nonmatching frames action for extended frames. */
	enum mcan_nonmatching_frames_action nonmatching_frames_action_extended;
	/** Reject Remote Standard Frames. */
	bool remote_frames_standard_reject;
	/** Reject Remote Extended Frames. */
	bool remote_frames_extended_reject;
	/** Extended ID Mask: 0x0-0x1FFFFFFF. */
	uint32_t extended_id_mask;
	/** Rx FIFO 0 Operation Mode. */
	bool rx_fifo_0_overwrite;
	/** Rx FIFO 0 Watermark: 1-64, other value disable it. */
	uint8_t rx_fifo_0_watermark;
	/** Rx FIFO 1 Operation Mode. */
	bool rx_fifo_1_overwrite;
	/** Rx FIFO 1 Watermark: 1-64, other value disable it. */
	uint8_t rx_fifo_1_watermark;
	/** Tx FIFO/Queue Mode, 0 for FIFO and 1 for Queue. */
	bool tx_queue_mode;
	/** Tx Event FIFO Watermark: 1-32, other value disable it. */
	uint8_t tx_event_fifo_watermark;
};

/**
 * \brief Initializes an MCAN configuration structure to defaults
 *
 * Initializes a given MCAN configuration struct to a set of known default
 * values. This function should be called on any new instance of the
 * configuration struct before being modified by the user application.
 *
 * The default configuration is as follows:
 *  \li Not run in standby mode
 *  \li Disable Watchdog
 *  \li Transmit pause enabled
 *  \li Edge filtering during bus integration enabled
 *  \li Protocol exception handling enabled
 *  \li Automatic retransmission enabled
 *  \li Clock stop request disabled
 *  \li Clock stop acknowledge disabled
 *  \li Timestamp Counter Prescaler 1
 *  \li Timeout Period with 0xFFFF
 *  \li Timeout Mode: Continuous operation
 *  \li Disable Timeout
 *  \li Transmitter Delay Compensation Offset is 0
 *  \li Transmitter Delay Compensation Filter Window Length is 0
 *  \li Reject nonmatching standard frames
 *  \li Reject nonmatching extended frames
 *  \li Reject remote standard frames
 *  \li Reject remote extended frames
 *  \li Extended ID Mask is 0x1FFFFFFF
 *  \li Rx FIFO 0 Operation Mode: overwrite
 *  \li Disable Rx FIFO 0 Watermark
 *  \li Rx FIFO 1 Operation Mode: overwrite
 *  \li Disable Rx FIFO 1 Watermark
 *  \li Tx FIFO/Queue Mode: FIFO
 *  \li Disable Tx Event FIFO Watermark
 *
 * \param[out] config  Pointer to configuration struct to initialize to
 *                     default values
 */
static inline void mcan_get_config_defaults(struct mcan_config *const config)
{
	/* Sanity check arguments */
	Assert(config);

	/* Default configuration values */
	config->run_in_standby = false;
	config->watchdog_configuration = 0x00;
	config->transmit_pause = true;
	config->edge_filtering = true;
	config->protocol_exception_handling = true;
	config->automatic_retransmission = true;
	config->clock_stop_request = false;
	config->clock_stop_acknowledge = false;
	config->timestamp_prescaler = 0;
	config->timeout_period = 0xFFFF;
	config->timeout_mode = MCAN_TIMEOUT_CONTINUES;
	config->timeout_enable = false;
	config->tdc_enable = false;
	config->delay_compensation_offset = 0;
#if (SAMV71B || SAME70B || SAMV70B)
	config->delay_compensation_filter_window_length = 0;
#endif
	config->nonmatching_frames_action_standard = MCAN_NONMATCHING_FRAMES_REJECT;
	config->nonmatching_frames_action_extended = MCAN_NONMATCHING_FRAMES_REJECT;
	config->remote_frames_standard_reject = true;
	config->remote_frames_extended_reject = true;
	config->extended_id_mask = 0x1FFFFFFF;
	config->rx_fifo_0_overwrite = true;
	config->rx_fifo_0_watermark = 0;
	config->rx_fifo_1_overwrite = true;
	config->rx_fifo_1_watermark = 0;
	config->tx_queue_mode = false;
	config->tx_event_fifo_watermark = 0;
}

void mcan_init(struct mcan_module *const module_inst, Mcan *hw, struct mcan_config *config);
void mcan_start(struct mcan_module *const module_inst);
void mcan_stop(struct mcan_module *const module_inst);
void mcan_enable_fd_mode(struct mcan_module *const module_inst);
void mcan_disable_fd_mode(struct mcan_module *const module_inst);
void mcan_enable_restricted_operation_mode(struct mcan_module *const module_inst);
void mcan_disable_restricted_operation_mode(struct mcan_module *const module_inst);

void mcan_enable_bus_monitor_mode(struct mcan_module *const module_inst);
void mcan_disable_bus_monitor_mode(struct mcan_module *const module_inst);
void mcan_enable_sleep_mode(struct mcan_module *const module_inst);
void mcan_disable_sleep_mode(struct mcan_module *const module_inst);
void mcan_enable_test_mode(struct mcan_module *const module_inst);
void mcan_disable_test_mode(struct mcan_module *const module_inst);

/**
 * \brief Can read timestamp count value.
 *
 * \param[in] module_inst  Pointer to the MCAN software instance struct
 *
 * \return Timestamp count value.
 */
static inline uint16_t mcan_read_timestamp_count_value(struct mcan_module *const module_inst)
{
	return module_inst->hw->MCAN_TSCV;
}

/**
 * \brief Can read timeout  count value.
 *
 * \param[in] module_inst  Pointer to the MCAN software instance struct
 *
 * \return Timeout count value.
 */
static inline uint16_t mcan_read_timeout_count_value(struct mcan_module *const module_inst)
{
	return module_inst->hw->MCAN_TOCV;
}

/**
 * \brief Can read error count.
 *
 * \param[in] module_inst  Pointer to the MCAN software instance struct
 *
 * \return Error count value.
 */
static inline uint32_t mcan_read_error_count(struct mcan_module *const module_inst)
{
	return module_inst->hw->MCAN_ECR;
}

/**
 * \brief Can read protocol status.
 *
 * \param[in] module_inst  Pointer to the MCAN software instance struct
 *
 * \return protocol status value.
 */
static inline uint32_t mcan_read_protocal_status(struct mcan_module *const module_inst)
{
	return module_inst->hw->MCAN_PSR;
}

/** @} */

/**
 * \name Rx Handling
 * @{
 */

/**
 * \brief Read high priority message status.
 *
 * \param[in] module_inst  Pointer to the MCAN software instance struct
 *
 * \return High priority message status value.
 */
static inline uint32_t mcan_read_high_priority_message_status(struct mcan_module *const module_inst)
{
	return module_inst->hw->MCAN_HPMS;
}

/**
 * \brief Get Rx buffer status.
 *
 * \param[in] module_inst  Pointer to the MCAN software instance struct
 * \param[in] index  Index offset in Rx buffer
 *
 * \return Rx buffer status value.
 *
 *  \retval true Rx Buffer updated from new message.
 *  \retval false Rx Buffer not updated.
 */
static inline bool mcan_rx_get_buffer_status(struct mcan_module *const module_inst, uint32_t index)
{
	if (index < 32) {
		if (module_inst->hw->MCAN_NDAT1 & (1 << index)) {
			return true;
		} else {
			return false;
		}
	} else {
		index -= 32;
		if (module_inst->hw->MCAN_NDAT2 & (1 << index)) {
			return true;
		} else {
			return false;
		}
	}
}

/**
 * \brief Clear Rx buffer status.
 *
 * \param[in] module_inst  Pointer to the MCAN software instance struct
 * \param[in] index  Index offset in Rx buffer
 *
 */
static inline void mcan_rx_clear_buffer_status(struct mcan_module *const module_inst, uint32_t index)
{
	if (index < 32) {
		module_inst->hw->MCAN_NDAT1 = (1 << index);
	} else {
		index -= 32;
		module_inst->hw->MCAN_NDAT2 = (1 << index);
	}
}

/**
 * \brief Get Rx FIFO status.
 *
 * \param[in] module_inst  Pointer to the MCAN software instance struct
 * \param[in] fifo_number  Rx FIFO 0 or 1
 *
 * \return Rx FIFO status value.
 */
static inline uint32_t mcan_rx_get_fifo_status(struct mcan_module *const module_inst, bool fifo_number)
{
	if (!fifo_number) {
		return module_inst->hw->MCAN_RXF0S;
	} else {
		return module_inst->hw->MCAN_RXF1S;
	}
}

/**
 * \brief Set Rx acknowledge.
 *
 * \param[in] module_inst  Pointer to the MCAN software instance struct
 * \param[in] fifo_number  Rx FIFO 0 or 1
 * \param[in] index  Index offset in FIFO
 */
static inline void mcan_rx_fifo_acknowledge(struct mcan_module *const module_inst, bool fifo_number, uint32_t index)
{
	if (!fifo_number) {
		module_inst->hw->MCAN_RXF0A = MCAN_RXF0A_F0AI(index);
	} else {
		module_inst->hw->MCAN_RXF1A = MCAN_RXF1A_F1AI(index);
	}
}

/**
 * \brief Get the standard message filter default value.
 *
 * The default configuration is as follows:
 *  \li Classic filter: SFID1 = filter, SFID2 = mask
 *  \li Store in Rx FIFO 0 if filter matches
 *  \li SFID2 = 0x7FFul
 *  \li SFID1 = 0x0ul
 *
 * \param[out] sd_filter  Pointer to standard filter element struct to initialize to default values
 */
static inline void mcan_get_standard_message_filter_element_default(struct mcan_standard_message_filter_element *sd_filter)
{
	sd_filter->S0.reg = MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFID2_Msk |
			MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFID1(0) |
			MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFEC(
			MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFEC_STF0M_Val) |
			MCAN_STANDARD_MESSAGE_FILTER_ELEMENT_S0_SFT_CLASSIC;
}

enum status_code mcan_set_rx_standard_filter(struct mcan_module *const module_inst, struct mcan_standard_message_filter_element *sd_filter, uint32_t index);

/**
 * \brief Get the extended message filter default value.
 *
 * The default configuration is as follows:
 *  \li Classic filter: SFID1 = filter, SFID2 = mask
 *  \li Store in Rx FIFO 1 if filter matches
 *  \li SFID2 = 0x1FFFFFFFul
 *  \li SFID1 = 0x0ul
 *
 * \param[out] et_filter  Pointer to extended filter element struct to initialize to default values
 */
static inline void mcan_get_extended_message_filter_element_default(struct mcan_extended_message_filter_element *et_filter)
{
	et_filter->F0.reg = MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0_EFID1(0) |
			MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0_EFEC(
			MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F0_EFEC_STF1M_Val);
	et_filter->F1.reg = MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F1_EFID2_Msk |
			MCAN_EXTENDED_MESSAGE_FILTER_ELEMENT_F1_EFT_CLASSIC;
}

enum status_code mcan_set_rx_extended_filter(struct mcan_module *const module_inst, struct mcan_extended_message_filter_element *et_filter, uint32_t index);

enum status_code mcan_get_rx_buffer_element(struct mcan_module *const module_inst, struct mcan_rx_element *rx_element, uint32_t index);

enum status_code mcan_get_rx_fifo_0_element(struct mcan_module *const module_inst, struct mcan_rx_element *rx_element, uint32_t index);

enum status_code mcan_get_rx_fifo_1_element(struct mcan_module *const module_inst, struct mcan_rx_element *rx_element, uint32_t index);

/**
 * \brief Get Tx FIFO/Queue status.
 *
 * \param[in] module_inst  Pointer to the MCAN software instance struct
 *
 * \return Tx FIFO/Queue status value.
 */
static inline uint32_t mcan_tx_get_fifo_queue_status(struct mcan_module *const module_inst)
{
	return module_inst->hw->MCAN_TXFQS;
}

/**
 * \brief Get Tx buffer request pending status.
 *
 * \param[in] module_inst  Pointer to the MCAN software instance struct
 *
 * \return Bit mask of Tx buffer request pending status value.
 */
static inline uint32_t mcan_tx_get_pending_status(struct mcan_module *const module_inst)
{
	return module_inst->hw->MCAN_TXBRP;
}

/**
 * \brief Tx buffer add transfer request.
 *
 * \param[in] module_inst  Pointer to the MCAN software instance struct
 * \param[in] trig_mask  The mask value to trigger transfer buffer
 *
 *  \return Status of the result.
 *
 *  \retval STATUS_OK   Set the transfer request.
 *  \retval STATUS_ERR_BUSY The module is in configuration.
 */
static inline enum status_code mcan_tx_transfer_request(struct mcan_module *const module_inst, uint32_t trig_mask)
{
	if (module_inst->hw->MCAN_CCCR & MCAN_CCCR_CCE) {
		return ERR_BUSY;
	}
	module_inst->hw->MCAN_TXBAR = trig_mask;
	return STATUS_OK;
}

/**
 * \brief Set Tx Queue operation.
 *
 * \param[in] module_inst  Pointer to the MCAN software instance struct
 * \param[in] trig_mask  The mask value to cancel transfer buffer
 *
 *  \return Status of the result.
 *
 *  \retval STATUS_OK   Set the transfer request.
 *  \retval STATUS_BUSY The module is in configuration.
 */
static inline enum status_code mcan_tx_cancel_request(struct mcan_module *const module_inst, uint32_t trig_mask)
{
	if (module_inst->hw->MCAN_CCCR & MCAN_CCCR_CCE) {
		return STATUS_ERR_BUSY;
	}
	module_inst->hw->MCAN_TXBCR = trig_mask;
	return STATUS_OK;
}

/**
 * \brief Get Tx transmission status.
 *
 * \param[in] module_inst  Pointer to the MCAN software instance struct
 *
 * \return Bit mask of Tx transmission status value.
 */
static inline uint32_t mcan_tx_get_transmission_status(struct mcan_module *const module_inst)
{
	return module_inst->hw->MCAN_TXBTO;
}

/**
 * \brief Get Tx cancellation status.
 *
 * \param[in] module_inst  Pointer to the MCAN software instance struct
 *
 * \return Bit mask of Tx cancellation status value.
 */
static inline uint32_t mcan_tx_get_cancellation_status(struct mcan_module *const module_inst)
{
	return module_inst->hw->MCAN_TXBCF;
}

/**
 * \brief Get Tx event FIFO status.
 *
 * \param[in] module_inst  Pointer to the MCAN software instance struct
 *
 * \return Tx event FIFO status value.
 */
static inline uint32_t mcan_tx_get_event_fifo_status(struct mcan_module *const module_inst)
{
	return module_inst->hw->MCAN_TXEFS;
}

/**
 * \brief Set Tx Queue operation.
 *
 * \param[in] module_inst  Pointer to the MCAN software instance struct
 * \param[in] index  Index for the transfer FIFO
 */
static inline void mcan_tx_event_fifo_acknowledge(struct mcan_module *const module_inst, uint32_t index)
{
	module_inst->hw->MCAN_TXEFA = MCAN_TXEFA_EFAI(index);
}

/**
 * \brief Get the default transfer buffer element.
 *
 * The default configuration is as follows:
 *  \li 11-bit standard identifier
 *  \li Transmit data frame
 *  \li ID = 0x0ul
 *  \li Store Tx events
 *  \li Frame transmitted in Classic MCAN format
 *  \li Data Length Code is 8
 *
 * \param[out] tx_element  Pointer to transfer element struct to initialize to default values
 */
static inline void mcan_get_tx_buffer_element_defaults(struct mcan_tx_element *tx_element)
{
	tx_element->T0.reg = 0;
	tx_element->T1.reg = MCAN_TX_ELEMENT_T1_EFC |
			MCAN_TX_ELEMENT_T1_DLC(MCAN_TX_ELEMENT_T1_DLC_DATA8_Val);
}

enum status_code mcan_set_tx_buffer_element(
		struct mcan_module *const module_inst,
		struct mcan_tx_element *tx_element, uint32_t index);

enum status_code mcan_get_tx_event_fifo_element(
		struct mcan_module *const module_inst,
		struct mcan_tx_event_element *tx_event_element, uint32_t index);

/**
 * \brief Can module interrupt source.
 *
 * Enum for the interrupt source.
 */
enum mcan_interrupt_source {
	/** Rx FIFO 0 New Message Interrupt Enable. */
	MCAN_RX_FIFO_0_NEW_MESSAGE = MCAN_IE_RF0NE,
	/** Rx FIFO 0 Watermark Reached Interrupt Enable. */
	MCAN_RX_FIFO_0_WATERMARK = MCAN_IE_RF0WE,
	/** Rx FIFO 0 Full Interrupt Enable. */
	MCAN_RX_FIFO_0_FULL = MCAN_IE_RF0FE,
	/** Rx FIFO 0 Message Lost Interrupt Enable. */
	MCAN_RX_FIFO_0_LOST_MESSAGE = MCAN_IE_RF0LE,
	/** Rx FIFO 1 New Message Interrupt Enable. */
	MCAN_RX_FIFO_1_NEW_MESSAGE = MCAN_IE_RF1NE,
	/** Rx FIFO 1 Watermark Reached Interrupt Enable. */
	MCAN_RX_FIFO_1_WATERMARK = MCAN_IE_RF1WE,
	/** Rx FIFO 1 Full Interrupt Enable. */
	MCAN_RX_FIFO_1_FULL = MCAN_IE_RF1FE,
	/** Rx FIFO 1 Message Lost Interrupt Enable. */
	MCAN_RX_FIFO_1_MESSAGE_LOST = MCAN_IE_RF1LE,
	/** High Priority Message Interrupt Enable. */
	MCAN_RX_HIGH_PRIORITY_MESSAGE = MCAN_IE_HPME,
	/**  Transmission Completed Interrupt Enable. */
	MCAN_TIMESTAMP_COMPLETE = MCAN_IE_TCE,
	/** Transmission Cancellation Finished Interrupt Enable. */
	MCAN_TX_CANCELLATION_FINISH = MCAN_IE_TCFE,
	/** Tx FIFO Empty Interrupt Enable. */
	MCAN_TX_FIFO_EMPTY = MCAN_IE_TFEE,
	/** Tx Event FIFO New Entry Interrupt Enable. */
	MCAN_TX_EVENT_FIFO_NEW_ENTRY = MCAN_IE_TEFNE,
	/** Tx Event FIFO Watermark Reached Interrupt Enable. */
	MCAN_TX_EVENT_FIFO_WATERMARK = MCAN_IE_TEFWE,
	/** Tx Event FIFO Full Interrupt Enable. */
	MCAN_TX_EVENT_FIFO_FULL = MCAN_IE_TEFFE,
	/** Tx Event FIFO Element Lost Interrupt Enable. */
	MCAN_TX_EVENT_FIFO_ELEMENT_LOST = MCAN_IE_TEFLE,
	/** Timestamp Wraparound Interrupt Enable. */
	MCAN_TIMESTAMP_WRAPAROUND = MCAN_IE_TSWE,
	/** Message RAM Access Failure Interrupt Enable. */
	MCAN_MESSAGE_RAM_ACCESS_FAILURE = MCAN_IE_MRAFE,
	/** Timeout Occurred Interrupt Enable. */
	MCAN_TIMEOUT_OCCURRED = MCAN_IE_TOOE,
	/** Message stored to Dedicated Rx Buffer Interrupt Enable. */
	MCAN_RX_BUFFER_NEW_MESSAGE = MCAN_IE_DRXE,
	/** Error Logging Overflow Interrupt Enable. */
	MCAN_ERROR_LOGGING_OVERFLOW = MCAN_IE_ELOE,
	/** Error Passive Interrupt Enable. */
	MCAN_ERROR_PASSIVE = MCAN_IE_EPE,
	/** Warning Status Interrupt Enable. */
	MCAN_WARNING_STATUS = MCAN_IE_EWE,
	/** Bus_Off Status Interrupt Enable. */
	MCAN_BUS_OFF = MCAN_IE_BOE,
	/** Watchdog Interrupt  Enable. */
	MCAN_WATCHDOG = MCAN_IE_WDIE,
	/**CRC Error Interrupt Enable */
	MCAN_CRC_ERROR = MCAN_IE_CRCEE,
	/** Bit Error Interrupt  Enable. */
	MCAN_BIT_ERROR = MCAN_IE_BEE,
	/** Acknowledge Error Interrupt Enable . */
	MCAN_ACKNOWLEDGE_ERROR = MCAN_IE_ACKEE,
	/** Format Error Interrupt Enable */
	MCAN_FORMAT_ERROR = MCAN_IE_FOEE,
	/** Stuff Error Interrupt Enable */
	MCAN_STUFF_ERROR = MCAN_IE_STEE
};

/**
 * \brief Enable MCAN interrupt.
 *
 * \param[in] module_inst  Pointer to the MCAN software instance struct
 * \param[in] source  Interrupt source type
 */
static inline void mcan_enable_interrupt(struct mcan_module *const module_inst, const enum mcan_interrupt_source source)
{
	module_inst->hw->MCAN_IE |= source;
}

/**
 * \brief Disable MCAN interrupt.
 *
 * \param[in] module_inst  Pointer to the MCAN software instance struct
 * \param[in] source  Interrupt source type
 */
static inline void mcan_disable_interrupt(struct mcan_module *const module_inst, const enum mcan_interrupt_source source)
{
	module_inst->hw->MCAN_IE &= ~source;
}

/**
 * \brief Get MCAN interrupt status.
 *
 * \param[in] module_inst  Pointer to the MCAN software instance struct
 */
static inline uint32_t mcan_read_interrupt_status(struct mcan_module *const module_inst)
{
	return module_inst->hw->MCAN_IR;
}

/**
 * \brief Clear MCAN interrupt status.
 *
 * \param[in] module_inst  Pointer to the MCAN software instance struct
 * \param[in] source  Interrupt source type
 *
 * \return Bit mask of interrupt status value.
 */
static inline void mcan_clear_interrupt_status(struct mcan_module *const module_inst, const enum mcan_interrupt_source source)
{
	module_inst->hw->MCAN_IR = source;
}

#endif

#endif /* SRC_HARDWARE_CANDRIVER_H_ */
