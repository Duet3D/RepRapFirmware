// Arduino Due - EMAC Sample 1
// Brief EMAC example for Arduino Due
// This example demonstrates how to configure and use
// the EMAC peripheral.
// By Wilfredo Molina @2013

// Required libraries
#include "variant.h"
#include <conf_eth.h>
#include <mini_ip.h>
#include <ethernet_phy.h>
#include <rmii.h>
#include <include/emac.h>
//#include <source/emac.c>
#include <include/rstc.h>
//#include <source/rstc.c>

void setup()
{
  // start serial port at 9600 bps:
  SerialUSB.begin(115200);
  while (!SerialUSB.available());
}

// The MAC address used for the test
static uint8_t gs_uc_mac_address[] =
		{ ETHERNET_CONF_ETHADDR0, ETHERNET_CONF_ETHADDR1, ETHERNET_CONF_ETHADDR2,
			ETHERNET_CONF_ETHADDR3, ETHERNET_CONF_ETHADDR4, ETHERNET_CONF_ETHADDR5
};

// The IP address used for test (ping ...)
static uint8_t gs_uc_ip_address[] =
		{ ETHERNET_CONF_IPADDR0, ETHERNET_CONF_IPADDR1,
			ETHERNET_CONF_IPADDR2, ETHERNET_CONF_IPADDR3 };

// The EMAC driver instance
static emac_device_t gs_emac_dev;

// Buffer for ethernet packets
static volatile uint8_t gs_uc_eth_buffer[EMAC_FRAME_LENTGH_MAX];

/**
 * \brief Process & return the ICMP checksum.
 *
 * \param p_buff Pointer to the buffer.
 * \param ul_len The length of the buffered data.
 *
 * \return Checksum of the ICMP.
 */
static uint16_t emac_icmp_checksum(uint16_t *p_buff, uint32_t ul_len)
{
	uint32_t i, ul_tmp;

	for (i = 0, ul_tmp = 0; i < ul_len; i++, p_buff++) {

		ul_tmp += SWAP16(*p_buff);
	}
	ul_tmp = (ul_tmp & 0xffff) + (ul_tmp >> 16);

	return (uint16_t) (~ul_tmp);
}

/**
 * \brief Display the IP packet.
 *
 * \param p_ip_header Pointer to the IP header.
 * \param ul_size    The data size.
 */
static void emac_display_ip_packet(p_ip_header_t p_ip_header, uint32_t ul_size)
{
/*	printf("======= IP %4d bytes, HEADER ==========\n\r", (int)ul_size);
	printf(" IP Version        = v.%d", (p_ip_header->ip_hl_v & 0xF0) >> 4);
	printf("\n\r Header Length     = %d", p_ip_header->ip_hl_v & 0x0F);
	printf("\n\r Type of service   = 0x%x", p_ip_header->ip_tos);
	printf("\n\r Total IP Length   = 0x%X",
			(((p_ip_header->ip_len) >> 8) & 0xff) +
			(((p_ip_header->ip_len) << 8) & 0xff00));
	printf("\n\r ID                = 0x%X",
			(((p_ip_header->ip_id) >> 8) & 0xff) +
			(((p_ip_header->ip_id) << 8) & 0xff00));
	printf("\n\r Header Checksum   = 0x%X",
			(((p_ip_header->ip_sum) >> 8) & 0xff) +
			(((p_ip_header->ip_sum) << 8) & 0xff00));
*/	SerialUSB.println("\r Protocol          = ");

	switch (p_ip_header->ip_p) {
	case IP_PROT_ICMP:
		SerialUSB.println("ICMP");
		break;

	case IP_PROT_IP:
		SerialUSB.println("IP");
		break;

	case IP_PROT_TCP:
		SerialUSB.println("TCP");
		break;

	case IP_PROT_UDP:
		SerialUSB.println("UDP");
		break;

	default:
//		printf("%d (0x%X)", p_ip_header->ip_p, p_ip_header->ip_p);
		break;
	}

/*	printf("\n\r IP Src Address    = %d:%d:%d:%d",
			p_ip_header->ip_src[0],
			p_ip_header->ip_src[1],
			p_ip_header->ip_src[2], p_ip_header->ip_src[3]);

	printf("\n\r IP Dest Address   = %d:%d:%d:%d",
			p_ip_header->ip_dst[0],
			p_ip_header->ip_dst[1],
			p_ip_header->ip_dst[2], p_ip_header->ip_dst[3]);
*/	SerialUSB.println("\n\r----------------------------------------\r");
}

/**
 * \brief Process the received ARP packet; change address and send it back.
 *
 * \param p_uc_data  The data to process.
 * \param ul_size The data size.
 */
static void emac_process_arp_packet(uint8_t *p_uc_data, uint32_t ul_size)
{
	uint32_t i;
	uint8_t ul_rc = EMAC_OK;

	p_ethernet_header_t p_eth = (p_ethernet_header_t) p_uc_data;
	p_arp_header_t p_arp = (p_arp_header_t) (p_uc_data + ETH_HEADER_SIZE);

	if (SWAP16(p_arp->ar_op) == ARP_REQUEST) {
/*		printf("-- IP  %d.%d.%d.%d\n\r",
				p_eth->et_dest[0], p_eth->et_dest[1],
				p_eth->et_dest[2], p_eth->et_dest[3]);

		printf("-- IP  %d.%d.%d.%d\n\r",
				p_eth->et_src[0], p_eth->et_src[1],
				p_eth->et_src[2], p_eth->et_src[3]);
*/
		// ARP reply operation
		p_arp->ar_op = SWAP16(ARP_REPLY);

		// Fill the destination address and source address
		for (i = 0; i < 6; i++) {
			// Swap ethernet destination address and ethernet source address
			p_eth->et_dest[i] = p_eth->et_src[i];
			p_eth->et_src[i] = gs_uc_mac_address[i];
			p_arp->ar_tha[i] = p_arp->ar_sha[i];
			p_arp->ar_sha[i] = gs_uc_mac_address[i];
		}
		// Swap the source IP address and the destination IP address
		for (i = 0; i < 4; i++) {
			p_arp->ar_tpa[i] = p_arp->ar_spa[i];
			p_arp->ar_spa[i] = gs_uc_ip_address[i];
		}
		ul_rc = emac_dev_write(&gs_emac_dev, p_uc_data, ul_size, NULL);
		if (ul_rc != EMAC_OK) {
			printf("E: ARP Send - 0x%x\n\r", ul_rc);
		}
	}
}

/**
 * \brief Process the received IP packet; change address and send it back.
 *
 * \param p_uc_data  The data to process.
 * \param ul_size The data size.
 */
static void emac_process_ip_packet(uint8_t *p_uc_data, uint32_t ul_size)
{
	uint32_t i;
	uint32_t ul_icmp_len;
	int32_t ul_rc = EMAC_OK;

	ul_size = ul_size;	// stop warning

	p_ethernet_header_t p_eth = (p_ethernet_header_t) p_uc_data;
	p_ip_header_t p_ip_header = (p_ip_header_t) (p_uc_data + ETH_HEADER_SIZE);

	p_icmp_echo_header_t p_icmp_echo =
			(p_icmp_echo_header_t) ((int8_t *) p_ip_header +
			ETH_IP_HEADER_SIZE);
/*	printf("-- IP  %d.%d.%d.%d\n\r", p_eth->et_dest[0], p_eth->et_dest[1],
			p_eth->et_dest[2], p_eth->et_dest[3]);

	printf("-- IP  %d.%d.%d.%d\n\r",
			p_eth->et_src[0], p_eth->et_src[1], p_eth->et_src[2],
			p_eth->et_src[3]);
*/	switch (p_ip_header->ip_p) {
	case IP_PROT_ICMP:
		if (p_icmp_echo->type == ICMP_ECHO_REQUEST) {
			p_icmp_echo->type = ICMP_ECHO_REPLY;
			p_icmp_echo->code = 0;
			p_icmp_echo->cksum = 0;

			// Checksum of the ICMP message
			ul_icmp_len = (SWAP16(p_ip_header->ip_len) - ETH_IP_HEADER_SIZE);
			if (ul_icmp_len % 2) {
				*((uint8_t *) p_icmp_echo + ul_icmp_len) = 0;
				ul_icmp_len++;
			}
			ul_icmp_len = ul_icmp_len / sizeof(uint16_t);

			p_icmp_echo->cksum = SWAP16(
					emac_icmp_checksum((uint16_t *)p_icmp_echo, ul_icmp_len));
			// Swap the IP destination  address and the IP source address
			for (i = 0; i < 4; i++) {
				p_ip_header->ip_dst[i] =
						p_ip_header->ip_src[i];
				p_ip_header->ip_src[i] = gs_uc_ip_address[i];
			}
			// Swap ethernet destination address and ethernet source address
			for (i = 0; i < 6; i++) {
				// Swap ethernet destination address and ethernet source address
				p_eth->et_dest[i] = p_eth->et_src[i];
				p_eth->et_src[i] = gs_uc_mac_address[i];
			}
			// Send the echo_reply
			ul_rc = emac_dev_write(&gs_emac_dev, p_uc_data,
					SWAP16(p_ip_header->ip_len) + 14, NULL);
			if (ul_rc != EMAC_OK) {
//				printf("E: ICMP Send - 0x%x\n\r", ul_rc);
			}
		}
		break;

	default:
		break;
	}
}

/**
 * \brief Process the received EMAC packet.
 *
 * \param p_uc_data  The data to process.
 * \param ul_size The data size.
 */
static void emac_process_eth_packet(uint8_t *p_uc_data, uint32_t ul_size)
{
	uint16_t us_pkt_format;

	p_ethernet_header_t p_eth = (p_ethernet_header_t) (p_uc_data);
	p_ip_header_t p_ip_header = (p_ip_header_t) (p_uc_data + ETH_HEADER_SIZE);
	ip_header_t ip_header;
	us_pkt_format = SWAP16(p_eth->et_protlen);

	switch (us_pkt_format) {
	// ARP Packet format
	case ETH_PROT_ARP:
		// Process the ARP packet
		emac_process_arp_packet(p_uc_data, ul_size);

		break;

	// IP protocol frame
	case ETH_PROT_IP:
		// Backup the header
		memcpy(&ip_header, p_ip_header, sizeof(ip_header_t));

		// Process the IP packet
		emac_process_ip_packet(p_uc_data, ul_size);

		// Dump the IP header
		emac_display_ip_packet(&ip_header, ul_size);
		break;

	default:
//		printf("=== Default w_pkt_format= 0x%X===\n\r", us_pkt_format);
		break;
	}
}

//brief EMAC interrupt handler.

void EMAC_Handler(void)
{
	emac_handler(&gs_emac_dev);
}

/**
 *  \brief EMAC example entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
void loop()
{
  
	uint32_t ul_frm_size;
	volatile uint32_t ul_delay;
	emac_options_t emac_option;
   
 	// Display MAC & IP settings
/*	printf("-- MAC %x:%x:%x:%x:%x:%x\n\r",
			gs_uc_mac_address[0], gs_uc_mac_address[1], gs_uc_mac_address[2],
			gs_uc_mac_address[3], gs_uc_mac_address[4], gs_uc_mac_address[5]);

	printf("-- IP  %d.%d.%d.%d\n\r", gs_uc_ip_address[0], gs_uc_ip_address[1],
			gs_uc_ip_address[2], gs_uc_ip_address[3]);
*/
	// Reset PHY
	rstc_set_external_reset(RSTC, 13); // (2^(13+1))/32768
	rstc_reset_extern(RSTC);
	SerialUSB.println("resetting PHY...");
	while (rstc_get_status(RSTC) & RSTC_SR_NRSTL) {
	};

	// Wait for PHY to be ready (CAT811: Max400ms)
	ul_delay = SystemCoreClock / 1000 / 3 * 400;
	while (ul_delay--);
	SerialUSB.println("PHY ready.");

	// Enable EMAC clock
	pmc_enable_periph_clk(ID_EMAC);

	// Fill in EMAC options
	emac_option.uc_copy_all_frame = 0;
	emac_option.uc_no_boardcast = 0;

	memcpy(emac_option.uc_mac_addr, gs_uc_mac_address, sizeof(gs_uc_mac_address));

	gs_emac_dev.p_hw = EMAC;
  
        SerialUSB.println("Init EMAC driver structure\r\n");
	// Init EMAC driver structure
	emac_dev_init(EMAC, &gs_emac_dev, &emac_option);

	// Enable Interrupt
	NVIC_EnableIRQ(EMAC_IRQn);
        
        SerialUSB.println("Init MAC PHY driver\r\n");

	// Init MAC PHY driver
	if (ethernet_phy_init(EMAC, BOARD_EMAC_PHY_ADDR, SystemCoreClock)
					!= EMAC_OK) {
		SerialUSB.println("PHY Initialize ERROR!\r\n");
		//return -1;
	}

	// Auto Negotiate, work in RMII mode
	if (ethernet_phy_auto_negotiate(EMAC, BOARD_EMAC_PHY_ADDR) != EMAC_OK) {

		SerialUSB.println("Auto Negotiate ERROR!\r\n");
		//return -1;
	}
        
        SerialUSB.println("Establish ethernet link");
	// Establish ethernet link
	while (ethernet_phy_set_link(EMAC, BOARD_EMAC_PHY_ADDR, 1) != EMAC_OK) {
          SerialUSB.println(EMAC_OK);
          SerialUSB.println("Set link ERROR!\r\n");
		//return -1;
	}
        
	while (1) {
		// Process packets
		if (EMAC_OK != emac_dev_read(&gs_emac_dev, (uint8_t *) gs_uc_eth_buffer,
						sizeof(gs_uc_eth_buffer), &ul_frm_size)) {
			continue;
		}

		if (ul_frm_size > 0) {
			// Handle input frame
			emac_process_eth_packet((uint8_t *) gs_uc_eth_buffer, ul_frm_size);
		}
	}
}
