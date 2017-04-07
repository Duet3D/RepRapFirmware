/****************************************************************//**
 *
 * @file mdns_responder.c
 *
 * @author   Logan Gunthorpe <logang@deltatee.com>
 *
 * @brief    mdns service discovery
 *           reqirues SO_REUSE=1 and LWIP_IGMP=1
 *
 * Copyright (c) Deltatee Enterprises Ltd. 2013
 * All rights reserved.
 *
 ********************************************************************/

/* 
 * Redistribution and use in source and binary forms, with or without
 * modification,are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Logan Gunthorpe <logang@deltatee.com>
 *
 */

#include "mdns_responder.h"

#include "lwipopts.h"
#include <lwip/src/include/lwip/udp.h>
#include <lwip/src/include/ipv4/lwip/igmp.h>
#include <lwip/src/include/lwip/debug.h>
#include <lwip/src/include/lwip/mem.h>

#ifndef MDNS_DEBUG
#define MDNS_DEBUG LWIP_DBG_OFF
#endif

#ifndef MDNS_PORT
#define MDNS_PORT 5353
#endif

#include <stdio.h>
#include <string.h>
#include <stdlib.h>			// for strtol

#if 1	//dc42
#include <stdbool.h>
#endif

#ifndef ip_set_option
#define ip_set_option(pcb, opt)   ((pcb)->so_options |= (opt))
#endif
extern struct netif gs_net_if;


#define FLAG_RESP   0x8400

const char *all_services = "\x09_services\x07_dns-sd\x04_udp\x05local";

#define QCLASS_IN   0x0001
#define CACHE_FLUSH 0x8000

#define QTYPE_A     0x0001
#define QTYPE_PTR   0x000C
#define QTYPE_TXT   0x0010
#define QTYPE_SRV   0x0021

#define DATA_POINTER 0xC000

#define TTL          10*60

static struct mdns_state mdns_state;

static const char dotlocal[] = "\x05local";
static const int dotlocal_len = sizeof(dotlocal);

struct mdns_header {
    uint16_t id;
    uint16_t flags;
    uint16_t questions;
    uint16_t answers;
    uint16_t authorities;
    uint16_t additionals;
};

struct record {
    uint16_t qtype;
    uint16_t qclass;
    uint32_t ttl;
    uint16_t data_length;
    char data[];
} __attribute__((__packed__));

struct srv_record {
    uint16_t priority;
    uint16_t weight;
    uint16_t port;
    char target[];
} __attribute__((__packed__));

struct mdns_state {
    char *hostnames[4];
    char *service_host;
    const struct mdns_service *services;
    int num_services;
    char *txt_records;
    struct udp_pcb *sendpcb;
    struct netif *netif;
};

static struct pbuf *populate_header(int answers, int authorities,
                                    int additionals)
{
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, sizeof(struct mdns_header),
                                PBUF_RAM);

    struct mdns_header *hdr = (struct mdns_header *) p->payload;
    hdr->id = 0;
    hdr->flags = htons(FLAG_RESP);
    hdr->questions = 0;
    hdr->answers = htons(answers);
    hdr->authorities = htons(authorities);
    hdr->additionals = htons(additionals);

    return p;
}

static int special_strlen(const char *name)
{
    int ret = 0;

    while (1) {
        ret++;
        int x = *name++;
        if (x == 0)
            return ret;
        if (x & 0xC0)
            return ret + 1;

        ret +=x;
        name += x;
    }
}

static int special_strcpy(char * dest, const char *name,
                          const struct pbuf *p)
{
    int ret = 0;
    int link_ret = -1;

    const char *end = ((char *) p->payload) + p->len;

    while (1) {
        int x = *name++;
        ret++;

        if (name > end) {
            *dest = 0;
            return -1;
        }

        if (x & 0xC0) {
            ret++;
            if (link_ret < 0) link_ret = ret;
            x = (x << 8) | *name++;
            name = &((char *)p->payload)[x & 0x3FFF];
            continue;
        }

        *dest++ = x;

        if (x == 0)
            return (link_ret >= 0) ? link_ret : ret;

        ret +=x;
        memcpy(dest, name, x);
        name += x;
        dest += x;
    }
}

static uint16_t populate_record(const char *name,
                                uint16_t qtype, uint16_t qclass,
                                uint32_t ttl, const void *data,
                                uint16_t datalen,
                                struct pbuf *dest)
{
    int title_len = special_strlen(name);
    int msglen = title_len  + sizeof(struct record) + datalen;
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, msglen, PBUF_RAM);

    memcpy(p->payload, name, title_len);
    char *end = ((char *)p->payload) + title_len;

    struct record *rec = (struct record *) end;
    rec->qtype = htons(qtype);
    rec->qclass = htons(qclass);
    rec->ttl = htonl(ttl);
    rec->data_length = htons(datalen);
    memcpy(rec->data, data, datalen);

    uint16_t ret = dest->tot_len + title_len + sizeof(*rec);

    pbuf_cat(dest, p);

    return htons(DATA_POINTER | ret);
}

static void send_a_record(struct mdns_state *ms, const char *name)
{
    struct pbuf *hdr = populate_header(1, 0, 0);

    populate_record(name, QTYPE_A, QCLASS_IN,
                    TTL, &ms->netif->ip_addr,
                    sizeof(ms->netif->ip_addr),
                    hdr);

    udp_send(ms->sendpcb, hdr);

    LWIP_DEBUGF(MDNS_DEBUG | LWIP_DBG_STATE,
                ("mdns: sending A response\n"));

    pbuf_free(hdr);
}

static void send_all_services(struct mdns_state *ms, const char *name)
{
    struct pbuf *hdr = populate_header(ms->num_services, 0, 0);

    uint16_t first_ptr = htons(DATA_POINTER | hdr->len);

    for (int i = 0; i < ms->num_services; i++) {
        if (i != 0)
            name = (char *) &first_ptr;
        populate_record(name, QTYPE_PTR, QCLASS_IN,
                        TTL, ms->services[i].name,
                        strlen(ms->services[i].name)+1,
                        hdr);
    }

    udp_send(ms->sendpcb, hdr);

    LWIP_DEBUGF(MDNS_DEBUG | LWIP_DBG_STATE,
                ("mdns: sending ALL response\n"));

    pbuf_free(hdr);

}

static void send_ptr_record(struct mdns_state *ms, const char *domain,
                            int service)
{
    struct pbuf *hdr = populate_header(1, 0, 3);

    uint16_t first_ptr = htons(DATA_POINTER | hdr->len);

    //PTR Record
    int service_len = strlen(ms->service_host);
    char service_name[service_len + 2];
    strcpy(service_name, ms->service_host);
    memcpy(&service_name[service_len], &first_ptr, sizeof(first_ptr));
    uint16_t srv_ptr = populate_record(domain, QTYPE_PTR, QCLASS_IN,
                                       TTL, service_name,
                                       sizeof(service_name),
                                       hdr);

    //SRV Record
    char buf[sizeof(struct srv_record) + service_len + dotlocal_len];
    struct srv_record *srv_rec = (struct srv_record *) buf;
    srv_rec->priority = htons(50);
    srv_rec->weight = htons(0);
    srv_rec->port = htons(ms->services[service].port);
    memcpy(srv_rec->target, ms->service_host, service_len);
    memcpy(srv_rec->target + service_len, dotlocal, dotlocal_len);
    uint16_t arec_ptr = populate_record((char *)&srv_ptr, QTYPE_SRV,
                                        QCLASS_IN, TTL,
                                        buf, sizeof(buf),
                                        hdr);

    arec_ptr = htons(ntohs(arec_ptr) + sizeof(struct srv_record));

    //TXT Record
    populate_record((char *) &srv_ptr, QTYPE_TXT,
                    QCLASS_IN, TTL,
                    ms->txt_records, strlen(ms->txt_records) + 1,
                    hdr);

    //A Record
    populate_record((char *) &srv_ptr, QTYPE_A,
                    QCLASS_IN, TTL,
                    &ms->netif->ip_addr,
                    sizeof(ms->netif->ip_addr),
                    hdr);

    udp_send(ms->sendpcb, hdr);

    LWIP_DEBUGF(MDNS_DEBUG | LWIP_DBG_STATE,
                ("mdns: sending PTR response\n"));

    pbuf_free(hdr);
}

static void send_srv_record(struct mdns_state *ms, const char *domain,
                            int service)
{
    struct pbuf *hdr = populate_header(1, 0, 2);

    uint16_t first_ptr = htons(DATA_POINTER | hdr->len);

    //SRV Record
    int service_len = strlen(ms->service_host);
    char buf[sizeof(struct srv_record) + service_len + dotlocal_len];
    struct srv_record *srv_rec = (struct srv_record *) buf;
    srv_rec->priority = htons(50);
    srv_rec->weight = htons(0);
    srv_rec->port = htons(ms->services[service].port);
    memcpy(srv_rec->target, ms->service_host, service_len);
    memcpy(srv_rec->target + service_len, dotlocal, dotlocal_len);
    uint16_t arec_ptr = populate_record(domain, QTYPE_SRV,
                                        QCLASS_IN, TTL,
                                        buf, sizeof(buf),
                                        hdr);

    arec_ptr = htons(ntohs(arec_ptr) + sizeof(struct srv_record));

    //TXT Record
    const char txt[1] = "";
    populate_record((char *) &first_ptr, QTYPE_TXT,
                    QCLASS_IN, TTL,
                    txt, sizeof(txt),
                    hdr);

    //A Record
    populate_record((char *) &arec_ptr, QTYPE_A,
                    QCLASS_IN, TTL,
                    &ms->netif->ip_addr,
                    sizeof(ms->netif->ip_addr),
                    hdr);

    udp_send(ms->sendpcb, hdr);

    LWIP_DEBUGF(MDNS_DEBUG | LWIP_DBG_STATE,
                ("mdns: sending SRV response\n"));

    pbuf_free(hdr);
}

static void send_rev_record(struct mdns_state *ms, const char *domain)
{
    struct pbuf *hdr = populate_header(1, 0, 0);

    int service_len = strlen(ms->service_host);
    char service_name[service_len + dotlocal_len];
    strcpy(service_name, ms->service_host);
    strcat(service_name, dotlocal);
    populate_record(domain, QTYPE_PTR, QCLASS_IN,
                    TTL, service_name,
                    sizeof(service_name),
                    hdr);

    udp_send(ms->sendpcb, hdr);

    LWIP_DEBUGF(MDNS_DEBUG | LWIP_DBG_STATE,
                ("mdns: sending REV PTR response\n"));

    pbuf_free(hdr);
}

static int query_hostname(struct mdns_state *ms, const char *domain)
{
    for (int i = 0; i < sizeof(ms->hostnames) / sizeof(*ms->hostnames); i++) {
        if (strcasecmp(domain, ms->hostnames[i]) == 0)
            return 1;
    }

    return 0;
}

static int query_ptr(struct mdns_state *ms, const char *domain)
{
    for (int i = 0; i < ms->num_services; i++) {
        if (strcasecmp(domain, ms->services[i].name) == 0)
            return i;
    }

    return -1;
}

static int query_service(struct mdns_state *ms, const char *domain)
{
    int hostlen = strlen(ms->service_host);

    if (!strncasecmp(domain, ms->service_host, hostlen))
        return -1;

    return query_ptr(ms, &domain[hostlen]);
}

static int compare_reverse_ptr(struct mdns_state *ms, char *buf)
{
    buf++;
    if (strtol(buf, &buf, 10) != ip4_addr4(&ms->netif->ip_addr))
        return 0;

    buf++;
    if (strtol(buf, &buf, 10) != ip4_addr3(&ms->netif->ip_addr))
        return 0;

    buf++;
    if (strtol(buf, &buf, 10) != ip4_addr2(&ms->netif->ip_addr))
        return 0;

    buf++;
    if (strtol(buf, &buf, 10) != ip4_addr1(&ms->netif->ip_addr))
        return 0;

    if(strcmp(buf, "\x07in-addr\04arpa") == 0)
        return 1;

    return 0;
}

static int parse_question(struct mdns_state *ms, struct udp_pcb *upcb,
                          char *data, int qlen, struct pbuf *p)
{
    int offset = 0;

    char buf[255];
    offset = special_strcpy(buf, data, p);

    if (offset < 0)
        return -1;

    int qtype = (data[offset] << 8) | data[offset+1];
    offset += 2;
    int qclass = (data[offset] << 8) | data[offset+1];
    (void)qclass;		// dc42 suppress 'unused' warning when debug is disabled

    offset += 2;

    LWIP_DEBUGF(MDNS_DEBUG | LWIP_DBG_STATE,
                ("mdns: question '%s' type %d class %d\n",
                    &buf[1], qtype, qclass));

    if (qtype == QTYPE_A) {
        if (query_hostname(ms, buf))
            send_a_record(ms, buf);
    } else if (qtype == QTYPE_PTR) {
        if (strcasecmp(all_services, buf) == 0) {
            send_all_services(ms, buf);
        } else if (compare_reverse_ptr(ms, buf)) {
            send_rev_record(ms, buf);
        } else {
            int service;
            if ((service = query_ptr(ms, buf)) >= 0)
                send_ptr_record(ms, buf, service);
        }
    } else if (qtype == QTYPE_SRV || qtype == QTYPE_TXT) {
        int service;
        if ((service = query_service(ms, buf)) >= 0)
            send_srv_record(ms, buf, service);
    }

    return offset;
}

static void recv(void *arg, struct udp_pcb *upcb, struct pbuf *p,
                 ip_addr_t *addr, u16_t port)
{
    struct mdns_state *ms = (struct mdns_state *) arg;
    struct mdns_header *h = (struct mdns_header *) p->payload;
    char *questions = (char *) &h[1];
    int qlen = p->len - sizeof(*h);

    h->flags = ntohs(h->flags);
    h->questions = ntohs(h->questions);

    LWIP_DEBUGF(MDNS_DEBUG | LWIP_DBG_STATE,
                ("mdns: packet from "));
    ip_addr_debug_print(MDNS_DEBUG | LWIP_DBG_STATE, addr);
    LWIP_DEBUGF(MDNS_DEBUG | LWIP_DBG_STATE,
                (" %04x %04x\n", h->flags, h->questions));

    if (h->id != 0 || h->questions == 0)
        goto free_and_return;

    for (int i = 0; i < h->questions; i++) {
        int offset = parse_question(ms, upcb, questions, qlen, p);

        if (offset < 0)
            goto free_and_return;

        questions += offset;
        qlen -= offset;
        if (qlen <= 0)
            break;
    }

free_and_return:
    pbuf_free(p);
}


static void free_hostnames(struct mdns_state *ms)
{
    for (int i = 0; i < sizeof(ms->hostnames) / sizeof(*ms->hostnames); i++) {
        if (ms->hostnames[i] != NULL) {
            mem_free(ms->hostnames[i]);
            ms->hostnames[i] = NULL;
        }
    }

    if (ms->service_host != NULL) {
        mem_free(ms->service_host);
        ms->service_host = NULL;
    }
}

static void setup_hostnames(struct mdns_state *ms, struct netif *netif)
{
    int hostlen = strlen(netif->hostname);

    ms->hostnames[0] = mem_malloc(hostlen + dotlocal_len + 1);
    sprintf(ms->hostnames[0], "%c%s%s", hostlen, netif->hostname,
            dotlocal);

    ms->hostnames[1] = mem_malloc(hostlen + 2 + dotlocal_len + 2);
    sprintf(ms->hostnames[1], "%c%s-%02X%s", hostlen+3, netif->hostname,
            netif->hwaddr[5], dotlocal);

    char macaddr[12];
    sprintf(macaddr, "%02X%02X%02X%02X%02X%02X",
            netif->hwaddr[0], netif->hwaddr[1], netif->hwaddr[2],
            netif->hwaddr[3], netif->hwaddr[4], netif->hwaddr[5]);
    int maclen = strlen(macaddr);

    ms->hostnames[2] = mem_malloc(hostlen + maclen + dotlocal_len + 2);
    sprintf(ms->hostnames[2], "%c%s-%s%s", maclen+hostlen+1, netif->hostname,
            macaddr, dotlocal);

    ms->hostnames[3] = mem_malloc(strlen(macaddr) + dotlocal_len + 1);
    sprintf(ms->hostnames[3], "%c%s%s", maclen, macaddr, dotlocal);

    for (int i = 0; i < sizeof(ms->hostnames) / sizeof(*ms->hostnames); i++) {
        LWIP_DEBUGF(MDNS_DEBUG | LWIP_DBG_STATE,
                    ("mdns: hostname registered: %s\n",
                     &ms->hostnames[i][1]));
    }

    ms->service_host = mem_malloc(hostlen + maclen + 3);
    sprintf(ms->service_host, "%c%s-%s", maclen+hostlen+1, netif->hostname,
            macaddr);

}

static void setup_txt_records(struct mdns_state *ms, const char *txt_records[])
{
    int totlen = 0;

    for (const char **rec = txt_records; *rec != NULL; rec++)
        totlen += 1 + strlen(*rec);

    ms->txt_records = mem_malloc(totlen+1);

    char *t = ms->txt_records;
    for (const char **rec = txt_records; *rec != NULL; rec++) {
        int l = strlen(*rec);
        *t++ = l;
        strcpy(t, *rec);
        t += l;
    }

}

err_t mdns_responder_init(const struct mdns_service *services,
                          int num_services,
                          const char *txt_records[])
{
#if 1	// dc42 changes to allow the set of services to be changed after initialisation
	static bool initialised = false;
	if (initialised)
	{
	    mdns_state.services = services;
	    mdns_state.num_services = num_services;
	    // Ideally we would also update the text records again here, but RRF never changes them
	    return ERR_OK;
	}
	initialised = true;
#endif

    err_t ret;

    memset(&mdns_state, 0, sizeof(mdns_state));
    mdns_state.netif = &gs_net_if;

    setup_hostnames(&mdns_state, &gs_net_if);
    setup_txt_records(&mdns_state, txt_records);

    mdns_state.services = services;
    mdns_state.num_services = num_services;

    struct ip_addr ipgroup;
    IP4_ADDR(&ipgroup, 224, 0, 0, 251);

    mdns_state.sendpcb = udp_new();
    if (mdns_state.sendpcb == NULL)
        return ERR_MEM;

    struct udp_pcb *pcb = udp_new();
    if (pcb == NULL) {
        udp_remove(mdns_state.sendpcb);
        return ERR_MEM;
    }

    if ((ret = igmp_joingroup(IP_ADDR_ANY, &ipgroup)) != ERR_OK)
        return ret;

    ip_set_option(pcb, SOF_REUSEADDR);
    ip_set_option(mdns_state.sendpcb, SOF_REUSEADDR);

    if ((ret = udp_bind(pcb, IP_ADDR_ANY, MDNS_PORT)) != ERR_OK)
        goto error_exit;

    udp_recv(pcb, recv, &mdns_state);

    if ((ret = udp_bind(mdns_state.sendpcb, 0, MDNS_PORT)) != ERR_OK)
        goto error_exit;

    if ((ret = udp_connect(mdns_state.sendpcb, &ipgroup, MDNS_PORT)) != ERR_OK)
        goto error_exit;

    return ERR_OK;

error_exit:
    udp_remove(pcb);
    udp_remove(mdns_state.sendpcb);
    return ret;

}

void mdns_update_hostname()
{
    free_hostnames(&mdns_state);
    setup_hostnames(&mdns_state, &gs_net_if);
}

void mdns_announce()
{
    struct mdns_state *ms = &mdns_state;
    int count = sizeof(ms->hostnames) / sizeof(*ms->hostnames);
    struct pbuf *hdr = populate_header(count, 0, 0);

    for (int i = 0; i < count; i++) {
        populate_record(ms->hostnames[i], QTYPE_A, QCLASS_IN | CACHE_FLUSH,
                        TTL, &ms->netif->ip_addr,
                        sizeof(ms->netif->ip_addr),
                        hdr);
    }

    udp_send(ms->sendpcb, hdr);

    LWIP_DEBUGF(MDNS_DEBUG | LWIP_DBG_STATE,
                ("mdns: sending announcment\n"));

    pbuf_free(hdr);
}
