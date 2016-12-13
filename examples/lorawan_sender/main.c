/*
 * Copyright (C) 2008, 2009, 2010  Kaspar Schleiser <kaspar@schleiser.de>
 * Copyright (C) 2013 INRIA
 * Copyright (C) 2013 Ludwig Knüpfer <ludwig.knuepfer@fu-berlin.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       Default application that shows a lot of functionality of RIOT
 *
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @author      Oliver Hahm <oliver.hahm@inria.fr>
 * @author      Ludwig Knüpfer <ludwig.knuepfer@fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>
#include <string.h>

#include "shell.h"
#include "shell_commands.h"

#include "net/lorawan/hdr.h"
#include "net/lorawan/crypto.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "net/netdev2.h"
#include "sx1276_internal.h"
#include "sx1276_params.h"
#include "sx1276_netdev.h"
#include "net/gnrc/netdev2.h"

#define LORAWAN_MTYPE_JOIN_REQUEST 0
#define LORAWAN_MTYPE_JOIN_ACCEPT 1
#define LORAWAN_MTYPE_UNCONFIRMED_DATA_UP 2
#define LORAWAN_MTYPE_UNCONFIRMED_DATA_DOWN 3
#define LORAWAN_MTYPE_CONFIRMED_DATA_UP 4
#define LORAWAN_MTYPE_CONFIRMED_DATA_DOWN 5

#define LORAWAN_MAJOR_VERSION 0

#define LORAWAN_MASK_ADR 0x80
#define LORAWAN_MASK_ADRACKREG 0x40
#define LORAWAN_MASK_ACK 0x20
#define LORAWAN_MASK_FOPTSLEN 0x07

#define DEV_ADDRESS 0x166fb58e
#define GNRC_LORA_MSG_QUEUE 16

static netdev2_t *nd;
sx1276_t sx1276;

static const uint8_t key[] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3A };

static uint8_t pktbuf[64];

void make_lorawan_hdr(uint8_t mtype, uint8_t maj, uint32_t address, uint8_t fctrl, uint16_t fcntr, uint8_t fport)
{
    /* Fill header */
    lw_hdr_t *hdr = (lw_hdr_t*) pktbuf;
    
    lw_hdr_set_mtype(hdr, mtype);
    lw_hdr_set_maj(hdr, maj);

    hdr->addr = byteorder_btoll(byteorder_htonl(address));
    
    hdr->fctrl = fctrl;
    hdr->fcnt = byteorder_btols(byteorder_htons(fcntr));

    /* Since there aren't options, copy port and payload */
    *(pktbuf+sizeof(lw_hdr_t)) = fport;
}

#define SX1276_MAC_STACKSIZE    (THREAD_STACKSIZE_DEFAULT)
static char stack[SX1276_MAC_STACKSIZE];

int send_unconfirmed_pkt(int argc, char **argv)
{
    (void) argc;
    uint16_t counter = atoi(argv[1]);
    uint8_t fport = atoi(argv[2]);

    make_lorawan_hdr(LORAWAN_MTYPE_UNCONFIRMED_DATA_UP, LORAWAN_MAJOR_VERSION, DEV_ADDRESS, 0, counter, fport);

    //copy payload
    uint8_t pktlen = strlen(argv[3]);
    memcpy(sizeof(lw_hdr_t)+pktbuf+1, argv[3], pktlen);

    //calculate MIC
    uint32_t mic;
    uint32_t mic_pos = sizeof(lw_hdr_t)+1+pktlen;
    printf("%i\n", (int) mic_pos);
    lorawan_calc_mic(pktbuf, mic_pos, key, DEV_ADDRESS, 0, counter, &mic);
    
    //copy MIC to pkt
    pktbuf[mic_pos+3] = mic & 0xff;
    pktbuf[mic_pos+2] = (mic >> 8) & 0xff;
    pktbuf[mic_pos+1] = (mic >> 16) & 0xff;
    pktbuf[mic_pos] = (mic >> 24) & 0xff;

    for(unsigned int i=0;i<mic_pos+4;i++)
    {
        printf("%02x ", pktbuf[i]);
    }
    printf("\n");

    uint32_t channel = 904500000;
    nd->driver->set(nd, NETOPT_CHANNEL, &channel, sizeof(uint32_t));

    struct iovec vec[1];
    vec[0].iov_base = pktbuf;
    vec[0].iov_len = mic_pos+4;

    nd->driver->send(nd, vec, 1); 
 
    xtimer_usleep(10000); /* wait for the chip */

    puts("tx_test: sended");
    return 0;
}

int lora_setup(int argc, char **argv) {
	if (argc < 4) {
		return -1;
	}

	int bw = atoi(argv[1]);
	int sf = atoi(argv[2]);
	int cr = atoi(argv[3]);

	sx1276_lora_bandwidth_t lora_bw;

	switch (bw) {
	case 125:
		lora_bw = SX1276_BW_125_KHZ;
		break;

	case 250:
		lora_bw = SX1276_BW_250_KHZ;
		break;

	case 500:
		lora_bw = SX1276_BW_500_KHZ;
		break;

	default:
		puts("lora_setup invalid bandwidth value passed");
		return -1;
	}

	sx1276_lora_spreading_factor_t lora_sf;
	if (sf < 7 || sf > 12) {
		puts("lora_setup: invalid spreading factor value passed");
		return -1;
	}

	lora_sf = (sx1276_lora_spreading_factor_t) sf;

	sx1276_lora_coding_rate_t lora_cr;
	if (cr < 5 || cr > 8) {
		puts("lora_setup: invalid coding rate value passed");
		return -1;
	}

	lora_cr = (sx1276_lora_coding_rate_t) (cr - 5);

	sx1276_configure_lora_bw(&sx1276, lora_bw);
	sx1276_configure_lora_sf(&sx1276, lora_sf);
	sx1276_configure_lora_cr(&sx1276, lora_cr);

	puts("lora_setup: configuration is set");

	return 0;
}

static const shell_command_t shell_commands[] = {
    { "send_unc_packet", "send unconfirmed packet", send_unconfirmed_pkt },
	{ "lora_setup", "<BW (125, 250, 512)> <SF (7..12)> <CR 4/(5,6,7,8)> - sets up LoRa modulation settings", lora_setup},
    { NULL, NULL, NULL }
};

void *_event_loop(void *arg)
{
    static msg_t _msg_q[GNRC_LORA_MSG_QUEUE];
    msg_t msg;
    netdev2_t *netdev = (netdev2_t*) arg;
    msg_init_queue(_msg_q, GNRC_LORA_MSG_QUEUE);

    while (1) {
        msg_receive(&msg);
        switch (msg.type) {
            case NETDEV2_MSG_TYPE_EVENT:
                netdev->driver->isr(netdev);
                break;
            default:
                break;
        }
    }
    return NULL;
}
static void _event_cb(netdev2_t *dev, netdev2_event_t event)
{
    msg_t msg;
    msg.type = NETDEV2_MSG_TYPE_EVENT;
    kernel_pid_t *pid = (kernel_pid_t*) dev->context;
    switch(event)
    {
        case NETDEV2_EVENT_ISR:
            msg_send(&msg, *pid);
            break;
        case NETDEV2_EVENT_TX_COMPLETE:
            printf("TX DONE\n");
            break;
#if 0
        case NETDEV2_EVENT_RX_COMPLETE:
            len = dev->driver->recv(dev, NULL, 5, &rx_info);
            dev->driver->recv(dev, message, len, NULL);
            printf("%s\n. {RSSI: %i\n", message, rx_info.rssi);
#endif
        default:
            break;
    }
}
int main(void)
{
    xtimer_init();

    memcpy(&sx1276.params, sx1276_params, sizeof(sx1276_params));
    netdev2_t *netdev = (netdev2_t*) &sx1276;
    nd = netdev;
    netdev->driver = &sx1276_driver;
    netdev->driver->init(netdev);
    netdev->event_callback = _event_cb;

    kernel_pid_t pid;
    pid = thread_create(stack, sizeof(stack), THREAD_PRIORITY_MAIN - 5, THREAD_CREATE_STACKTEST,
                     _event_loop, (void *) netdev, "asd");
    netdev->context = &pid;
    (void) puts("Welcome to RIOT!");

    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
