/*
 * Copyright (C) 2016 Fundación Inria Chile
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_sx1276
 * @{
 * @file
 * @brief       Netdev2 adoption for the sx1276 driver
 *
 * @author      José Ignacio Alamos <jose.alamos@inria.cl>
 * @}
 */

#include "net/netdev2.h"

static int _send(netdev2_t *netdev, const struct iovec *vector, unsigned count);
static int _recv(netdev2_t *netdev, void *buf, size_t len, void *info);
static int _init(netdev2_t *netdev);
static void _isr(netdev2_t *netdev);
static int _get(netdev2_t *netdev, netopt_t opt, void *val, size_t max_len);
static int _set(netdev2_t *netdev, netopt_t opt, void *val, size_t len);

const netdev2_driver_t at86rf2xx_driver = {
    .send = _send,
    .recv = _recv,
    .init = _init,
    .isr = _isr,
    .get = _get,
    .set = _set,
};

static int _init(netdev2_t *netdev)
{
    sx1276_t *sx1276 = (sx1276_t*) netdev;

    sx1276_settings_t settings;
    settings->channel = RF_FREQUENCY;
    settings->modem = SX1276_MODEM_LORA;
    settings->state = SX1276_RF_IDLE;

    sx1276->settings = settings;

    sx1276->sx1276_event_cb = event_handler_thread;

    /* Launch initialization of driver and device */
    puts("init_radio: initializing driver...");
    sx1276_init(&sx1276);

    /* Configure the device */
    init_configs();

    /* Put chip into sleep */
    sx1276_set_sleep(&sx1276);

    puts("init_radio: sx1276 initialization done");

    sx1276_lora_settings_t settings;

    settings.bandwidth = SX1276_BW_125_KHZ;
    settings.coderate = SX1276_CR_4_5;
    settings.datarate = SX1276_SF12;
    settings.crc_on = true;
    settings.freq_hop_on = false;
    settings.hop_period = 0;
    settings.implicit_header = false;
    settings.iq_inverted = false;
    settings.low_datarate_optimize = false;
    settings.payload_len = 0;
    settings.power = 14;
    settings.preamble_len = LORA_PREAMBLE_LENGTH;
    settings.rx_continuous = true;
    settings.tx_timeout = 1000 * 1000 * 30; // 30 sec
    settings.rx_timeout = LORA_SYMBOL_TIMEOUT;

    sx1276_configure_lora(&sx1276, &settings);

    sx1276_set_channel(&sx1276, 868500000);
    return 0;
}
