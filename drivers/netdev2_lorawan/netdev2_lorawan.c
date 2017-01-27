/*
 * Copyright (C) Fundación Inria Chile 2017
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     driver_netdev2_eth
 * @{
 *
 * @file
 * @brief       Common code for LoRa(WAN) drivers
 *
 * @author      José Ignacio Alamos <jose.alamos@inria.cl>
 *
 * @}
 */

#include <assert.h>
#include <errno.h>
#include "net/netdev2/lorawan.h"
#include "LoRaMac-api-v3.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

int netdev2_lorawan_get(netdev2_lorawan_t *dev, netopt_t opt, void *value, size_t max_len)
{
    switch(opt)
    {
        case NETOPT_LORAWAN_JOIN:
            *((uint8_t*) value) = LoRaMacJoinReq(dev->dev_eui, dev->app_eui, dev->app_key);
            break;
        default:
            break;
    }
    return 0;
}

int netdev2_lorawan_set(netdev2_lorawan_t *dev, netopt_t opt, void *value, size_t value_len)
{
    switch(opt)
    {
        case NETOPT_LORAWAN_DEV_EUI:
            dev->dev_eui = (uint8_t*) value;
            return sizeof(uint8_t*);
        case NETOPT_LORAWAN_APP_EUI:
            dev->app_eui = (uint8_t*) value;
            return sizeof(uint8_t*);
        case NETOPT_LORAWAN_APP_KEY:
            dev->app_key = (uint8_t*) value;
            return sizeof(uint8_t*);
        default:
            break;
    }
    return 0;
}
