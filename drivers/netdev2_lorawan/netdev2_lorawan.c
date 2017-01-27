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
#include <string.h>
#include <errno.h>
#include "net/netdev2/lorawan.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

int netdev2_lorawan_get(netdev2_lorawan_t *dev, netopt_t opt, void *value, size_t max_len)
{
    return 0;
}

int netdev2_lorawan_set(netdev2_lorawan_t *dev, netopt_t opt, void *value, size_t value_len)
{
    switch (opt) {
        case NETOPT_ADDRESS:
        {
            assert(value_len == sizeof(uint32_t));
            uint32_t dev_addr = *((uint32_t *)value);
            /* real validity needs to be checked by device, since sub-GHz and
             * 2.4 GHz band radios have different legal values. Here we only
             * check that it fits in an 8-bit variabl*/
            assert(dev_addr <= UINT32_MAX);
            dev->lorawan.dev_addr = dev_addr;
            return sizeof(uint16_t);
        }
        case NETOPT_LORAWAN_NWK_SKEY:
        {
            assert(value_len == KEY_SIZE);
            memcpy(dev->lorawan.nwk_skey, value, value_len);
            return sizeof(uint16_t);
        }
        case NETOPT_LORAWAN_NWK_AKEY:
        {
            assert(value_len == KEY_SIZE);
            memcpy(dev->lorawan.app_skey, value, value_len);
            return sizeof(uint16_t);
        }
        case NETOPT_LORAWAN_NET_ID:
        {
            assert(value_len == sizeof(uint32_t));
            uint32_t net_id = *((uint32_t *)value);
            /* real validity needs to be checked by device, since sub-GHz and
             * 2.4 GHz band radios have different legal values. Here we only
             * check that it fits in an 8-bit variabl*/
            assert(net_id <= UINT32_MAX);
            dev->lorawan.net_id  = net_id ;
            return sizeof(uint16_t);
        }
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
