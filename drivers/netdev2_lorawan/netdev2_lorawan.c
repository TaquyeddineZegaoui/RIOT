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

#define ENABLE_DEBUG (0)
#include "debug.h"

int netdev2_lorawan_get(netdev2_lorawan_t *dev, netopt_t opt, void *value, size_t max_len)
{
    return 0;
}

int netdev2_lorawan_set(netdev2_lorawan_t *dev, netopt_t opt, void *value, size_t value_len)
{
	int res = -ENOTSUP;

    switch (opt) {
        case NETOPT_ADDRESS:
        {
            assert(value_len == sizeof(uint32_t));
            uint32_t dev_addr = *((uint32_t *)value);
            /* real validity needs to be checked by device, since sub-GHz and
             * 2.4 GHz band radios have different legal values. Here we only
             * check that it fits in an 8-bit variabl*/
            assert(dev_addr <= UINT32_MAX);
            dev->lorawan.lorawan_dev_addr = dev_addr;
            res = sizeof(uint16_t);
            break;
        }
        case NETOPT_LORAWAN_NWK_SKEY:
        {
            assert(value_len == KEY_SIZE);
            uint32_t dev_addr = *((uint32_t *)value);
            /* real validity needs to be checked by device, since sub-GHz and
             * 2.4 GHz band radios have different legal values. Here we only
             * check that it fits in an 8-bit variabl*/
            assert(dev_addr <= UINT32_MAX);
            dev->lorawan.dev_addr = dev_addr;
            res = sizeof(uint16_t);
            break;
        }
        case NETOPT_LORAWAN_NWK_AKEY:
        {
            assert(value_len == KEY_SIZE);
            uint32_t net_id = *((uint32_t *)value);
            /* real validity needs to be checked by device, since sub-GHz and
             * 2.4 GHz band radios have different legal values. Here we only
             * check that it fits in an 8-bit variabl*/
            assert(dev_addr <= UINT32_MAX);
            dev->lorawan.net_id  = net_id ;
            res = sizeof(uint16_t);
            break;
        }
        case NETOPT_LORAWAN_NET_ID:
        {
            assert(value_len == sizeof(uint32_t));
            uint32_t net_id = *((uint32_t *)value);
            /* real validity needs to be checked by device, since sub-GHz and
             * 2.4 GHz band radios have different legal values. Here we only
             * check that it fits in an 8-bit variabl*/
            assert(dev_addr <= UINT32_MAX);
            dev->lorawan.net_id  = net_id ;
            res = sizeof(uint16_t);
            break;
        }
        default:
            break;
    }
    return res;
}
