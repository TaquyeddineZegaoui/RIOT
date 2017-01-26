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

int netdev2_lorawan_get(netdev2_t *dev, netopt_t opt, void *value, size_t max_len)
{
    return 0;
}

int netdev2_lorawan_set(netdev2_t *dev, netopt_t opt, void *value, size_t value_len)
{
    return 0;
}
