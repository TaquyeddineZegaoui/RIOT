/*
 * Copyright (C) Fundación Inria Chile 2017
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for
 * more details.
 */

/**
 * @ingroup     drivers_netdev_netdev2
 * @{
 *
 * @file
 * @brief       Definitions for netdev2 common lorawan code
 *
 * @author      José Ignacio Alamos <jose.alamos@inria.cl>
 */

#ifndef NETDEV2_ETH_H
#define NETDEV2_ETH_H

#include <stdint.h>

#include "net/netdev2.h"
#include "net/netopt.h"

#define LORAWAN_MCPS_REQUEST 1
#define LORAWAN_MCPS_IND 2
#define LORAWAN_MLME_REQUEST 4
#define LORAWAN_MAC_DONE 8

#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    netdev2_t netdev;                       /**< @ref netdev2_t base class */
    uint8_t flags;
} netdev2_lorawan_t;

/**
 * @brief   Fallback function for netdev2 lorawan devices' _get function
 *
 * Supposed to be used by netdev2 drivers as default case.
 *
 * @warning Driver *MUST* implement NETOPT_ADDRESS case!
 *
 * @param[in]   dev     network device descriptor
 * @param[in]   opt     option type
 * @param[out]  value   pointer to store the option's value in
 * @param[in]   max_len maximal amount of byte that fit into @p value
 *
 * @return              number of bytes written to @p value
 * @return              <0 on error
 */
int netdev2_lorawan_get(netdev2_t *dev, netopt_t opt, void *value, size_t max_len);

/**
 * @brief   Fallback function for netdev2 ethernet devices' _set function
 *
 * @param[in] dev       network device descriptor
 * @param[in] opt       option type
 * @param[in] value     value to set
 * @param[in] value_len the length of @p value
 *
 * @return              number of bytes used from @p value
 * @return              <0 on error
 */
int netdev2_lorawan_set(netdev2_t *dev, netopt_t opt, void *value, size_t value_len);

#ifdef __cplusplus
}
#endif
/** @} */
#endif /* NETDEV2_ETH_H */
