/*
 * Copyright (C) 2016 Fundación Inria Chile
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    lorawan_crypto
 * @brief       LoRaWAN
 * @{
 * @file
 * @brief       
 * @author      José Ignacio Alamos <jialamos@uc.cl>
 */

#ifndef LORAWAN_CRYPTO_H_
#define LORAWAN_CRYPTO_H_

#include <inttypes.h>
#ifdef __cplusplus
extern "C" {
#endif

void lorawan_calc_mic(const void *buf, size_t size, const uint8_t *key, uint32_t address, uint8_t dir, uint32_t seq_counter, uint32_t *mic);

void lorawan_payload_encrypt(void *buf, size_t size, uint8_t *key, uint32_t address, uint8_t dir, uint32_t seq_counter, void *enc_payload);

void lorawan_payload_decrypt(void *buf, size_t size, uint8_t *key, uint32_t address, uint8_t dir, uint32_t seq_counter, void *dec_payload);

void lorawan_join_calc_mic(void *buf, size_t size, uint8_t *key, void *mic);

void lorawan_join_decrypt(uint8_t *key, uint8_t *app_nonce, uint16_t dev_nonce, uint8_t n_session_key, uint8_t *app_session_key);

#ifdef __cplusplus
}  /* extern "C" */
#endif
#endif
