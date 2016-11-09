/*
 * Copyright (C) 2016 Fundación Inria Chile
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     sys_hashes
 * @{
 *
 * @file
 * @brief       AES_CMAC implementation
 *
 * @author      José Ignacio Alamos <jose.alamos@inria.cl>
 *
 * @}
 */


#include <stdio.h>
#include "hashes/cmac.h"
#include <inttypes.h>
#include "crypto/ciphers.h"
#include <string.h>

#define XOR128(x,y) do { \
    uint8_t i; \
    for(i=0;i<16;i++) { \
        (y)[i] = x[i] ^ y[i]; \
    } \
} while(0)

#define MIN(a,b) a < b ? a : b
#define LEFTSHIFT(x, y) do { \
    uint8_t i; \
    for(i=0;i<15;i++) \
    { \
        (y)[i] = ((x)[i] << 1) | ((x)[i+1] >> 7); \
    } \
    (y)[15] = (x)[15] << 1; \
} while(0)

void cmac_init(cmac_context *ctx, const uint8_t *key, uint8_t keySize)
{
    //TODO: Add proper errors
    memset(ctx->X, 0, CMAC_BLOCK_SIZE);
    memset(ctx->M_last, 0, CMAC_BLOCK_SIZE);
    ctx->M_n = 0;
    cipher_init(&(ctx->aes_ctx), CIPHER_AES_128, key, keySize);
}


void cmac_update(cmac_context *ctx, const void *data, size_t len)
{
    uint8_t d[16];
    uint8_t c;
    while(len)
    {
        //Copy data until multiple of 128 bits
        c = MIN(CMAC_BLOCK_SIZE-ctx->M_n, len);
        memcpy(ctx->M_last+ctx->M_n, data, c);
        ctx->M_n += c;
        len -= c;
        data = (void*) (((uint8_t*) data) + c);
        if(ctx->M_n > 16) ctx->M_n = 0;

        //Exit in case no block is processed
        if(ctx->M_n < CMAC_BLOCK_SIZE)
            break;

        //A block was generated. Do AES and stuff
        XOR128(ctx->M_last, ctx->X);
        cipher_encrypt(&ctx->aes_ctx, ctx->X, d);
        memcpy(ctx->X, d, CMAC_BLOCK_SIZE);
    }
}

void cmac_final(cmac_context *ctx, void *digest)
{
    //Generate subkeys
    uint8_t K[CMAC_BLOCK_SIZE];
    uint8_t L[CMAC_BLOCK_SIZE];

    memset(K, 0, CMAC_BLOCK_SIZE);
    cipher_encrypt(&ctx->aes_ctx, K, L);

    if(L[0] & 0x80)
    {
        LEFTSHIFT(L, K);
        L[15] ^= 0x87;
    }
    else
    {
        LEFTSHIFT(L, K);
    }

    if(ctx->M_n != 16)
    {
        //Generate K2
        if(K[0] & 0x80)
        {
            LEFTSHIFT(K, K);
            K[15] ^= 0x87;
        }
        else
        {
            LEFTSHIFT(K, K);
        }
        //Padding goes here
        memset(ctx->M_last+ctx->M_n, 0, CMAC_BLOCK_SIZE-ctx->M_n);
        ctx->M_last[ctx->M_n] = 0x80;
    }
    XOR128(K, ctx->M_last);
    XOR128(ctx->M_last, ctx->X);
    cipher_encrypt(&ctx->aes_ctx, ctx->X, L);
    memcpy(digest, L, CMAC_BLOCK_SIZE);
}
