#include <stdio.h>
#include "net/lorawan/crypto.h"
#include "hashes/cmac.h"

#define INITIAL_BLOCK {0x49, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}

void lorawan_calc_mic(const void *buf, size_t size, const uint8_t *key, uint32_t address, uint8_t dir, uint32_t seq_counter, uint32_t *mic)
{
    uint8_t block[16] = INITIAL_BLOCK;
    uint8_t out[16];
    block[5] = dir;

    block[6] = address & 0xff;
    block[7] = (address >> 8) & 0xff;
    block[8] = (address >> 16) & 0xff;
    block[9] = (address >> 24) & 0xff;

    block[10] = seq_counter & 0xff;
    block[11] = (seq_counter >> 8) & 0xff;
    block[12] = (seq_counter >> 16) & 0xff;
    block[13] = (seq_counter >> 24) & 0xff;

    block[15] = size & 0xff;

    cmac_context_t ctx;
    cmac_init(&ctx, key, 16);
    cmac_update(&ctx, block, 16);
    cmac_update(&ctx, buf, size);
    cmac_final(&ctx, out);
   
   *mic = (uint32_t) ((uint32_t) out[0] << 24) | ((uint32_t) out[1] << 16) | ((uint32_t) out[2] << 8) | ((uint32_t) out[3]);
}
