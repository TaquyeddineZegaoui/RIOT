/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech
 ___ _____ _   ___ _  _____ ___  ___  ___ ___
/ __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
\__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
|___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
embedded.connectivity.solutions===============

Description: LoRa MAC layer implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis ( Semtech ), Gregory Cristian ( Semtech ) and Daniel Jäckle ( STACKFORCE )
*/
#include <stdlib.h>
#include <stdint.h>

#include "crypto/ciphers.h"
#include "hashes/cmac.h"

#include "LoRaMacCrypto.h"
#include <string.h>
#include "byteorder.h"

/*!
 * CMAC/AES Message Integrity Code (MIC) Block B0 size
 */
#define LORAMAC_MIC_BLOCK_B0_SIZE                   16

/*!
 * Key size
 */
#define KEYSIZE                                     16


typedef struct  __attribute__((packed)) {
    uint8_t fb;
    uint32_t u8_pad;
    uint8_t dir; 
    le_uint32_t dev_addr;
    le_uint32_t fcnt;
    uint8_t u32_pad;
    uint8_t len;
} lorawan_mic_t;

/*!
 * Contains the computed MIC field.
 *
 * \remark Only the 4 first bytes are used
 */
static uint8_t digest[16];

/*!
 * Encryption aBlock and sBlock
 */
static uint8_t aBlock[] = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                          };
static uint8_t sBlock[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                          };

/*!
 * CMAC computation context variable
 */

static cmac_context_t CmacContext;

/*!
 * Aes computation context variable
 */

static cipher_t    AesContext;;

/*!
 * \brief Computes the LoRaMAC frame MIC field  
 *
 * \param [IN]  buffer          Data buffer
 * \param [IN]  size            Data buffer size
 * \param [IN]  key             AES key to be used
 * \param [IN]  address         Frame address
 * \param [IN]  dir             Frame direction [0: uplink, 1: downlink]
 * \param [IN]  sequenceCounter Frame sequence counter
 * \param [OUT] mic Computed MIC field
 */
uint32_t LoRaMacComputeMic( const uint8_t *buffer, uint16_t size, const uint8_t *key, uint32_t address, uint8_t dir, uint32_t sequenceCounter)
{
    lorawan_mic_t mic;
    mic.fb = 0x49;
    mic.u8_pad = 0;
    mic.dir = dir;
    mic.dev_addr = byteorder_btoll(byteorder_htonl(address));
    mic.fcnt = byteorder_btoll(byteorder_htonl(sequenceCounter));
    mic.u32_pad = 0;
    mic.len = size & 0xFF;

    cmac_init(&CmacContext, key, KEYSIZE);
    cmac_update(&CmacContext, &mic, LORAMAC_MIC_BLOCK_B0_SIZE );
    cmac_update(&CmacContext, buffer, size);
    cmac_final(&CmacContext, digest);

    return ( uint32_t )( ( uint32_t )digest[3] << 24 | ( uint32_t )digest[2] << 16 | ( uint32_t )digest[1] << 8 | ( uint32_t )digest[0] );
}

void LoRaMacPayloadEncrypt( const uint8_t *buffer, uint16_t size, const uint8_t *key, uint32_t address, uint8_t dir, uint32_t sequenceCounter, uint8_t *encBuffer )
{
    uint16_t i;
    uint8_t bufferIndex = 0;
    uint16_t ctr = 1;

    cipher_init(&AesContext, CIPHER_AES_128, key, KEYSIZE);

    aBlock[5] = dir;

    aBlock[6] = ( address ) & 0xFF;
    aBlock[7] = ( address >> 8 ) & 0xFF;
    aBlock[8] = ( address >> 16 ) & 0xFF;
    aBlock[9] = ( address >> 24 ) & 0xFF;

    aBlock[10] = ( sequenceCounter ) & 0xFF;
    aBlock[11] = ( sequenceCounter >> 8 ) & 0xFF;
    aBlock[12] = ( sequenceCounter >> 16 ) & 0xFF;
    aBlock[13] = ( sequenceCounter >> 24 ) & 0xFF;

    while( size >= 16 )
    {
        aBlock[15] = ( ( ctr ) & 0xFF );
        ctr++;
        cipher_encrypt(&AesContext, aBlock, sBlock);
        for( i = 0; i < 16; i++ )
        {
            encBuffer[bufferIndex + i] = buffer[bufferIndex + i] ^ sBlock[i];
        }
        size -= 16;
        bufferIndex += 16;
    }

    if( size > 0 )
    {
        aBlock[15] = ( ( ctr ) & 0xFF );
        cipher_encrypt(&AesContext, aBlock, sBlock);
        for( i = 0; i < size; i++ )
        {
            encBuffer[bufferIndex + i] = buffer[bufferIndex + i] ^ sBlock[i];
        }
    }
}

void LoRaMacPayloadDecrypt( const uint8_t *buffer, uint16_t size, const uint8_t *key, uint32_t address, uint8_t dir, uint32_t sequenceCounter, uint8_t *decBuffer )
{
    LoRaMacPayloadEncrypt( buffer, size, key, address, dir, sequenceCounter, decBuffer );
}

void LoRaMacJoinComputeMic( const uint8_t *buffer, uint16_t size, const uint8_t *key, uint32_t *mic )
{
    cmac_init(&CmacContext, key, KEYSIZE);
    cmac_update(&CmacContext, buffer, size);
    cmac_final(&CmacContext, digest);

    *mic = ( uint32_t )( ( uint32_t )digest[3] << 24 | ( uint32_t )digest[2] << 16 | ( uint32_t )digest[1] << 8 | ( uint32_t )digest[0] );
}

void LoRaMacJoinDecrypt( const uint8_t *buffer, uint16_t size, const uint8_t *key, uint8_t *decBuffer )
{
    cipher_init(&AesContext, CIPHER_AES_128, key, KEYSIZE);
    cipher_encrypt(&AesContext, buffer, decBuffer);

    // Check if optional CFList is included
    if( size >= 16 )
    {
        cipher_encrypt(&AesContext, buffer + 16, decBuffer + 16);
    }
}

void LoRaMacJoinComputeSKeys( const uint8_t *key, const uint8_t *appNonce, uint16_t devNonce, uint8_t *nwkSKey, uint8_t *appSKey )
{
    uint8_t nonce[16];
    uint8_t *pDevNonce = ( uint8_t * )&devNonce;
    
    cipher_init(&AesContext, CIPHER_AES_128, key, KEYSIZE);

    memset( nonce, 0, sizeof( nonce ) );
    nonce[0] = 0x01;
    memcpy( nonce + 1, appNonce, 6 );
    memcpy( nonce + 7, pDevNonce, 2 );
    cipher_encrypt(&AesContext, nonce, nwkSKey);

    memset( nonce, 0, sizeof( nonce ) );
    nonce[0] = 0x02;
    memcpy( nonce + 1, appNonce, 6 );
    memcpy( nonce + 7, pDevNonce, 2 );
    cipher_encrypt(&AesContext, nonce, appSKey);

}
