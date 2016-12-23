/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech
Description: End device commissioning parameters
License: Revised BSD License, see LICENSE.TXT file include in the project
Maintainer: Miguel Luis and Gregory Cristian
*/
#ifndef __LORA_COMMISSIONING_H__
#define __LORA_COMMISSIONING_H__

#include "common.h"

/*!
 * When set to 1 the application uses the Over-the-Air activation procedure
 * When set to 0 the application uses the Personalization activation procedure
 */
#define OVER_THE_AIR_ACTIVATION                     1

/*!
 * Indicates if the end-device is to be connected to a private or public network
 */
#define LORAWAN_PUBLIC_NETWORK                      true

/*!
 * IEEE Organizationally Unique Identifier ( OUI ) (big endian)
 */
#define IEEE_OUI                                     0x06, 0x00, 0xf9

/*!
 * Mote device IEEE EUI (big endian)
 *
 * \remark In this application the value is automatically generated by calling
 *         BoardGetUniqueId function
 */
#define LORAWAN_DEVICE_EUI                          {0x5e, 0xb5, 0xdd, 0xe8, 0x79, 0x9e, 0xe9, 0x2e}

/*!
 * Application IEEE EUI (big endian)
 */
#define LORAWAN_APPLICATION_EUI                     { 0x76, 0x5e, 0x2e, 0x5e, 0xcf, 0x3d, 0xd3, 0xdb }

/*!
 * AES encryption/decryption cipher application key
 */
#define LORAWAN_APPLICATION_KEY                     { 0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3A }

/*!
 * Current network ID
 */
#define LORAWAN_NETWORK_ID                          ( uint32_t )0

/*!
 * Device address on the network (big endian)
 *
 * \remark In this application the value is automatically generated using
 *         a pseudo random generator seeded with a value derived from
 *         BoardUniqueId value if LORAWAN_DEVICE_ADDRESS is set to 0
 */
#ifdef SAMR21_XPRO 
#define LORAWAN_DEVICE_ADDRESS                      ( uint32_t )0x166fb58e
#else
#define LORAWAN_DEVICE_ADDRESS                      ( uint32_t )0x1677f14e
#endif

/*!
 * AES encryption/decryption cipher network session key
 */
#define LORAWAN_NWKSKEY                             { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3A }

/*!
 * AES encryption/decryption cipher application session key
 */
#define LORAWAN_APPSKEY                             { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3A }

#endif // __LORA_COMMISSIONING_H__
