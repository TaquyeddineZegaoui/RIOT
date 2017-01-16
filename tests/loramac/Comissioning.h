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
#ifdef OTA
	#define OVER_THE_AIR_ACTIVATION                     1
#else 
	#define OVER_THE_AIR_ACTIVATION                     0
#endif	

/*!
 * Indicates if the end-device is to be connected to a private or public network
 */

#ifdef PUBLIC_NTW
	#define LORAWAN_PUBLIC_NETWORK                      true
#else
	#define LORAWAN_PUBLIC_NETWORK                      false
#endif

/*!
 * Mote device IEEE EUI (big endian)
 *
 * \remark In this application the value is automatically generated by calling
 *         BoardGetUniqueId function
 */
#ifndef LORAWAN_DEVICE_EUI
#define LORAWAN_DEVICE_EUI                          {0x06, 0x00, 0xf9, 0xff, 0x2c, 0xca, 0xd3, 0x35}
#endif

/*!
 * Application IEEE EUI (big endian)
 */
#ifndef LORAWAN_APPLICATION_EUI
#define LORAWAN_APPLICATION_EUI                     { 0x76, 0x5e, 0x2e, 0x5e, 0xcf, 0x3d, 0xd3, 0xdb }
#endif

/*!
 * AES encryption/decryption cipher application key
 */
#ifndef LORAWAN_APPLICATION_KEY
#define LORAWAN_APPLICATION_KEY                     { 0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3A }
#endif

/*!
 * Current network ID
 */
#ifndef LORAWAN_NETWORK_ID
#define LORAWAN_NETWORK_ID                          ( uint32_t )0
#endif

/*!
 * Device address on the network (big endian)
 *
 * \remark In this application the value is automatically generated using
 *         a pseudo random generator seeded with a value derived from
 *         BoardUniqueId value if LORAWAN_DEVICE_ADDRESS is set to 0
 */
#ifndef LORAWAN_DEVICE_ADDRESS
#define LORAWAN_DEVICE_ADDRESS                      ( uint32_t )0x166fb58e
#endif

/*!
 * AES encryption/decryption cipher network session key
 */

#ifndef LORAWAN_NWKSKEY
#define LORAWAN_NWKSKEY                             { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3A }
#endif
/*!
 * AES encryption/decryption cipher application session key
 */
#ifndef LORAWAN_APPSKEY
#define LORAWAN_APPSKEY                             { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3A }
#endif

#endif // __LORA_COMMISSIONING_H__