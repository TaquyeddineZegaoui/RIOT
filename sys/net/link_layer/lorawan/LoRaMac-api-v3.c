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

#include "LoRaMac-api-v3.h"
#include "LoRaMacTest.h"
#include "thread.h"
#include <string.h>

/*!
 *  Extern function declarations.
 */
extern LoRaMacStatus_t Send( LoRaMacHeader_t *macHdr, uint8_t fPort,
                             void *fBuffer, uint16_t fBufferSize );
extern LoRaMacStatus_t PrepareFrame( LoRaMacHeader_t *macHdr, LoRaMacFrameCtrl_t *fCtrl,
                                     uint8_t fPort, void *fBuffer, uint16_t fBufferSize );
extern LoRaMacStatus_t SendFrameOnChannel( ChannelParams_t channel );
extern netdev2_lorawan_t *get_dev_ptr(void);

/*!
 * Static variables
 */
static LoRaMacCallbacks_t LoRaMacCallbacks;


/*!
 * \brief   MLME-Confirm event function
 *
 * \param   [IN] mlmeConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */

void LoRaMacInit( LoRaMacCallbacks_t *callbacks, kernel_pid_t mac_pid)
{

    LoRaMacCallbacks.MacEvent = callbacks->MacEvent;

    LoRaMacInitialization(  mac_pid);
}

