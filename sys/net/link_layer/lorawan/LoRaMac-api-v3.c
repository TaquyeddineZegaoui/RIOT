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
static LoRaMacEventFlags_t LoRaMacEventFlags;
static LoRaMacEventInfo_t LoRaMacEventInfo;
static LoRaMacPrimitives_t LoRaMacPrimitives;
static LoRaMacCallbacks_t LoRaMacCallbacks;

/*!
 * \brief   MCPS-Confirm event function
 *
 * \param   [IN] mcpsConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void McpsConfirm(void)
{
    netdev2_lorawan_t *netdev = get_dev_ptr();
    LoRaMacEventInfo.Status = netdev->frame_status;
    LoRaMacEventFlags.Bits.Tx = 1;

    LoRaMacEventInfo.TxDatarate = netdev->datarate;
    LoRaMacEventInfo.TxNbRetries = netdev->n_retries;
    LoRaMacEventInfo.TxAckReceived = netdev->ack_received;

     if( ( netdev->LoRaMacFlags.Bits.McpsInd != 1 ) && ( netdev->LoRaMacFlags.Bits.MlmeReq != 1 ) )
    {
        LoRaMacCallbacks.MacEvent( &LoRaMacEventFlags, &LoRaMacEventInfo );
        LoRaMacEventFlags.Value = 0;
    }
}

/*!
 * \brief   MCPS-Indication event function
 *
 * \param   [IN] mcpsIndication - Pointer to the indication structure,
 *               containing indication attributes.
 */
static void McpsIndication(void)
{
    netdev2_lorawan_t *netdev = get_dev_ptr();
    LoRaMacEventInfo.Status = netdev->frame_status;
    LoRaMacEventFlags.Bits.Rx = 1;
    LoRaMacEventFlags.Bits.RxSlot = netdev->RxSlot;
    LoRaMacEventFlags.Bits.Multicast = netdev->Multicast;
    if( netdev->RxData == true )
    {
        LoRaMacEventFlags.Bits.RxData = 1;
    }

    LoRaMacEventInfo.RxPort = netdev->Port;
    LoRaMacEventInfo.RxBuffer = netdev->Buffer;
    LoRaMacEventInfo.RxBufferSize = netdev->BufferSize;
    LoRaMacEventInfo.RxRssi = netdev->Rssi;
    LoRaMacEventInfo.RxSnr = netdev->Snr;

    LoRaMacCallbacks.MacEvent( &LoRaMacEventFlags, &LoRaMacEventInfo );
    LoRaMacEventFlags.Value = 0;
}

/*!
 * \brief   MLME-Confirm event function
 *
 * \param   [IN] mlmeConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */

void MlmeConfirm(uint8_t mlme_req)
{
    netdev2_lorawan_t *netdev = get_dev_ptr();
    if( netdev->frame_status == LORAMAC_EVENT_INFO_STATUS_OK )
    {
        switch( mlme_req )
        {
            case MLME_JOIN:
            {
                // Status is OK, node has joined the network
                LoRaMacEventFlags.Bits.Tx = 1;
                LoRaMacEventFlags.Bits.Rx = 1;
                LoRaMacEventFlags.Bits.JoinAccept = 1;
                break;
            }
            case MLME_LINK_CHECK:
            {
                LoRaMacEventFlags.Bits.Tx = 1;
                LoRaMacEventFlags.Bits.Rx = 1;
                LoRaMacEventFlags.Bits.LinkCheck = 1;

                LoRaMacEventInfo.DemodMargin = netdev->demod_margin;
                LoRaMacEventInfo.NbGateways = netdev->number_of_gateways;
                break;
            }
            default:
                break;
        }
    }
    LoRaMacEventInfo.Status = netdev->frame_status;

    if( netdev->LoRaMacFlags.Bits.McpsInd != 1 )
    {
        LoRaMacCallbacks.MacEvent( &LoRaMacEventFlags, &LoRaMacEventInfo );
        LoRaMacEventFlags.Value = 0;
    }
}

void LoRaMacInit( LoRaMacCallbacks_t *callbacks, kernel_pid_t mac_pid)
{
    LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
    LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
    LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;

    LoRaMacCallbacks.MacEvent = callbacks->MacEvent;

    LoRaMacInitialization( &LoRaMacPrimitives, mac_pid);
}

void LoRaMacMulticastChannelAdd( MulticastParams_t *channelParam )
{
    LoRaMacMulticastChannelLink( channelParam );
}

void LoRaMacMulticastChannelRemove( MulticastParams_t *channelParam )
{
    LoRaMacMulticastChannelUnlink( channelParam );
}


uint8_t LoRaMacSendFrame( uint8_t fPort, void *fBuffer, uint16_t fBufferSize )
{
    netdev2_lorawan_t *netdev = get_dev_ptr();
    MibRequestConfirm_t mibGet;
    uint8_t retStatus;

    memset( ( uint8_t* )&LoRaMacEventInfo, 0, sizeof( LoRaMacEventInfo ) );

    mibGet.Type = MIB_CHANNELS_DATARATE;
    LoRaMacMibGetRequestConfirm( &mibGet );

    netdev->fBuffer = fBuffer;
    netdev->fBufferSize = fBufferSize;
    netdev->fPort = fPort;
    netdev->Datarate = mibGet.Param.ChannelsDatarate;

    switch( LoRaMacMcpsRequest(MCPS_UNCONFIRMED) )
    {
        case LORAMAC_STATUS_OK:
            retStatus = 0U;
            break;
        case LORAMAC_STATUS_BUSY:
            retStatus = 1U;
            break;
        case LORAMAC_STATUS_NO_NETWORK_JOINED:
            retStatus = 2U;
            break;
        case LORAMAC_STATUS_LENGTH_ERROR:
        case LORAMAC_STATUS_MAC_CMD_LENGTH_ERROR:
            retStatus = 3U;
            break;
        case LORAMAC_STATUS_SERVICE_UNKNOWN:
            retStatus = 4U;
            break;
        case LORAMAC_STATUS_DEVICE_OFF:
            retStatus = 6U;
            break;
        default:
            retStatus = 1U;
            break;
    }

    return retStatus;
}

uint8_t LoRaMacSendConfirmedFrame( uint8_t fPort, void *fBuffer, uint16_t fBufferSize, uint8_t nbRetries )
{
    netdev2_lorawan_t *netdev = get_dev_ptr();
    MibRequestConfirm_t mibGet;
    uint8_t retStatus;

    memset( ( uint8_t* )&LoRaMacEventInfo, 0, sizeof( LoRaMacEventInfo ) );

    mibGet.Type = MIB_CHANNELS_DATARATE;
    LoRaMacMibGetRequestConfirm( &mibGet );

    netdev->fBuffer = fBuffer;
    netdev->fBufferSize = fBufferSize;
    netdev->fPort = fPort;
    netdev->NbTrials = nbRetries;
    netdev->Datarate = mibGet.Param.ChannelsDatarate;

    switch( LoRaMacMcpsRequest(MCPS_CONFIRMED) )
    {
        case LORAMAC_STATUS_OK:
            retStatus = 0U;
            break;
        case LORAMAC_STATUS_BUSY:
            retStatus = 1U;
            break;
        case LORAMAC_STATUS_NO_NETWORK_JOINED:
            retStatus = 2U;
            break;
        case LORAMAC_STATUS_LENGTH_ERROR:
        case LORAMAC_STATUS_MAC_CMD_LENGTH_ERROR:
            retStatus = 3U;
            break;
        case LORAMAC_STATUS_SERVICE_UNKNOWN:
            retStatus = 4U;
            break;
        case LORAMAC_STATUS_DEVICE_OFF:
            retStatus = 6U;
            break;
        default:
            retStatus = 1U;
            break;
    }

    return retStatus;
}


void LoRaMacSetChannel( uint8_t id, ChannelParams_t params )
{
    LoRaMacChannelAdd( id, params );
}


uint32_t LoRaMacGetUpLinkCounter( void )
{
    MibRequestConfirm_t mibGet;

    mibGet.Type = MIB_UPLINK_COUNTER;

    LoRaMacMibGetRequestConfirm( &mibGet );

    return mibGet.Param.UpLinkCounter;
}

uint32_t LoRaMacGetDownLinkCounter( void )
{
    MibRequestConfirm_t mibGet;

    mibGet.Type = MIB_DOWNLINK_COUNTER;

    LoRaMacMibGetRequestConfirm( &mibGet );

    return mibGet.Param.DownLinkCounter;
}

void LoRaMacSetMicTest( uint16_t txPacketCounter )
{
    LoRaMacTestSetMic( txPacketCounter );
}
