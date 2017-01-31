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
static void McpsIndication( McpsIndication_t *mcpsIndication )
{
    netdev2_lorawan_t *netdev = get_dev_ptr();
    LoRaMacEventInfo.Status = netdev->frame_status;
    LoRaMacEventFlags.Bits.Rx = 1;
    LoRaMacEventFlags.Bits.RxSlot = netdev->RxSlot;
    LoRaMacEventFlags.Bits.Multicast = netdev->Multicast;
    if( mcpsIndication->RxData == true )
    {
        LoRaMacEventFlags.Bits.RxData = 1;
    }

    LoRaMacEventInfo.RxPort = netdev->Port;
    LoRaMacEventInfo.RxBuffer = netdev->Buffer;
    LoRaMacEventInfo.RxBufferSize = mcpsIndication->BufferSize;
    LoRaMacEventInfo.RxRssi = mcpsIndication->Rssi;
    LoRaMacEventInfo.RxSnr = mcpsIndication->Snr;

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
    MibRequestConfirm_t mibGet;
    McpsReq_t mcpsRequest;
    uint8_t retStatus;

    memset( ( uint8_t* )&LoRaMacEventInfo, 0, sizeof( LoRaMacEventInfo ) );

    mibGet.Type = MIB_CHANNELS_DATARATE;
    LoRaMacMibGetRequestConfirm( &mibGet );

    mcpsRequest.Type = MCPS_UNCONFIRMED;
    mcpsRequest.Req.Unconfirmed.fBuffer = fBuffer;
    mcpsRequest.Req.Unconfirmed.fBufferSize = fBufferSize;
    mcpsRequest.Req.Unconfirmed.fPort = fPort;
    mcpsRequest.Req.Unconfirmed.Datarate = mibGet.Param.ChannelsDatarate;

    switch( LoRaMacMcpsRequest( &mcpsRequest ) )
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
    MibRequestConfirm_t mibGet;
    McpsReq_t mcpsRequest;
    uint8_t retStatus;

    memset( ( uint8_t* )&LoRaMacEventInfo, 0, sizeof( LoRaMacEventInfo ) );

    mibGet.Type = MIB_CHANNELS_DATARATE;
    LoRaMacMibGetRequestConfirm( &mibGet );

    mcpsRequest.Type = MCPS_CONFIRMED;
    mcpsRequest.Req.Confirmed.fBuffer = fBuffer;
    mcpsRequest.Req.Confirmed.fBufferSize = fBufferSize;
    mcpsRequest.Req.Confirmed.fPort = fPort;
    mcpsRequest.Req.Confirmed.NbTrials = nbRetries;
    mcpsRequest.Req.Confirmed.Datarate = mibGet.Param.ChannelsDatarate;

    switch( LoRaMacMcpsRequest( &mcpsRequest ) )
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

void LoRaMacSetRx2Channel( Rx2ChannelParams_t param )
{
    MibRequestConfirm_t mibSet;

    mibSet.Type = MIB_RX2_CHANNEL;
    mibSet.Param.Rx2Channel = param;

    LoRaMacMibSetRequestConfirm( &mibSet );
}

void LoRaMacSetChannelsMask( uint16_t *mask )
{
    MibRequestConfirm_t mibSet;

    mibSet.Type = MIB_CHANNELS_MASK;
    mibSet.Param.ChannelsMask = mask;

    LoRaMacMibSetRequestConfirm( &mibSet );
}

void LoRaMacSetChannelsNbRep( uint8_t nbRep )
{
    MibRequestConfirm_t mibSet;

    mibSet.Type = MIB_CHANNELS_NB_REP;
    mibSet.Param.ChannelNbRep = nbRep;

    LoRaMacMibSetRequestConfirm( &mibSet );
}

void LoRaMacSetMaxRxWindow( uint32_t delay )
{
    MibRequestConfirm_t mibSet;

    mibSet.Type = MIB_MAX_RX_WINDOW_DURATION;
    mibSet.Param.MaxRxWindow = delay;

    LoRaMacMibSetRequestConfirm( &mibSet );
}

void LoRaMacSetReceiveDelay1( uint32_t delay )
{
    MibRequestConfirm_t mibSet;

    mibSet.Type = MIB_RECEIVE_DELAY_1;
    mibSet.Param.ReceiveDelay1 = delay;

    LoRaMacMibSetRequestConfirm( &mibSet );
}

void LoRaMacSetReceiveDelay2( uint32_t delay )
{
    MibRequestConfirm_t mibSet;

    mibSet.Type = MIB_RECEIVE_DELAY_2;
    mibSet.Param.ReceiveDelay2 = delay;

    LoRaMacMibSetRequestConfirm( &mibSet );
}

void LoRaMacSetJoinAcceptDelay1( uint32_t delay )
{
    MibRequestConfirm_t mibSet;

    mibSet.Type = MIB_JOIN_ACCEPT_DELAY_1;
    mibSet.Param.JoinAcceptDelay1 = delay;

    LoRaMacMibSetRequestConfirm( &mibSet );
}

void LoRaMacSetJoinAcceptDelay2( uint32_t delay )
{
    MibRequestConfirm_t mibSet;

    mibSet.Type = MIB_JOIN_ACCEPT_DELAY_2;
    mibSet.Param.JoinAcceptDelay2 = delay;

    LoRaMacMibSetRequestConfirm( &mibSet );
}

void LoRaMacSetChannelsDatarate( int8_t datarate )
{
    MibRequestConfirm_t mibSet;

    mibSet.Type = MIB_CHANNELS_DATARATE;
    mibSet.Param.ChannelsDatarate = datarate;

    LoRaMacMibSetRequestConfirm( &mibSet );
}

void LoRaMacSetChannelsTxPower( int8_t txPower )
{
    MibRequestConfirm_t mibSet;

    mibSet.Type = MIB_CHANNELS_TX_POWER;
    mibSet.Param.ChannelsTxPower = txPower;

    LoRaMacMibSetRequestConfirm( &mibSet );
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
