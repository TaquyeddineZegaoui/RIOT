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
#include <stdbool.h>

#include "net/netdev2.h"
#include "net/netopt.h"
#include "net/lorawan/timer.h"
#include "LoRaMac-definitions.h"

#ifdef __cplusplus
extern "C" {
#endif

#define KEY_SIZE		16	

#define LORAMAC_PHY_MAXPAYLOAD                      255
#define LORA_MAC_COMMAND_MAX_LENGTH                 15


typedef enum eMlme
{
    MLME_JOIN,
    MLME_LINK_CHECK,
}Mlme_t;

/*!
 * Enumeration containing the status of the operation of a MAC service
 */
typedef enum eLoRaMacEventInfoStatus
{
    LORAMAC_EVENT_INFO_STATUS_OK = 0,
    LORAMAC_EVENT_INFO_STATUS_ERROR,
    LORAMAC_EVENT_INFO_STATUS_TX_TIMEOUT,
    LORAMAC_EVENT_INFO_STATUS_RX2_TIMEOUT,
    LORAMAC_EVENT_INFO_STATUS_RX2_ERROR,
    LORAMAC_EVENT_INFO_STATUS_JOIN_FAIL,
    LORAMAC_EVENT_INFO_STATUS_DOWNLINK_REPEATED,
    LORAMAC_EVENT_INFO_STATUS_DOWNLINK_TOO_MANY_FRAMES_LOSS,
    LORAMAC_EVENT_INFO_STATUS_ADDRESS_FAIL,
    LORAMAC_EVENT_INFO_STATUS_MIC_FAIL,
}LoRaMacEventInfoStatus_t;

/*!
 * LoRaMAC channels parameters definition
 */
typedef union uDrRange
{
    int8_t Value;
    struct sFields
    {
        int8_t Min : 4;
        int8_t Max : 4;
    }Fields;
}DrRange_t;

typedef struct sChannelParams
{
    uint32_t Frequency;
    DrRange_t DrRange;
    uint8_t Band;
}ChannelParams_t;


typedef struct lorawan_tx_rx_config
{
	bool adr_ctrl;                            /* LoRaMac ADR control status*/
	bool nwk_status;                       /* Boolean if network is joined*/
}lorawan_tx_rx_config_t;

typedef struct lorawan_sesion
{
	uint8_t nwk_skey[KEY_SIZE];        		   /* AES encryption/decryption cipher network session key */
	uint8_t app_skey[KEY_SIZE];                /* AES encryption/decryption cipher application session key */
	uint32_t dev_addr;                 		   /* Mote Address */
	uint32_t net_id;              			   /* Network ID ( 3 bytes ) */	
	bool public;                               /* Indicates if the node is connected to a private or public network */
	bool RepeaterSupport;                      /* Indicates if the node supports repeaters*/
	//LoRaMacPrimitives_t *LoRaMacPrimitives;  /* LoRaMac upper layer event functions*/
	//LoRaMacCallback_t *LoRaMacCallbacks;     /* LoRaMac upper layer callback functions*/
	lorawan_tx_rx_config_t tx_rx;              /* Transreception session info*/
}lorawan_sesion_t;

typedef struct sMulticastParams
{
    uint32_t Address;
    uint8_t NwkSKey[16];
    uint8_t AppSKey[16];
    uint32_t DownLinkCounter;
    struct sMulticastParams *Next;
}MulticastParams_t;

typedef enum eDeviceClass
{
    CLASS_A,
    CLASS_B,
    CLASS_C,
}DeviceClass_t;

typedef struct sRx2ChannelParams
{
    uint32_t Frequency;
    uint8_t  Datarate;
}Rx2ChannelParams_t;

typedef struct sLoRaMacParams
{
    int8_t ChannelsTxPower;
    int8_t ChannelsDatarate;
    uint32_t MaxRxWindow;
    uint32_t ReceiveDelay1;
    uint32_t ReceiveDelay2;
    uint32_t JoinAcceptDelay1;
    uint32_t JoinAcceptDelay2;
    uint8_t ChannelsNbRep;
    uint8_t Rx1DrOffset;
    Rx2ChannelParams_t Rx2Channel;
    uint16_t ChannelsMask[6];
}LoRaMacParams_t;


typedef enum eMcps
{
    MCPS_UNCONFIRMED,
    MCPS_CONFIRMED,
    MCPS_MULTICAST,
    MCPS_PROPRIETARY,
}Mcps_t;

typedef struct sLoRaMacPrimitives
{
    void ( *MacMcpsConfirm )(void);
}LoRaMacPrimitives_t;

typedef union eLoRaMacFlags_t
{
    uint8_t Value;
    struct sMacFlagBits
    {
        uint8_t McpsReq         : 1;
        uint8_t McpsInd         : 1;
        uint8_t MlmeReq         : 1;
        uint8_t MacDone         : 1;
    }Bits;
}LoRaMacFlags_t;

typedef struct {
    netdev2_t netdev;                       /**< @ref netdev2_t base class */
    lorawan_sesion_t lorawan;
    uint8_t *dev_eui;
    uint8_t *app_key;
    uint8_t *app_eui;
    uint16_t dev_nonce;
    MulticastParams_t *MulticastChannels;
    DeviceClass_t LoRaMacDeviceClass;
    bool RepeaterSupport;
    uint8_t LoRaMacBuffer[LORAMAC_PHY_MAXPAYLOAD];
    uint16_t LoRaMacBufferPktLen;
    uint8_t LoRaMacPayload[LORAMAC_PHY_MAXPAYLOAD];
    uint8_t LoRaMacRxPayload[LORAMAC_PHY_MAXPAYLOAD];
    uint32_t uplink_counter;
    uint32_t DownLinkCounter;
    bool IsUpLinkCounterFixed;
    bool IsRxWindowsEnabled;
    uint32_t AdrAckCounter;
    bool NodeAckRequested;
    bool SrvAckRequested;
    bool MacCommandsInNextTx;
    uint8_t MacCommandsBufferIndex;
    uint8_t MacCommandsBufferToRepeatIndex;
    uint8_t MacCommandsBuffer[LORA_MAC_COMMAND_MAX_LENGTH];
    uint8_t MacCommandsBufferToRepeat[LORA_MAC_COMMAND_MAX_LENGTH];
    uint16_t ChannelsMaskRemaining[6];
    LoRaMacParams_t LoRaMacParams;
    LoRaMacParams_t LoRaMacParamsDefaults;
    uint8_t ChannelsNbRepCounter;
    uint8_t MaxDCycle;
    uint16_t AggregatedDCycle;
    TimerTime_t AggregatedLastTxDoneTime;
    TimerTime_t AggregatedTimeOff;
    bool DutyCycleOn;
    uint8_t Channel;
    uint8_t LastTxChannel;
    uint32_t LoRaMacState;
    TimerEvent_t MacStateCheckTimer;
    LoRaMacPrimitives_t *LoRaMacPrimitives;
    TimerEvent_t TxDelayedTimer;
    TimerEvent_t RxWindowTimer1;
    TimerEvent_t RxWindowTimer2;
    uint32_t RxWindow1Delay;
    uint32_t RxWindow2Delay;
    TimerEvent_t AckTimeoutTimer;
    uint8_t AckTimeoutRetries;
    uint8_t AckTimeoutRetriesCounter;
    bool AckTimeoutRetry;
    TimerTime_t TxTimeOnAir;
    uint16_t JoinRequestTrials;
    LoRaMacFlags_t LoRaMacFlags;
    uint8_t RxSlot;
    uint8_t frame_status;
    uint8_t demod_margin;
    uint8_t number_of_gateways;
    uint8_t last_command;
    uint8_t last_frame;
    int8_t tx_power;
    bool ack_received;
    uint8_t datarate;
    uint8_t n_retries;
    uint32_t received_downlink;
    uint8_t Multicast;
    uint8_t Port;
    uint8_t RxDatarate;
    uint8_t FramePending;
    uint8_t *Buffer;
    uint8_t BufferSize;
    bool RxData;
    int16_t Rssi;
    uint8_t Snr;
    //Frame variables
    uint8_t fPort;
    void *fBuffer;
    uint16_t fBufferSize;
    int8_t Datarate;
    uint8_t NbTrials;

    bool b_tx;
    bool b_rx;
    bool link_check;
    bool join_req;
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
int netdev2_lorawan_get(netdev2_lorawan_t *dev, netopt_t opt, void *value, size_t max_len);

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
int netdev2_lorawan_set(netdev2_lorawan_t *dev, netopt_t opt, void *value, size_t value_len);

#ifdef __cplusplus
}
#endif
/** @} */
#endif /* NETDEV2_ETH_H */
