/*
 * Copyright (C) 2016 Fundacion Inria Chile
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/* @file        main.c
 * @brief       Drop Watcher main app, all important parameters are set by MAKEFILE
 *              
 *              This app works with Inria-Chile/RIOT/lorawan_drop_watcher branch
 *
 * @author      Francisco Molina <francisco.molina@inria.cl>
*/
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "shell.h"
#include "shell_commands.h"
#include "thread.h"
#include "xtimer.h"
#include "lpm.h"
#include "random.h"

#include "driver_config.h"
#include "board.h"
#include "boards_hw.h"
#include "infrared.h"
#include "app_config.h"

#include "rtctimers.h"
#include "periph/rtc.h"

#include "sx1276.h"
#include "sx1276_registers.h"
#include "sx1276_params.h"
#include "sx1276_netdev.h"
#include "sx1276_registers.h"

#include "LoRaMac.h"
#include "net/lorawan/board_definitions.h" 
#include "lorawan_config.h"
#include "LoRaMac-api-v3.h"
#include "net/gnrc/netdev2.h"
#include "net/netdev2.h"

#define ENABLE_DEBUG  (0)
#include "debug.h"

#define GNRC_LORA_MSG_QUEUE         16

#define LORAWAN_TX_TIMEOUT_EVENT    10
#define LORAWAN_RX_TIMEOUT_EVENT    11
#define LORAWAN_CRC_ERROR           12
#define LORAWAN_FHSS_CHANGE_CHANNEL 13
#define LORAWAN_CAD_DONE            14

/*!
 * Thread Variables and packet count
 */
static sx1276_t sx1276;                     // Lora driver object
static netdev2_t *nd;                       // Netedev object
volatile uint8_t HasLoopedThroughMain = 0;  // Changed threw main variables
volatile uint8_t rx_done_flag = 0;          // Rx done flag
volatile uint8_t rx_timeout_flag = 0;       // Rx timeout flag
uint32_t count = 0;                         // Transmited packet count

/*!
*   Node id for APP, hash generated from DEVEUI
*/
static uint8_t NodeId[] = {0x00, 0x00, 0x00, 0x00};

/*!
 * User application data buffer size
 */
#if defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )

#define LORAWAN_APP_DATA_SIZE                       11

#endif

static uint8_t DevEui[] = LORAWAN_DEVICE_EUI;

#if( OVER_THE_AIR_ACTIVATION != 0 )

static uint8_t AppEui[] = LORAWAN_APPLICATION_EUI;
static uint8_t AppKey[] = LORAWAN_APPLICATION_KEY;

#else
static uint8_t NwkSKey[] = LORAWAN_NWKSKEY;
static uint8_t AppSKey[] = LORAWAN_APPSKEY;

/*!
 * Device address
 */
static uint32_t DevAddr = LORAWAN_DEVICE_ADDRESS;

#endif

/*!
 * Indicates if the MAC layer has already joined a network.
 */
static bool IsNetworkJoined = false;

/*!
 * Application port
 */
static uint8_t AppPort = LORAWAN_APP_PORT;

/*!
 * User application data size
 */
static uint8_t AppDataSize = LORAWAN_APP_DATA_SIZE;

/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_MAX_SIZE                           64

/*!
 * User application data
 */
static uint8_t AppData[LORAWAN_APP_DATA_MAX_SIZE];

/*!
 * Indicates if the node is sending confirmed or unconfirmed messages
 */
static uint8_t IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;

/*!
 * Indicates if a new packet can be sent
 */
static bool TxNextPacket = false;
static bool DownlinkStatusUpdate = false;

static LoRaMacCallbacks_t LoRaMacCallbacks;

/*!
 * LoRaWAN compliance tests support data
 */
struct ComplianceTest_s
{
    bool Running;
    uint8_t State;
    bool IsTxConfirmed;
    uint8_t AppPort;
    uint8_t AppDataSize;
    uint8_t *AppDataBuffer;
    uint16_t DownLinkCounter;
    bool LinkCheck;
    uint8_t DemodMargin;
    uint8_t NbGateways;
}ComplianceTest;


/*!
 * \brief   Prepares the payload of the frame
 */
static void PrepareTxFrame( uint8_t port )
{
    switch( port )
    {
    case 1:
        {
            /* Status Byte, only 2 states defined to this point*/
            if(count_drops(DROP_TIME))
                AppData[8] = COUNT_FAIL;
            else
                AppData[8] = APP_OK;

            uint16_t time = get_time();
            AppData[0] = NodeId[0];
            AppData[1] = NodeId[1];
            AppData[2] = NodeId[2];
            AppData[3] = NodeId[3];
            AppData[4] = get_drops();                       // Stores counted drops
            AppData[5] = (uint8_t) (time & 0xff);           // 2 byte for accurate time measured
            AppData[6] = (uint8_t) ((time >> 8) & 0xff);    
            AppData[7] = board_get_battery_level();         // Battery measurement
        }

    case 2:
        {
            /* Status Byte, only 2 states defined to this point*/
            if(count_drops(DROP_TIME))
                AppData[8] = COUNT_FAIL;
            else
                AppData[8] = APP_OK;

            uint16_t time = get_time();
            AppData[0] = NodeId[0];
            AppData[1] = NodeId[1];
            AppData[2] = NodeId[2];
            AppData[3] = NodeId[3];
            AppData[4] = get_drops();                       // Stores counted drops
            AppData[5] = (uint8_t) (time & 0xff);           // 2 byte for accurate time measured
            AppData[6] = (uint8_t) ((time >> 8) & 0xff);    
            AppData[7] = board_get_battery_level();         // Battery measurement

            printf("DROPS: %d \n", get_drops());
            printf("BAT: %d \n", (100*board_get_battery_level())/255);
        }
        break;
    case 224:
        if( ComplianceTest.LinkCheck == true )
        {
            ComplianceTest.LinkCheck = false;
            AppDataSize = 3;
            AppData[0] = 5;
            AppData[1] = ComplianceTest.DemodMargin;
            AppData[2] = ComplianceTest.NbGateways;
            ComplianceTest.State = 1;
        }
        else
        {
            switch( ComplianceTest.State )
            {
            case 4:
                ComplianceTest.State = 1;
                break;
            case 1:
                AppDataSize = 2;
                AppData[0] = ComplianceTest.DownLinkCounter >> 8;
                AppData[1] = ComplianceTest.DownLinkCounter;
                break;
            }
        }
        break;
    default:
        break;
    }
}

static void ProcessRxFrame( LoRaMacEventFlags_t *flags, LoRaMacEventInfo_t *info )
{
    switch( info->RxPort ) // Check Rx port number
    {
    case 1: // The application LED can be controlled on port 1 or 2
    case 2:
        if( info->RxBufferSize == 1 )
        {
        }
        break;
    case 224:
        if( ComplianceTest.Running == false )
        {
            // Check compliance test enable command (i)
            if( ( info->RxBufferSize == 4 ) && 
                ( info->RxBuffer[0] == 0x01 ) &&
                ( info->RxBuffer[1] == 0x01 ) &&
                ( info->RxBuffer[2] == 0x01 ) &&
                ( info->RxBuffer[3] == 0x01 ) )
            {
                IsTxConfirmed = false;
                AppPort = 224;
                AppDataSize = 2;
                ComplianceTest.DownLinkCounter = 0;
                ComplianceTest.LinkCheck = false;
                ComplianceTest.DemodMargin = 0;
                ComplianceTest.NbGateways = 0;
                ComplianceTest.Running = true;
                ComplianceTest.State = 1;
                
                LoRaMacSetAdrOn( true );
            }
        }
        else
        {
            ComplianceTest.State = info->RxBuffer[0];
            switch( ComplianceTest.State )
            {
            case 0: // Check compliance test disable command (ii)
                IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;
                AppPort = LORAWAN_APP_PORT;
                AppDataSize = LORAWAN_APP_DATA_SIZE;
                ComplianceTest.DownLinkCounter = 0;
                ComplianceTest.Running = false;
                LoRaMacSetAdrOn( LORAWAN_ADR_ON );
                break;
            case 1: // (iii, iv)
                AppDataSize = 2;
                break;
            case 2: // Enable confirmed messages (v)
                IsTxConfirmed = true;
                ComplianceTest.State = 1;
                break;
            case 3:  // Disable confirmed messages (vi)
                IsTxConfirmed = false;
                ComplianceTest.State = 1;
                break;
            case 4: // (vii)
                AppDataSize = info->RxBufferSize;

                AppData[0] = 4;
                for( uint8_t i = 1; i < AppDataSize; i++ )
                {
                    AppData[i] = info->RxBuffer[i] + 1;
                }
                break;
            case 5: // (viii)
                LoRaMacLinkCheckReq( );
                break;
            default:
                break;
            }
        }
        break;
    default:
        break;
    }
}

static bool SendFrame( void )
{
    uint8_t sendFrameStatus = 0;

    /* Sends unconfirmed message*/
    if( IsTxConfirmed == false )
    {
        sendFrameStatus = LoRaMacSendFrame( AppPort, AppData, AppDataSize );
    }
    /* Sends confirmed message*/
    else
    {
        sendFrameStatus = LoRaMacSendConfirmedFrame( AppPort, AppData, AppDataSize, 8 );
    }
    /* Returns false if no need to re-schedule, prints frameStatus Debug info*/
    switch( sendFrameStatus )
    {
        case 0:
            DEBUG("OK \n");
            return false;
        case 1:
            DEBUG("BUSY \n");  
            return false;
        case 2:
            DEBUG("NO_NETWORK_JOINED \n");  
            return false;     
        case 3: // LENGTH_ERROR
            // Send empty frame in order to flush MAC commands
            LoRaMacSendFrame( 0, NULL, 0 );
            return false;
        case 4:
            DEBUG("MAC_CMD_ERROR \n");  
            return false;
        case 5: // NO_FREE_CHANNEL
            // Try again later
            DEBUG("NO_FREE_CHANNEL \n");  
            return true;
        case 6:
            DEBUG("DEVICE_OFF \n");  
            return false;
        default:
            return false;
    }
}

/*!
 * \brief Function executed on TxNextPacket Timeout event
 */
static void OnTxNextPacketTimerEvent(void *arg)
{
    (void)arg;
    struct tm time;
    rtc_get_alarm(&time);

    /* Schedule next transmision AppPacket*/
    time.tm_sec += APP_TX_DUTYCYCLE_SEC /*+ randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND )*/;
    time.tm_min += APP_TX_DUTYCYCLE_MIN /*+ randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND )*/;
    time.tm_hour += APP_TX_DUTYCYCLE_HOUR /*+ randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND )*/;
    time_check(&time);

    rtc_clear_alarm();
    rtc_set_alarm(&time, OnTxNextPacketTimerEvent, 0);
    TxNextPacket = true;
}

#if defined OTA
/*!
 * \brief Function executed on JoinReq Timeout event
 */
static void OnJoinReqTimerEvent(void *arg)
{
    (void)arg;
    struct tm time;
    rtc_get_alarm(&time);

    /* Schedule next transmision JoinReq or AppPacket*/
    if(IsNetworkJoined == false)
    {
        time.tm_sec  += OVER_THE_AIR_ACTIVATION_DUTYCYCLE_SEC;
        time.tm_min  += OVER_THE_AIR_ACTIVATION_DUTYCYCLE_MIN;
    }
    else
    {
        time.tm_sec += APP_TX_DUTYCYCLE_SEC;
        time.tm_min += APP_TX_DUTYCYCLE_MIN;
        time.tm_hour += APP_TX_DUTYCYCLE_HOUR;
    }
    time_check(&time);

    rtc_clear_alarm(); 
    if(IsNetworkJoined == true)
        rtc_set_alarm(&time, OnTxNextPacketTimerEvent, 0);
    else
        rtc_set_alarm(&time, OnJoinReqTimerEvent, 0);

    TxNextPacket = true;
}
#endif

/*!
 * \brief Function to be executed on MAC layer event
 */
static void OnMacEvent( LoRaMacEventFlags_t *flags, LoRaMacEventInfo_t *info )
{
    if( flags->Bits.JoinAccept == 1 )
    {
        IsNetworkJoined = true;
    }
    else
    {
        if( flags->Bits.Tx == 1 )
        {
        }

        if( flags->Bits.Rx == 1 )
        {
            if( ComplianceTest.Running == true )
            {
                ComplianceTest.DownLinkCounter++;
                if( flags->Bits.LinkCheck == 1 )
                {
                    ComplianceTest.LinkCheck = true;
                    ComplianceTest.DemodMargin = info->DemodMargin;
                    ComplianceTest.NbGateways = info->NbGateways;
                }
            }
            if( flags->Bits.RxData == true )
            {
                ProcessRxFrame( flags, info );
            }

            DownlinkStatusUpdate = true;
        }
    }
}

void *_event_loop(void *arg)
{
    static msg_t _msg_q[GNRC_LORA_MSG_QUEUE];
    msg_t msg, reply;
    netdev2_t *netdev = (netdev2_t*) arg;
    msg_init_queue(_msg_q, GNRC_LORA_MSG_QUEUE);

    gnrc_netapi_opt_t *opt;
    RadioEvents_t *events;
    int res;

    while (1) {
        msg_receive(&msg);
        switch (msg.type) {
            case NETDEV2_MSG_TYPE_EVENT:
                netdev->driver->isr(netdev);
                break;
            case GNRC_NETAPI_MSG_TYPE_SND:
                //gnrc_pktsnip_t *pkt = msg.content.ptr;
                //gnrc_netdev2->send(gnrc_netdev2, pkt);
                break;
            case GNRC_NETAPI_MSG_TYPE_SET:
                /* read incoming options */
                opt = msg.content.ptr;
                /* set option for device driver */
                res = netdev->driver->set(netdev, opt->opt, opt->data, opt->data_len);
                /* send reply to calling thread */
                reply.type = GNRC_NETAPI_MSG_TYPE_ACK;
                reply.content.value = (uint32_t)res;
                msg_reply(&msg, &reply);
                break;
            case GNRC_NETAPI_MSG_TYPE_GET:
                /* read incoming options */
                opt = msg.content.ptr;
                /* get option from device driver */
                res = netdev->driver->get(netdev, opt->opt, opt->data, opt->data_len);
                /* send reply to calling thread */
                reply.type = GNRC_NETAPI_MSG_TYPE_ACK;
                reply.content.value = (uint32_t)res;
                msg_reply(&msg, &reply);
                break;
            case LORAWAN_TX_TIMEOUT_EVENT:
                puts("sx1276: TX timeout");
                events = radio_get_event_ptr();
                events->TxTimeout();
                break;
            case LORAWAN_RX_TIMEOUT_EVENT:
                puts("sx1276: RX timeout");
                events = radio_get_event_ptr();
                events->RxTimeout();
                break;
            case LORAWAN_CRC_ERROR:
                puts("sx1276: RX CRC_ERROR");
                events = radio_get_event_ptr();
                events->RxError();
                break;
            case LORAWAN_FHSS_CHANGE_CHANNEL:
                events = radio_get_event_ptr();
                events->FhssChangeChannel(sx1276._internal.last_channel);
                break;
            case LORAWAN_CAD_DONE:
                events = radio_get_event_ptr();
                events->CadDone(sx1276._internal.is_last_cad_success);
                break;
            case LORAWAN_TIMER_MAC_STATE:
                OnMacStateCheckTimerEvent();
                break;
            case LORAWAN_TIMER_RX_WINDOW1:
                OnRxWindow1TimerEvent();
                break;
            case LORAWAN_TIMER_RX_WINDOW2:
                OnRxWindow2TimerEvent();
                break;
            case LORAWAN_TIMER_ACK_TIMEOUT:
                OnAckTimeoutTimerEvent();
                break;
            case LORAWAN_TIMER_TX_DELAYED:
                OnTxDelayedTimerEvent();
                break;
            default:
                break;
        }
    }
    return NULL;
}

static unsigned char message[64];
static void _event_cb(netdev2_t *dev, netdev2_event_t event)
{
    msg_t msg;
    msg.type = NETDEV2_MSG_TYPE_EVENT;
    kernel_pid_t *pid = (kernel_pid_t*) dev->context;
    size_t len;
    struct netdev2_radio_rx_info rx_info;
    RadioEvents_t *events = radio_get_event_ptr();
    switch(event)
    {
        case NETDEV2_EVENT_ISR:
            msg_send(&msg, *pid);
            break;
        case NETDEV2_EVENT_TX_COMPLETE:
            puts("sx1276: TX done");
            printf("TX done, COUNT : %lu \r\n",count);
            count++;
            events->TxDone();
            break;
        case NETDEV2_EVENT_TX_TIMEOUT:
            msg.type = LORAWAN_TX_TIMEOUT_EVENT;
            msg_send(&msg, *pid);
            break;
        case NETDEV2_EVENT_RX_COMPLETE:
            rx_done_flag++;
            len = dev->driver->recv(dev, NULL, 5, &rx_info);
            dev->driver->recv(dev, message, len, NULL);
            printf("%s\n. {RSSI: %i, SNR: %i}", message, rx_info.rssi, (signed int) rx_info.lqi);
            puts("sx1276: RX Done");
            events->RxDone(message, len, (signed int) rx_info.rssi, (signed int) rx_info.lqi);
            break;
        case NETDEV2_EVENT_RX_TIMEOUT:
            rx_timeout_flag++;
            msg.type = LORAWAN_RX_TIMEOUT_EVENT;
            msg_send(&msg, *pid);
            break;
        case NETDEV2_EVENT_CRC_ERROR:
            msg.type = LORAWAN_CRC_ERROR;
            msg_send(&msg, *pid);
            break;
        case NETDEV2_EVENT_FHSS_CHANGE_CHANNEL:
            msg.type = LORAWAN_FHSS_CHANGE_CHANNEL;
            msg_send(&msg, *pid);
            break;
        case NETDEV2_EVENT_CAD_DONE:
            msg.type = LORAWAN_CAD_DONE;
            msg_send(&msg, *pid);
            break;
        default:
            break;
    }
}


uint8_t TimerLowPowerHandler(void)
{
    /* Loops 5 times threw main before going to LPM*/
    if( HasLoopedThroughMain < 5 )
        {
            HasLoopedThroughMain++;
            return 0;
        }
        else
        {
            /* Resets flags, puts lora driver to sleep, enters LPM*/
            rx_done_flag = 0;
            rx_timeout_flag = 0;
            HasLoopedThroughMain = 0;
            return 1;
        }   
}

#define SX1276_MAC_STACKSIZE    (THREAD_STACKSIZE_DEFAULT)

static char stack[SX1276_MAC_STACKSIZE];
/**
 * Main application entry point.
 */
int main( void )
{
    /* Driver, timer and rtc setup*/
    app_init(&sx1276);

    /* Init netdev layer*/
    memcpy(&sx1276.params, sx1276_params, sizeof(sx1276_params));
    netdev2_t *netdev = (netdev2_t*) &sx1276;
    nd = netdev;
    netdev->driver = &sx1276_driver;
    netdev->driver->init(netdev);
    netdev->event_callback = _event_cb;

    /* Set-up Thread*/
    kernel_pid_t pid;
    pid = thread_create(stack, sizeof(stack), THREAD_PRIORITY_MAIN - 5, THREAD_CREATE_STACKTEST,
                     _event_loop, (void *) netdev, "asd");
    netdev->context = &pid;

    /* LoRaMac Initiation*/
    LoRaMacCallbacks.MacEvent = OnMacEvent;
    LoRaMacCallbacks.GetBatteryLevel = board_get_battery_level;
    LoRaMacInit( &LoRaMacCallbacks, pid );
#if( OVER_THE_AIR_ACTIVATION != 0 )
    uint8_t sendFrameStatus = 0;
#endif
    bool trySendingFrameAgain = false;

    /* Initialize LoRaMac device unique ID, init periph */
    #ifdef NZ32_SC151
        board_init_periph();
        board_get_unique_id( DevEui );
        printf("BoardID: ");
        for(uint8_t i = 0; i < 8; i++)
        {
            printf("%02x ",*(DevEui + i));
        }
        printf("\n");

        /* Get power source*/
        if(get_board_power_source() == BATTERY_POWER)
            DEBUG("BATTERY_POWER \n");
        else
            DEBUG("USB POWER \n");
    #endif

    /* get unique node id*/
    board_get_node_id(DevEui, 4, NodeId);

    IsNetworkJoined = false;

#if( OVER_THE_AIR_ACTIVATION == 0 )
    if( DevAddr == 0 )
    {
        // Random seed initialization
        srand1( board_get_random_seed( ) );
        // Choose a random device address
        DevAddr = randr( 0, 0x01FFFFFF );
    }
    LoRaMacInitNwkIds( LORAWAN_NETWORK_ID, DevAddr, NwkSKey, AppSKey );
    IsNetworkJoined = true;
    // Schedules a packet transmision
    OnTxNextPacketTimerEvent(NULL);
#else


    // Schedules a Join Request
    OnJoinReqTimerEvent(NULL);
#endif

    LoRaMacSetAdrOn( LORAWAN_ADR_ON );
    LoRaMacSetPublicNetwork( LORAWAN_PUBLIC_NETWORK );

    while( 1 )
    {
        /* Join request OTA loop*/
        while( IsNetworkJoined == false )
        {
#if( OVER_THE_AIR_ACTIVATION != 0 )
            if( TxNextPacket == true )
            {
                TxNextPacket = false;
                
                sendFrameStatus = LoRaMacJoinReq( DevEui, AppEui, AppKey );
                switch( sendFrameStatus )
                {
                case 1: // BUSY
                    DEBUG("BUSY \n");
                    break;
                case 0: // OK
                    DEBUG("OK \n");
                    break;
                case 2: // NO_NETWORK_JOINED
                    DEBUG("NO_NETWORK_JOINED \n");
                    break;
                case 3: // LENGTH_PORT_ERROR
                    DEBUG("LENGTH_PORT_ERROR \n");
                    break;
                case 4: // MAC_CMD_ERROR
                    DEBUG("MAC_CMD_ERROR \n");
                    break;
                case 6: // DEVICE_OFF
                    DEBUG("DEVICE_OFF \n");
                    break;
                default:
                    break;
                }
            }
            /* Send to sleep if 2 RxTimeout, 1 Rx done, and Mac layer is idle */
            if((rx_done_flag || (rx_timeout_flag >= 2)) && ( LoRaMacState  != LORAMAC_STATUS_BUSY ))
                TimerLowPowerHandler();

            /* Reset counters */
            if(IsNetworkJoined == true)
            {
                count = 0;
            }
#endif
        }
        
    /* Normal Transmition loop*/
        if( DownlinkStatusUpdate == true )
        {
            DownlinkStatusUpdate = false;
        }

        if( trySendingFrameAgain == true )
        {
            trySendingFrameAgain = SendFrame( );
        }
        if( TxNextPacket == true )
        {
            TxNextPacket = false;

            PrepareTxFrame( AppPort );
            trySendingFrameAgain = SendFrame( );
        }

    /* Send to sleep if 2 RxTimeout, 1 Rx done, and Mac layer is idle */
        if((rx_done_flag || (rx_timeout_flag >= 2)) && ( LoRaMacState  != LORAMAC_STATUS_BUSY ))
            TimerLowPowerHandler();
    }

    return 0;
}
