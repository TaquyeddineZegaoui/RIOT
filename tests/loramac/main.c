/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech
Description: LoRaMac classA device implementation
License: Revised BSD License, see LICENSE.TXT file include in the project
Maintainer: Miguel Luis and Gregory Cristian
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
#include "periph/rtc.h"
#include "random.h"

#include "common.h"
#include "board.h"
#include "boards_hw.h"
#include "infrared.h"

#include "sx1276_regs_lora.h"
#include "sx1276_regs_fsk.h"

#include "LoRaMac.h"
#include "net/lorawan/board_definitions.h" 
#include "Comissioning.h"
#include "LoRaMac-api-v3.h"
#include "net/gnrc/netdev2.h"
#include "net/netdev2.h"
#include "sx1276_params.h"
#include "sx1276_netdev.h"

#define GNRC_LORA_MSG_QUEUE 16

#define LORAWAN_TX_TIMEOUT_EVENT 10
#define LORAWAN_RX_TIMEOUT_EVENT 11
#define LORAWAN_CRC_ERROR 12
#define LORAWAN_FHSS_CHANGE_CHANNEL 13
#define LORAWAN_CAD_DONE 14
/*!
 * Thread Variables and packet count
 */
volatile uint8_t HasLoopedThroughMain = 0;
uint32_t count = 0;
static sx1276_t sx1276;

static netdev2_t *nd;
/*!
*   Node id
*/
static uint8_t NodeId[] = {0x00, 0x00, 0x00, 0x00};

/*
* Drope time
*/
#define DROP_TIME                                   (4*SEC_IN_USEC)

/*!
 * Join requests trials duty cycle.
 */
#define OVER_THE_AIR_ACTIVATION_DUTYCYCLE           10000 // 10 [s] value in ms

/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            15*1000

/*!
 * Defines a random delay for application data transmission duty cycle. 1s,
 * value in [ms].
 */
#define APP_TX_DUTYCYCLE_RND                        1000

/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_CONFIRMED_MSG_ON                    false

/*!
 * LoRaWAN Adaptive Data Rate
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_ON                              0

/*!
 * LoRaWAN application port
 */
#define LORAWAN_APP_PORT                            2

/*!
 * User application data buffer size
 */

#if defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )

#define LORAWAN_APP_DATA_SIZE                       11

#endif

#if( OVER_THE_AIR_ACTIVATION != 0 )

static uint8_t DevEui[] = LORAWAN_DEVICE_EUI;
static uint8_t AppEui[] = LORAWAN_APPLICATION_EUI;
static uint8_t AppKey[] = LORAWAN_APPLICATION_KEY;

#else
    #ifdef NZ32_SC151
    static uint8_t DevEui[] = LORAWAN_DEVICE_EUI;
    #endif
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
 * Defines the application data transmission duty cycle
 */
static uint32_t TxDutyCycleTime;

/*!
 * Timer to handle the application data transmission duty cycle
 */
static TimerEvent_t TxNextPacketTimer;
static uint64_t TxNextPacketTimerInit   = 0;
static uint64_t TxNextPacketTimerActual = 0;

#if( OVER_THE_AIR_ACTIVATION != 0 )

/*!
 * Defines the join request timer
 */
static TimerEvent_t JoinReqTimer;

#endif

/*!
 * Indicates if a new packet can be sent
 */
static bool TxNextPacket = true;
static bool ScheduleNextTx = true;
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
            if(count_drops(DROP_TIME))
                AppData[8] = 0xFF;
            else
                AppData[8] = 0xAA;

            uint16_t time = get_time();
            AppData[0] = NodeId[0];
            AppData[1] = NodeId[1];
            AppData[2] = NodeId[2];
            AppData[3] = NodeId[3];
            AppData[4] = get_drops();
            AppData[5] = (uint8_t) (time & 0xff);
            AppData[6] = (uint8_t) ((time >> 8) & 0xff);
            AppData[7] = board_get_battery_level();
        }

    case 2:
        {
            if(count_drops(DROP_TIME))
                AppData[8] = 0xFF;
            else
                AppData[8] = 0xAA;

            uint16_t time = get_time();
            AppData[0] = NodeId[0];
            AppData[1] = NodeId[1];
            AppData[2] = NodeId[2];
            AppData[3] = NodeId[3];
            AppData[4] = get_drops();
            AppData[5] = (uint8_t) (time & 0xff);
            AppData[6] = (uint8_t) ((time >> 8) & 0xff);
            AppData[7] = board_get_battery_level();

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

    if( IsTxConfirmed == false )
    {
        sendFrameStatus = LoRaMacSendFrame( AppPort, AppData, AppDataSize );
    }
    else
    {
        sendFrameStatus = LoRaMacSendConfirmedFrame( AppPort, AppData, AppDataSize, 8 );
    }

    switch( sendFrameStatus )
    {
    case 3: // LENGTH_ERROR
        // Send empty frame in order to flush MAC commands
        LoRaMacSendFrame( 0, NULL, 0 );
        return false;
    case 5: // NO_FREE_CHANNEL
        // Try again later
        return true;
    default:
        return false;
    }
}

#if( OVER_THE_AIR_ACTIVATION != 0 )

/*!
 * \brief Function executed on JoinReq Timeout event
 */
static void OnJoinReqTimerEvent( void )
{
    //TimerStop( &JoinReqTimer );
    xtimer_remove(&JoinReqTimer.dev);
    TxNextPacket = true;
}

#endif

/*!
 * \brief Function executed on TxNextPacket Timeout event
 */
static void OnTxNextPacketTimerEvent( void )
{
    //TimerStop( &TxNextPacketTimer );
    xtimer_remove(&TxNextPacketTimer.dev);
    TxNextPacket = true;
    ScheduleNextTx = true;
}

/*!
 * \brief Function to be executed on MAC layer event
 */
static void OnMacEvent( LoRaMacEventFlags_t *flags, LoRaMacEventInfo_t *info )
{
    if( flags->Bits.JoinAccept == 1 )
    {
    #if( OVER_THE_AIR_ACTIVATION != 0 )
            //TimerStop( &JoinReqTimer );
    xtimer_remove(&JoinReqTimer.dev);
    #endif
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
    // Schedule a new transmission
    // ScheduleNextTx = true;
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
            len = dev->driver->recv(dev, NULL, 5, &rx_info);
            dev->driver->recv(dev, message, len, NULL);
            printf("%s\n. {RSSI: %i, SNR: %i}", message, rx_info.rssi, (signed int) rx_info.lqi);
            puts("sx1276: RX Done");
            events->RxDone(message, len, (signed int) rx_info.rssi, (signed int) rx_info.lqi);
            break;
        case NETDEV2_EVENT_RX_TIMEOUT:
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

void TimerLowPowerHandler(uint32_t time)
{
    if( HasLoopedThroughMain < 5 )
        {
            HasLoopedThroughMain++;
        }
        else
        {
            HasLoopedThroughMain = 0;
            xtimer_usleep (time*1000);
        }   
}
#define SX1276_MAC_STACKSIZE    (THREAD_STACKSIZE_DEFAULT)

static char stack[SX1276_MAC_STACKSIZE];
/**
 * Main application entry point.
 */
int main( void )
{

#if( OVER_THE_AIR_ACTIVATION != 0 )
    uint8_t sendFrameStatus = 0;
#endif
    bool trySendingFrameAgain = false;

    /* set sx1276 pointer, init xtimer */
    radio_set_ptr(&sx1276);
    xtimer_init();

    memcpy(&sx1276.params, sx1276_params, sizeof(sx1276_params));
    netdev2_t *netdev = (netdev2_t*) &sx1276;
    nd = netdev;
    netdev->driver = &sx1276_driver;
    netdev->driver->init(netdev);
    netdev->event_callback = _event_cb;

    kernel_pid_t pid;
    pid = thread_create(stack, sizeof(stack), THREAD_PRIORITY_MAIN - 5, THREAD_CREATE_STACKTEST,
                     _event_loop, (void *) netdev, "asd");
    netdev->context = &pid;

    /* get unique node id*/
    #ifdef NZ32_SC151

    /* initialize all available ADC lines */
    adc_init(ADC_VDIV);
    adc_init(ADC_LINE(0));
    adc_init(ADC_LINE(1));
    adc_init(ADC_LINE(2));
    adc_init(ADC_VREF);
    gpio_init(USB_DETECT, GPIO_IN);
    gpio_init(EMITTER_PIN, GPIO_OUT);
    gpio_init(BAT_LEVEL, GPIO_OUT);
    gpio_clear(BAT_LEVEL);

    board_get_unique_id( DevEui );
    board_get_node_id(DevEui, 4, NodeId);
    
    /* get power source*/
    if(get_board_power_source() == BATTERY_POWER)
        puts("BATTERY_POWER");
    else
        puts("USB POWER");
    #endif

    LoRaMacCallbacks.MacEvent = OnMacEvent;
    LoRaMacCallbacks.GetBatteryLevel = board_get_battery_level;
    LoRaMacInit( &LoRaMacCallbacks, pid );

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
#else
    // Initialize LoRaMac device unique ID
    #ifdef NZ32_SC151
    board_get_unique_id( DevEui );
    printf("BoardID: ");
    for(uint8_t i = 0; i < 8; i++)
    {
        printf("%02x ",*(DevEui + i));
    }
    printf("\n");
    #endif

    // Sends a JoinReq Command every OVER_THE_AIR_ACTIVATION_DUTYCYCLE
    // seconds until the network is joined
    //TimerInit( &JoinReqTimer, OnJoinReqTimerEvent, (kernel_pid_t) 0);
    JoinReqTimer.dev.target = 0;
    JoinReqTimer.dev.callback =  (void*) OnJoinReqTimerEvent;
    JoinReqTimer.pid =  (kernel_pid_t) 0;
    //TimerSetValue( &JoinReqTimer, OVER_THE_AIR_ACTIVATION_DUTYCYCLE, 0);
#endif

    TxNextPacket = true;
    //TimerInit( &TxNextPacketTimer, OnTxNextPacketTimerEvent, (kernel_pid_t) 0 );
    TxNextPacketTimer.dev.target = 0;
    TxNextPacketTimer.dev.callback =  (void*) OnTxNextPacketTimerEvent;
    TxNextPacketTimer.pid = (kernel_pid_t) 0;

    LoRaMacSetAdrOn( LORAWAN_ADR_ON );
    LoRaMacSetPublicNetwork( LORAWAN_PUBLIC_NETWORK );

    while( 1 )
    {
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
                    puts("BUSY");
                    break;
                case 0: // OK
                    puts("OK");
                    break;
                case 2: // NO_NETWORK_JOINED
                    puts("NO_NETWORK_JOINED");
                    break;
                case 3: // LENGTH_PORT_ERROR
                    puts("LENGTH_PORT_ERROR");
                    break;
                case 4: // MAC_CMD_ERROR
                    puts("MAC_CMD_ERROR");
                    break;
                case 6: // DEVICE_OFF
                    puts("DEVICE_OFF");
                    break;
                default:
                    // Relaunch timer for next trial
                    xtimer_set(&(JoinReqTimer.dev), xtimer_ticks_from_usec(OVER_THE_AIR_ACTIVATION_DUTYCYCLE*1000).ticks32); 
                    //TimerStart( &JoinReqTimer, 1);
                    break;
                }
            }
            // Send to sleep if TxDone
            if(IsTxConfirmed == true)
                TimerLowPowerHandler(OVER_THE_AIR_ACTIVATION_DUTYCYCLE);
#endif
        }
        
        if( DownlinkStatusUpdate == true )
        {
            DownlinkStatusUpdate = false;
            // Switch LED 2 ON for each received downlink
        }

        if( ScheduleNextTx == true)
        {
            ScheduleNextTx = false;

            if( ComplianceTest.Running == true )
            {
                TxNextPacket = true;
            }
            else
            {
                // Schedule next packet transmission
                TxDutyCycleTime = APP_TX_DUTYCYCLE /*+ randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND )*/;
                TxNextPacketTimerInit = xtimer_now_usec64();

                xtimer_set(&(TxNextPacketTimer.dev), xtimer_ticks_from_usec(TxDutyCycleTime*1000).ticks32); 
                //TimerSetValue( &TxNextPacketTimer, TxDutyCycleTime, 0);
                //TimerStart( &TxNextPacketTimer, 1);
            }
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

        // Send to sleep if TxDone
        if(IsTxConfirmed == true)
        {
            TxNextPacketTimerActual = xtimer_now_usec64() - TxNextPacketTimerInit ;
            TimerLowPowerHandler( (uint32_t) TxDutyCycleTime);
        }
    }

    return 0;
}
