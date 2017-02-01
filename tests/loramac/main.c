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
#include "Comissioning.h"
#include "LoRaMac-api-v3.h"
#include "net/gnrc/netdev2.h"
#include "net/netdev2.h"
#include "sx1276_params.h"
#include "sx1276_netdev.h"
#include "LoRaMac.h"

#define GNRC_LORA_MSG_QUEUE 16

#define LORAWAN_TX_TIMEOUT_EVENT 10
#define LORAWAN_RX_TIMEOUT_EVENT 11
#define LORAWAN_CRC_ERROR 12
#define LORAWAN_FHSS_CHANGE_CHANNEL 13
#define LORAWAN_CAD_DONE 14
/*!
 * Thread Variables and packet count
 */
uint32_t count = 0;
static sx1276_t sx1276;

static netdev2_t *nd;

/*!
 * LoRaWAN confirmed messages
 */
#ifndef LORAWAN_CONFIRMED_MSG_ON 
    #define LORAWAN_CONFIRMED_MSG_ON                    false
#endif

/*!
 * LoRaWAN Adaptive Data Rate
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 */
#ifndef LORAWAN_ADR_ON 
    #define LORAWAN_ADR_ON                              0
#endif

/*!
 * LoRaWAN application port
 */
#ifndef LORAWAN_APP_PORT 
    #define LORAWAN_APP_PORT                            2
#endif

/*!
 * User application data buffer size
 */

#if defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )

#define LORAWAN_APP_DATA_SIZE                       11

#endif


extern LoRaMacStatus_t Send( LoRaMacHeader_t *macHdr, uint8_t fPort,
                             void *fBuffer, uint16_t fBufferSize );
static uint8_t DevEui[] = LORAWAN_DEVICE_EUI;
static uint8_t AppEui[] = LORAWAN_APPLICATION_EUI;
static uint8_t AppKey[] = LORAWAN_APPLICATION_KEY;

static uint8_t NwkSKey[] = LORAWAN_NWKSKEY;
static uint8_t AppSKey[] = LORAWAN_APPSKEY;

/*!
 * Device address
 */
static uint32_t DevAddr = LORAWAN_DEVICE_ADDRESS;

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
static uint8_t PrepareTxFrame( uint8_t port, uint8_t* data, uint8_t size)
{
    AppPort = port;
    AppDataSize = size;

    switch( port )
    {
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
    {
        if(size > LORAWAN_APP_DATA_SIZE )
        {
            puts("ERROR: max data size exceeded");
            return 0;
        }

        for(uint8_t i = 0; i < size; i++)
            AppData[i] = *data++;
        break;
    }
    }
    return 1;
}

static void ProcessRxFrame(LoRaMacEventInfo_t *info )
{
    kernel_pid_t pid = *((kernel_pid_t*) nd->context);
    netdev2_lorawan_t *dev = (netdev2_lorawan_t*) &sx1276;
    uint8_t res;

    switch( info->RxPort ) // Check Rx port number
    {
    case 1: // The application LED can be controlled on port 1 or 2
    case 2:
        if( dev->BufferSize == 1 )
        {
        }
        break;
    case 224:
        if( ComplianceTest.Running == false )
        {
            // Check compliance test enable command (i)
            if( ( dev->BufferSize == 4 ) && 
                ( dev->Buffer[0] == 0x01 ) &&
                ( dev->Buffer[1] == 0x01 ) &&
                ( dev->Buffer[2] == 0x01 ) &&
                ( dev->Buffer[3] == 0x01 ) )
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
                
                bool adr = true;
                gnrc_netapi_set(*((kernel_pid_t*)nd->context), NETOPT_LORAWAN_ADR, 0, &(adr), sizeof(uint8_t));
            }
        }
        else
        {
            ComplianceTest.State = dev->Buffer[0];
            switch( ComplianceTest.State )
            {
            case 0: // Check compliance test disable command (ii)
                IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;
                AppPort = LORAWAN_APP_PORT;
                AppDataSize = LORAWAN_APP_DATA_SIZE;
                ComplianceTest.DownLinkCounter = 0;
                ComplianceTest.Running = false;
                bool adr = LORAWAN_ADR_ON;
                gnrc_netapi_set(*((kernel_pid_t*)nd->context), NETOPT_LORAWAN_ADR, 0, &(adr), sizeof(uint8_t));
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
                AppDataSize = dev->BufferSize;

                AppData[0] = 4;
                for( uint8_t i = 1; i < AppDataSize; i++ )
                {
                    AppData[i] = dev->Buffer[i] + 1;
                }
                break;
            case 5: // (viii)
                gnrc_netapi_get(pid, NETOPT_LORAWAN_LINK_CHECK,0, &res, sizeof(uint8_t));
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
    msg_t msg;
    lorawan_send_t lws;
    lws.port = AppPort;
    lws.buffer = AppData;
    lws.size = AppDataSize;
    lws.retries = 8;
    msg.type = GNRC_NETAPI_MSG_TYPE_SND;
    msg.content.ptr = &lws;
    kernel_pid_t pid = *((kernel_pid_t*) nd->context);

    if( IsTxConfirmed == false )
    {
        lws.type = 0;
    }
    else
    {
        lws.type = 1;
    }
    msg_send(&msg, pid);

    switch( sendFrameStatus )
    {
    case 3: // LENGTH_ERROR
        // Send empty frame in order to flush MAC commands
        //TODO: Fix this
        //LoRaMacSendFrame( 0, NULL, 0 );
        return false;
    case 5: // NO_FREE_CHANNEL
        // Try again later
        return true;
    default:
        return false;
    }
}

/*!
 * \brief Function to be executed on MAC layer event
 */
static void OnMacEvent(LoRaMacEventInfo_t *info )
{
    netdev2_lorawan_t *dev = (netdev2_lorawan_t*) &sx1276;
    if( dev->join_req == 1 )
    {
        IsNetworkJoined = true;
    }
    else
    {
        if( dev->b_tx == 1 )
        {
        }

        if( dev->b_rx == 1 )
        {
            if( ComplianceTest.Running == true )
            {
                ComplianceTest.DownLinkCounter++;
                if( dev->link_check == 1 )
                {
                    ComplianceTest.LinkCheck = true;
                    ComplianceTest.DemodMargin = info->DemodMargin;
                    ComplianceTest.NbGateways = info->NbGateways;
                }
            }
            if( dev->RxData == true )
            {
                ProcessRxFrame(info );
            }

            DownlinkStatusUpdate = true;
        }
    }
}

int lorawan_setup(int argc, char **argv) {
    netdev2_t *netdev = (netdev2_t*) &sx1276;


    if (argc < 4) {
        puts("ERROR: Not enough arguments");
        return -1;
    }

    if(strstr(argv[1], "ota") != NULL)
    {
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
        IsNetworkJoined = false;

        kernel_pid_t pid = *((kernel_pid_t*) nd->context);
        uint8_t res;
        gnrc_netapi_set(pid, NETOPT_LORAWAN_DEV_EUI,0, DevEui, sizeof(uint8_t*));
        gnrc_netapi_set(pid, NETOPT_LORAWAN_APP_EUI,0, AppEui, sizeof(uint8_t*));
        gnrc_netapi_set(pid, NETOPT_LORAWAN_APP_KEY,0, AppKey, sizeof(uint8_t*));
        gnrc_netapi_get(pid, NETOPT_LORAWAN_JOIN,0, &res, sizeof(uint8_t));

        puts("Activation Type: OTA");
    }
    else if(strstr(argv[1], "abp") != NULL)
    {
        if( DevAddr == 0 )
        {
            uint32_t netid = LORAWAN_NETWORK_ID;

            // Random seed initialization
            random_init( board_get_random_seed( ) );
            // Choose a random device address
            DevAddr = random_uint32_range( 0, 0x01FFFFFF+1 );
            // Get sesion Keys
            gnrc_netapi_set(*((kernel_pid_t*)netdev->context), NETOPT_ADDRESS, 0, &DevAddr, sizeof(DevAddr));
            gnrc_netapi_set(*((kernel_pid_t*)netdev->context), NETOPT_LORAWAN_APP_SKEY, 0, AppSKey, sizeof(AppKey)/sizeof(AppSKey[0]));
            gnrc_netapi_set(*((kernel_pid_t*)netdev->context), NETOPT_LORAWAN_NWK_SKEY, 0, NwkSKey, sizeof(NwkSKey)/sizeof(NwkSKey[0]));
            gnrc_netapi_set(*((kernel_pid_t*)netdev->context), NETOPT_LORAWAN_NET_ID, 0, &(netid), sizeof(netid));
            IsNetworkJoined = true;
        }
        puts("Activation Type: ABP");
    }
    else
    {
        puts("ERROR: Invalid activation type");
        return -1;
    }

    uint8_t adr;
    if(strstr(argv[2], "on") != NULL)
    {
        adr = 1;
        gnrc_netapi_set(*((kernel_pid_t*)netdev->context), NETOPT_LORAWAN_ADR, 0, &(adr), sizeof(uint8_t));
        puts("Adaptative Data Rate: ON");
    }
    else if(strstr(argv[2], "off") != NULL)
    {
        adr = 0;
        gnrc_netapi_set(*((kernel_pid_t*)netdev->context), NETOPT_LORAWAN_ADR, 0, &(adr), sizeof(uint8_t));
        puts("Adaptative Data Rate: OFF");
    }
    else
    {
        puts("ERROR: Invalid Adaptative Data Rate Setting");
        return -1;
    }

    uint8_t sw;
    if(strstr(argv[3], "public") != NULL)
    {
        sw = true;
        gnrc_netapi_set(*((kernel_pid_t*)netdev->context), NETOPT_LORAWAN_NWK_TYPE, 0, &(sw), sizeof(uint8_t));
        puts("Network: PUBLIC");
    }
    else if(strstr(argv[3], "private") != NULL)
    {
        sw = false;
        gnrc_netapi_set(*((kernel_pid_t*)netdev->context), NETOPT_LORAWAN_NWK_TYPE, 0, &(sw), sizeof(uint8_t));
        puts("Network: PRIVATE");
    }
    else
    {
        puts("ERROR: Invalid Network Type");
        return -1;
    }

    puts("lorawan_setup: configuration is set");

    return 0;
}

int send(int argc, char **argv)
{
    
    if (argc <= 3) {
        puts("ERROR: Not enough arguments");
        return -1;
    }
    if(strstr(argv[1], "unconfirmed") != NULL)
    {
        IsTxConfirmed = false;
        puts("Message Type: Unconfirmed");
    }
    else if(strstr(argv[1], "confirmed") != NULL)
    {
        IsTxConfirmed = true;
        puts("Message Type: Confirmed");
    }
    else
    {
        puts("ERROR: Invalid Message Type");
        return -1;
    }


    uint8_t port = atoi(argv[2]);
    if(port >243)
    {
        puts("ERROR: Invalid AppPort");
        return -1;
    }

    puts("Preparing frame");

    uint8_t size = strlen(argv[3]);
    if(size>254)
    {
        puts("ERROR: Input string bigger max physical payload");
        return -1;
    }
    uint8_t data[255];
    memcpy(data,argv[3],size);

    if(PrepareTxFrame( port, data, size))
    {
        SendFrame( );
        puts("Frame sent");
    }    
    
    return 0;
}

static int _send(lorawan_send_t *lws)
{
    //LoRaMacSendFrame(lws->port, lws->buffer, lws->size);
    netdev2_lorawan_t *dev = (netdev2_lorawan_t*) &sx1276;
    LoRaMacStatus_t status = LORAMAC_STATUS_SERVICE_UNKNOWN;
    LoRaMacHeader_t macHdr;

    if( ( ( dev->LoRaMacState & MAC_TX_RUNNING ) == MAC_TX_RUNNING ) ||
        ( ( dev->LoRaMacState & MAC_TX_DELAYED ) == MAC_TX_DELAYED ) )
    {
        return LORAMAC_STATUS_BUSY;
    }

    macHdr.Value = 0;
    dev->frame_status = LORAMAC_EVENT_INFO_STATUS_ERROR;

    switch(lws->type)
    {
        case 0:
        {
            dev->AckTimeoutRetries = 1;

            macHdr.Bits.MType = FRAME_TYPE_DATA_UNCONFIRMED_UP;
            break;
        }
        case 1:
        {
            dev->AckTimeoutRetriesCounter = 1;
            dev->AckTimeoutRetries = lws->retries;

            macHdr.Bits.MType = FRAME_TYPE_DATA_CONFIRMED_UP;
            break;
        }
        /*
        case MCPS_PROPRIETARY:
        {
            readyToSend = true;
            dev->AckTimeoutRetries = 1;

            macHdr.Bits.MType = FRAME_TYPE_PROPRIETARY;
            fBuffer = dev->fBuffer;
            fBufferSize = dev->fBufferSize;
            break;
        }
        */
        default:
            break;
    }

    status = Send( &macHdr, lws->port, lws->buffer, lws->size );
    if( status == LORAMAC_STATUS_OK )
    {
        //dev->McpsConfirm.McpsRequest = mcpsRequest->Type;
        dev->LoRaMacFlags.Bits.McpsReq = 1;
    }
    else
    {
        dev->NodeAckRequested = false;
    }

    return status;
}

static const shell_command_t shell_commands[] = {
    { "lorawan_setup", "<ActivationType (ota, abp)> <ADR (on, off)> <NetworkType (public, private)> - sets up LoRaWAN settings", lorawan_setup},
    { "send", "<Type (confirmed, unconfirmed)> <AppPort(1..243)> <payload> ", send },
    { NULL, NULL, NULL }
};

void *_event_loop(void *arg)
{
    static msg_t _msg_q[GNRC_LORA_MSG_QUEUE];
    msg_t msg, reply;
    netdev2_t *netdev = (netdev2_t*) arg;
    msg_init_queue(_msg_q, GNRC_LORA_MSG_QUEUE);
    lorawan_send_t *lws;
    gnrc_netapi_opt_t *opt;
    int res;

    while (1) {
        msg_receive(&msg);
        switch (msg.type) {
            case NETDEV2_MSG_TYPE_EVENT:
                netdev->driver->isr(netdev);
                break;
            case GNRC_NETAPI_MSG_TYPE_SND:
                lws = msg.content.ptr;
                _send(lws);
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
                OnRadioTxTimeout(netdev);
                break;
            case LORAWAN_RX_TIMEOUT_EVENT:
                puts("sx1276: RX timeout");
                OnRadioRxTimeout(netdev);
                break;
            case LORAWAN_CRC_ERROR:
                puts("sx1276: RX CRC_ERROR");
                OnRadioRxError(netdev);
                break;
            case LORAWAN_FHSS_CHANGE_CHANNEL:
                assert(false);
                //events->FhssChangeChannel(sx1276._internal.last_channel);
                break;
            case LORAWAN_CAD_DONE:
                assert(false);
                //events->CadDone(sx1276._internal.is_last_cad_success);
                break;
            case LORAWAN_TIMER_MAC_STATE:
                OnMacStateCheckTimerEvent(netdev);
                break;
            case LORAWAN_TIMER_RX_WINDOW1:
                OnRxWindow1TimerEvent(netdev);
                break;
            case LORAWAN_TIMER_RX_WINDOW2:
                OnRxWindow2TimerEvent(netdev);
                break;
            case LORAWAN_TIMER_ACK_TIMEOUT:
                OnAckTimeoutTimerEvent(netdev);
                break;
            case LORAWAN_TIMER_TX_DELAYED:
                OnTxDelayedTimerEvent(netdev);
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
    switch(event)
    {
        case NETDEV2_EVENT_ISR:
            msg_send(&msg, *pid);
            break;
        case NETDEV2_EVENT_TX_COMPLETE:
            puts("sx1276: TX done");
            printf("TX done, COUNT : %lu \r\n",count);
            count++;
            OnRadioTxDone(dev);
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
            OnRadioRxDone(dev, message, len, (signed int) rx_info.rssi, (signed int) rx_info.lqi);
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

#define SX1276_MAC_STACKSIZE    (THREAD_STACKSIZE_DEFAULT)

static char stack[SX1276_MAC_STACKSIZE];
/**
 * Main application entry point.
 */
int main( void )
{
    
    /* set sx1276 pointer, init xtimer */
    lorawan_set_pointer((netdev2_lorawan_t*) &sx1276);
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

    // Set LoRaMAC thread
    LoRaMacCallbacks.MacEvent = OnMacEvent;
    LoRaMacCallbacks.GetBatteryLevel = board_get_battery_level;
    LoRaMacInit( &LoRaMacCallbacks, pid );

    /* start the shell */
    puts("Initialization successful - starting the shell now");
    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    while(1){}

    return 0;
}
