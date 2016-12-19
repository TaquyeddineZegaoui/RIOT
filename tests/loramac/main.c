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

#include "sx1276_regs_lora.h"
#include "sx1276_regs_fsk.h"

#include "LoRaMac.h"
#include "loramac/board_definitions.h" 
#include "Comissioning.h"
#include "LoRaMac-api-v3.h"

/*!
 * Thread Variables and packet count
 */
uint32_t count = 0;
static sx1276_t sx1276;


/*!
*   Node id
*/
#define         NODEID                              0x01

/*!
 * Join requests trials duty cycle.
 */
#define OVER_THE_AIR_ACTIVATION_DUTYCYCLE           1000 // 10 [s] value in ms

/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            5000

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
#define LORAWAN_ADR_ON                              1

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
static bool ScheduleNextTx = false;
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
            AppData[0] = NODEID;
            AppData[1] = get_drops();
            AppData[2] = get_measured_time();
            AppData[3] = board_get_battery_level();
            AppData[4] = 0xFF;
        }

    case 2:
        {
            AppData[0] = NODEID;
            AppData[1] = get_drops();
            AppData[2] = get_measured_time();
            AppData[3] = board_get_battery_level();
            AppData[4] = 0xFF;
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
    TimerStop( &JoinReqTimer );
    TxNextPacket = true;
}

#endif

/*!
 * \brief Function executed on TxNextPacket Timeout event
 */
static void OnTxNextPacketTimerEvent( void )
{
    TimerStop( &TxNextPacketTimer );
    TxNextPacket = true;
}

/*!
 * \brief Function to be executed on MAC layer event
 */
static void OnMacEvent( LoRaMacEventFlags_t *flags, LoRaMacEventInfo_t *info )
{
    if( flags->Bits.JoinAccept == 1 )
    {
    #if( OVER_THE_AIR_ACTIVATION != 0 )
            TimerStop( &JoinReqTimer );
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
    ScheduleNextTx = true;
}

void event_handler_thread(void *arg, sx1276_event_type_t event_type)
{
    sx1276_rx_packet_t *packet = (sx1276_rx_packet_t *) &sx1276._internal.last_packet;
    RadioEvents_t *events = radio_get_event_ptr();
    switch (event_type) {

        case SX1276_TX_DONE:
            //puts("sx1276: TX done");
            printf("TX done, COUNT : %lu \r\n",count);
            count++;
            events->TxDone();
            break;

        case SX1276_TX_TIMEOUT:
            puts("sx1276: TX timeout");
            events->TxTimeout();
            break;

        case SX1276_RX_DONE:
            puts("sx1276: RX Done");
            events->RxDone(packet->content, packet->size, packet->rssi_value, packet-> snr_value);
            break;

        case SX1276_RX_TIMEOUT:
            puts("sx1276: RX timeout");
            events->RxTimeout();
            break;

        case SX1276_RX_ERROR_CRC:
            puts("sx1276: RX CRC_ERROR");
            events->RxError();
            break;

        case SX1276_FHSS_CHANGE_CHANNEL:
            events->FhssChangeChannel(sx1276._internal.last_channel);
            break;

        case SX1276_CAD_DONE:
            events->CadDone(sx1276._internal.is_last_cad_success);
            break;

        default:
            break;
    }
}

void init_radio(void)
{
    sx1276_lora_settings_t settings_lora;
    sx1276_settings_t settings;

    sx1276.nss_pin = SX1276_SPI_NSS;
    sx1276.spi = SX1276_SPI;

    sx1276.dio0_pin = SX1276_DIO0;
    sx1276.dio1_pin = SX1276_DIO1;
    sx1276.dio2_pin = SX1276_DIO2;
    sx1276.dio3_pin = SX1276_DIO3;

    sx1276.dio4_pin = (gpio_t) NULL;
    sx1276.dio5_pin = (gpio_t) NULL;
    sx1276.reset_pin = (gpio_t) SX1276_RESET;

    //settings.channel = RF_FREQUENCY;
    settings.modem = SX1276_MODEM_LORA;
    settings.state = SX1276_RF_IDLE;
    settings.lora = settings_lora;

    sx1276.settings = settings;
    sx1276.sx1276_event_cb = event_handler_thread;

    /* Launch initialization of driver and device */
    puts("init_radio: initializing driver...");

    puts("init_radio: sx1276 initialization done");
}

/**
 * Main application entry point.
 */
int main( void )
{

#if( OVER_THE_AIR_ACTIVATION != 0 )
    uint8_t sendFrameStatus = 0;
#endif
    bool trySendingFrameAgain = false;

    radio_set_ptr(&sx1276);
    xtimer_init();
    init_radio();
    
    /* initialize all available ADC lines */
    adc_init(ADC_VDIV);
    adc_init(ADC_LINE(0));
    adc_init(ADC_VREF);

    gpio_init(USB_DETECT, GPIO_IN);
    gpio_init(BAT_LEVEL, GPIO_OUT);
    gpio_clear(BAT_LEVEL);

    if(get_board_power_source() == BATTERY_POWER)
        puts("BATTERY_POWER");
    else
        puts("USB POWER");

    LoRaMacCallbacks.MacEvent = OnMacEvent;
    LoRaMacCallbacks.GetBatteryLevel = board_get_battery_level;
    LoRaMacInit( &LoRaMacCallbacks );

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
    TimerInit( &JoinReqTimer, OnJoinReqTimerEvent );
    TimerSetValue( &JoinReqTimer, OVER_THE_AIR_ACTIVATION_DUTYCYCLE );
#endif

    TxNextPacket = true;
    TimerInit( &TxNextPacketTimer, OnTxNextPacketTimerEvent );

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
                    TimerStart( &JoinReqTimer );
                    break;
                }
            }
            TimerLowPowerHandler(OVER_THE_AIR_ACTIVATION_DUTYCYCLE);
#endif
        }
        
        if( DownlinkStatusUpdate == true )
        {
            DownlinkStatusUpdate = false;
            // Switch LED 2 ON for each received downlink
            puts("DOWNLINK");
        }

        if( ScheduleNextTx == true )
        {
            ScheduleNextTx = false;

            if( ComplianceTest.Running == true )
            {
                TxNextPacket = true;
            }
            else
            {
                // Schedule next packet transmission
                TxDutyCycleTime = APP_TX_DUTYCYCLE + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
                TxNextPacketTimerInit = xtimer_now_usec64();
                TimerSetValue( &TxNextPacketTimer, TxDutyCycleTime );
                TimerStart( &TxNextPacketTimer );
            }
        }

        if( trySendingFrameAgain == true )
        {
            trySendingFrameAgain = SendFrame( );
        }
        if( TxNextPacket == true )
        {
            TxNextPacket = false;
            LED0_TOGGLE;

            PrepareTxFrame( AppPort );

            trySendingFrameAgain = SendFrame( );
        }

        TxNextPacketTimerActual = xtimer_now_usec64() - TxNextPacketTimerInit ;
        TimerLowPowerHandler( (uint32_t) TxDutyCycleTime);
    }

    return 0;
}