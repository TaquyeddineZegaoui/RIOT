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

#include "LoRaMacCrypto.h"
#include "LoRaMac.h"
#include "LoRaMacTest.h"
#include "thread.h"
#include "random.h"
#include <string.h>


/*!
 * \brief Returns the minimum value betwen a and b
 *
 * \param [IN] a 1st value
 * \param [IN] b 2nd value
 * \retval minValue Minimum value
 */
#define MIN( a, b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )

/*!
 * \brief Returns the maximum value betwen a and b
 *
 * \param [IN] a 1st value
 * \param [IN] b 2nd value
 * \retval maxValue Maximum value
 */
#define MAX( a, b ) ( ( ( a ) > ( b ) ) ? ( a ) : ( b ) )

/*!
 * \brief Returns 2 raised to the power of n
 *
 * \param [IN] n power value
 * \retval result of raising 2 to the power n
 */
#define POW2( n ) ( 1 << n )

/*!
 * Radio wakeup time from SLEEP mode
 */
#define RADIO_OSC_STARTUP                           1 // [ms]

/*!
 * Radio PLL lock and Mode Ready delay which can vary with the temperature
 */
#define RADIO_SLEEP_TO_RX                           2 // [ms]

/*!
 * Radio complete Wake-up Time with margin for temperature compensation
 */
#define RADIO_WAKEUP_TIME ( RADIO_OSC_STARTUP + RADIO_SLEEP_TO_RX )


/*!
 * FRMPayload overhead to be used when setting the Radio.SetMaxPayloadLength
 * in RxWindowSetup function.
 * Maximum PHYPayload = MaxPayloadOfDatarate/MaxPayloadOfDatarateRepeater + LORA_MAC_FRMPAYLOAD_OVERHEAD
 */
#define LORA_MAC_FRMPAYLOAD_OVERHEAD                13 // MHDR(1) + FHDR(7) + Port(1) + MIC(4)

/*!
 * Device IEEE EUI
 */
/*!
 * Radio driver supported modems
 */
typedef enum
{
    MODEM_FSK = 0,
    MODEM_LORA,
}RadioModems_t;

/*!
 * Radio driver internal state machine states definition
 */
typedef enum
{
    RF_IDLE = 0,
    RF_RX_RUNNING,
    RF_TX_RUNNING,
    RF_CAD,
}RadioState_t;

static netdev2_lorawan_t *dev;

netdev2_lorawan_t *get_dev_ptr(void)
{
    return dev;
}

#if defined( USE_BAND_433 )
/*!
 * Data rates table definition
 */
const uint8_t Datarates[]  = { 12, 11, 10,  9,  8,  7,  7, 50 };

/*!
 * Maximum payload with respect to the datarate index. Cannot operate with repeater.
 */
const uint8_t MaxPayloadOfDatarate[] = { 59, 59, 59, 123, 250, 250, 250, 250 };

/*!
 * Maximum payload with respect to the datarate index. Can operate with repeater.
 */
const uint8_t MaxPayloadOfDatarateRepeater[] = { 59, 59, 59, 123, 230, 230, 230, 230 };

/*!
 * Tx output powers table definition
 */
const int8_t TxPowers[]    = { 20, 14, 11,  8,  5,  2 };

/*!
 * LoRaMac bands
 */
static Band_t Bands[LORA_MAX_NB_BANDS] =
{
    BAND0,
};

/*!
 * LoRaMAC channels
 */
static ChannelParams_t Channels[LORA_MAX_NB_CHANNELS] =
{
    LC1,
    LC2,
    LC3,
};
#elif defined( USE_BAND_780 )
/*!
 * Data rates table definition
 */
const uint8_t Datarates[]  = { 12, 11, 10,  9,  8,  7,  7, 50 };

/*!
 * Maximum payload with respect to the datarate index. Cannot operate with repeater.
 */
const uint8_t MaxPayloadOfDatarate[] = { 59, 59, 59, 123, 250, 250, 250, 250 };

/*!
 * Maximum payload with respect to the datarate index. Can operate with repeater.
 */
const uint8_t MaxPayloadOfDatarateRepeater[] = { 59, 59, 59, 123, 230, 230, 230, 230 };

/*!
 * Tx output powers table definition
 */
const int8_t TxPowers[]    = { 20, 14, 11,  8,  5,  2 };

/*!
 * LoRaMac bands
 */
static Band_t Bands[LORA_MAX_NB_BANDS] =
{
    BAND0,
};

/*!
 * LoRaMAC channels
 */
static ChannelParams_t Channels[LORA_MAX_NB_CHANNELS] =
{
    LC1,
    LC2,
    LC3,
};
#elif defined( USE_BAND_868 )
/*!
 * Data rates table definition
 */
const uint8_t Datarates[]  = { 12, 11, 10,  9,  8,  7,  7, 50 };

/*!
 * Maximum payload with respect to the datarate index. Cannot operate with repeater.
 */
const uint8_t MaxPayloadOfDatarate[] = { 51, 51, 51, 115, 242, 242, 242, 242 };

/*!
 * Maximum payload with respect to the datarate index. Can operate with repeater.
 */
const uint8_t MaxPayloadOfDatarateRepeater[] = { 51, 51, 51, 115, 222, 222, 222, 222 };

/*!
 * Tx output powers table definition
 */
const int8_t TxPowers[]    = { 20, 14, 11,  8,  5,  2 };

/*!
 * LoRaMac bands
 */
static Band_t Bands[LORA_MAX_NB_BANDS] =
{
    BAND0,
    BAND1,
    BAND2,
    BAND3,
    BAND4,
};

/*!
 * LoRaMAC channels
 */
static ChannelParams_t Channels[LORA_MAX_NB_CHANNELS] =
{
    LC1,
    LC2,
    LC3,
};
#elif defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )
/*!
 * Data rates table definition
 */
const uint8_t Datarates[]  = { 10, 9, 8,  7,  8,  0,  0, 0, 12, 11, 10, 9, 8, 7, 0, 0 };

/*!
 * Up/Down link data rates offset definition
 */
const int8_t datarateOffsets[16][4] =
{
    { DR_10, DR_9 , DR_8 , DR_8  }, // DR_0
    { DR_11, DR_10, DR_9 , DR_8  }, // DR_1
    { DR_12, DR_11, DR_10, DR_9  }, // DR_2
    { DR_13, DR_12, DR_11, DR_10 }, // DR_3
    { DR_13, DR_13, DR_12, DR_11 }, // DR_4
};

/*!
 * Maximum payload with respect to the datarate index. Cannot operate with repeater.
 */
const uint8_t MaxPayloadOfDatarate[] = { 11, 53, 125, 242, 242, 0, 0, 0, 53, 129, 242, 242, 242, 242, 0, 0 };

/*!
 * Maximum payload with respect to the datarate index. Can operate with repeater.
 */
const uint8_t MaxPayloadOfDatarateRepeater[] = { 11, 53, 125, 242, 242, 0, 0, 0, 33, 109, 222, 222, 222, 222, 0, 0 };

/*!
 * Tx output powers table definition
 */
const int8_t TxPowers[]    = { 30, 28, 26, 24, 22, 20, 18, 16, 14, 12, 10 };

/*!
 * LoRaMac bands
 */
static Band_t Bands[LORA_MAX_NB_BANDS] =
{
    BAND0,
};

/*!
 *  * LoRaMAC channels
 *   */
static ChannelParams_t Channels[LORA_MAX_NB_CHANNELS];

/*!
 * Defines the first channel for RX window 2 for US band
 */
#define LORAMAC_FIRST_RX2_CHANNEL           ( (uint32_t) 923.3e6 )

/*!
 * Defines the last channel for RX window 2 for US band
 */
#define LORAMAC_LAST_RX2_CHANNEL            ( (uint32_t) 927.5e6 )

/*!
 * Defines the step width of the channels for RX window 2
 */
#define LORAMAC_STEPWIDTH_RX2_CHANNEL       ( (uint32_t) 600e3 )

#else
    #error "Please define a frequency band in the compiler options."
#endif

/*!
 * LoRaMac internal states
 */
enum eLoRaMacState
{
    MAC_IDLE          = 0x00000000,
    MAC_TX_RUNNING    = 0x00000001,
    MAC_RX            = 0x00000002,
    MAC_ACK_REQ       = 0x00000004,
    MAC_ACK_RETRY     = 0x00000008,
    MAC_TX_DELAYED    = 0x00000010,
    MAC_TX_CONFIG     = 0x00000020,
    MAC_RX_ABORT      = 0x00000040,
};


/*!
 * \brief This function prepares the MAC to abort the execution of function
 *        OnRadioRxDone in case of a reception error.
 */
static void PrepareRxDoneAbort(netdev2_t *netdev);

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRadioRxDone(netdev2_t *netdev, uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );


/*!
 * \brief Searches and set the next random available channel
 *
 * \param [OUT] Time to wait for the next transmission according to the duty
 *              cycle.
 *
 * \retval status  Function status [1: OK, 0: Unable to find a channel on the
 *                                  current datarate]
 */
static bool SetNextChannel( TimerTime_t* time );

/*!
 * \brief Sets the network to public or private. Updates the sync byte.
 *
 * \param [IN] enable if true, it enables a public network
 */
static void SetPublicNetwork( bool enable );

/*!
 * \brief Initializes and opens the reception window
 *
 * \param [IN] freq window channel frequency
 * \param [IN] datarate window channel datarate
 * \param [IN] bandwidth window channel bandwidth
 * \param [IN] timeout window channel timeout
 */
static void RxWindowSetup( uint32_t freq, int8_t datarate, uint32_t bandwidth, uint16_t timeout, bool rxContinuous );

/*!
 * \brief Verifies if the RX window 2 frequency is in range
 *
 * \param [IN] freq window channel frequency
 *
 * \retval status  Function status [1: OK, 0: Frequency not applicable]
 */
static bool Rx2FreqInRange( uint32_t freq );

/*!
 * \brief Adds a new MAC command to be sent.
 *
 * \Remark MAC layer internal function
 *
 * \param [in] cmd MAC command to be added
 *                 [MOTE_MAC_LINK_CHECK_REQ,
 *                  MOTE_MAC_LINK_ADR_ANS,
 *                  MOTE_MAC_DUTY_CYCLE_ANS,
 *                  MOTE_MAC_RX2_PARAM_SET_ANS,
 *                  MOTE_MAC_DEV_STATUS_ANS
 *                  MOTE_MAC_NEW_CHANNEL_ANS]
 * \param [in] p1  1st parameter ( optional depends on the command )
 * \param [in] p2  2nd parameter ( optional depends on the command )
 *
 * \retval status  Function status [0: OK, 1: Unknown command, 2: Buffer full]
 */
static LoRaMacStatus_t AddMacCommand( uint8_t cmd, uint8_t p1, uint8_t p2 );

/*!
 * \brief Parses the MAC commands which must be repeated.
 *
 * \Remark MAC layer internal function
 *
 * \param [IN] cmdBufIn  Buffer which stores the MAC commands to send
 * \param [IN] length  Length of the input buffer to parse
 * \param [OUT] cmdBufOut  Buffer which stores the MAC commands which must be
 *                         repeated.
 *
 * \retval Size of the MAC commands to repeat.
 */
static uint8_t ParseMacCommandsToRepeat( uint8_t* cmdBufIn, uint8_t length, uint8_t* cmdBufOut );

/*!
 * \brief Validates if the payload fits into the frame, taking the datarate
 *        into account.
 *
 * \details Refer to chapter 4.3.2 of the LoRaWAN specification, v1.0
 *
 * \param lenN Length of the application payload. The length depends on the
 *             datarate and is region specific
 *
 * \param datarate Current datarate
 *
 * \param fOptsLen Length of the fOpts field
 *
 * \retval [false: payload does not fit into the frame, true: payload fits into
 *          the frame]
 */
static bool ValidatePayloadLength( uint8_t lenN, int8_t datarate, uint8_t fOptsLen );

/*!
 * \brief Counts the number of bits in a mask.
 *
 * \param [IN] mask A mask from which the function counts the active bits.
 * \param [IN] nbBits The number of bits to check.
 *
 * \retval Number of enabled bits in the mask.
 */
static uint8_t CountBits( uint16_t mask, uint8_t nbBits );

#if defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )
/*!
 * \brief Counts the number of enabled 125 kHz channels in the channel mask.
 *        This function can only be applied to US915 band.
 *
 * \param [IN] channelsMask Pointer to the first element of the channel mask
 *
 * \retval Number of enabled channels in the channel mask
 */
static uint8_t CountNbEnabled125kHzChannels( uint16_t *channelsMask );

#if defined( USE_BAND_915_HYBRID )
/*!
 * \brief Validates the correctness of the channel mask for US915, hybrid mode.
 *
 * \param [IN] mask Block definition to set.
 * \param [OUT] channelsMask Pointer to the first element of the channel mask
 */
static void ReenableChannels( uint16_t mask, uint16_t* channelMask );

/*!
 * \brief Validates the correctness of the channel mask for US915, hybrid mode.
 *
 * \param [IN] channelsMask Pointer to the first element of the channel mask
 *
 * \retval [true: channel mask correct, false: channel mask not correct]
 */
static bool ValidateChannelMask( uint16_t* channelMask );
#endif

#endif

/*!
 * \brief Limits the Tx power according to the number of enabled channels
 *
 * \retval Returns the maximum valid tx power
 */
static int8_t LimitTxPower( int8_t txPower );

/*!
 * \brief Verifies, if a value is in a given range.
 *
 * \param value Value to verify, if it is in range
 *
 * \param min Minimum possible value
 *
 * \param max Maximum possible value
 *
 * \retval Returns the maximum valid tx power
 */
static bool ValueInRange( int8_t value, int8_t min, int8_t max );

/*!
 * \brief Calculates the next datarate to set, when ADR is on or off
 *
 * \param [IN] adrEnabled Specify whether ADR is on or off
 *
 * \param [IN] updateChannelMask Set to true, if the channel masks shall be updated
 *
 * \param [OUT] datarateOut Reports the datarate which will be used next
 *
 * \retval Returns the state of ADR ack request
 */
static bool AdrNextDr( bool adrEnabled, bool updateChannelMask, int8_t* datarateOut );

/*!
 * \brief Disables channel in a specified channel mask
 *
 * \param [IN] id - Id of the channel
 *
 * \param [IN] mask - Pointer to the channel mask to edit
 *
 * \retval [true, if disable was successful, false if not]
 */
static bool DisableChannelInMask( uint8_t id, uint16_t* mask );

/*!
 * \brief Decodes MAC commands in the fOpts field and in the payload
 */
static void ProcessMacCommands( uint8_t *payload, uint8_t macIndex, uint8_t commandsSize, uint8_t snr );

/*!
 * \brief LoRaMAC layer generic send frame
 *
 * \param [IN] macHdr      MAC header field
 * \param [IN] fPort       MAC payload port
 * \param [IN] fBuffer     MAC data buffer to be sent
 * \param [IN] fBufferSize MAC data buffer size
 * \retval status          Status of the operation.
 */
LoRaMacStatus_t Send( LoRaMacHeader_t *macHdr, uint8_t fPort, void *fBuffer, uint16_t fBufferSize );

/*!
 * \brief LoRaMAC layer frame buffer initialization
 *
 * \param [IN] macHdr      MAC header field
 * \param [IN] fCtrl       MAC frame control field
 * \param [IN] fOpts       MAC commands buffer
 * \param [IN] fPort       MAC payload port
 * \param [IN] fBuffer     MAC data buffer to be sent
 * \param [IN] fBufferSize MAC data buffer size
 * \retval status          Status of the operation.
 */
LoRaMacStatus_t PrepareFrame( LoRaMacHeader_t *macHdr, LoRaMacFrameCtrl_t *fCtrl, uint8_t fPort, void *fBuffer, uint16_t fBufferSize );

/*
 * \brief Schedules the frame according to the duty cycle
 *
 * \retval Status of the operation
 */
static LoRaMacStatus_t ScheduleTx( void );

/*
 * \brief Sets the duty cycle for retransmissions
 *
 * \retval Duty cycle
 */
static uint16_t RetransmissionDutyCylce( void );

/*
 * \brief Calculates the back-off time for the band of a channel.
 *
 * \param [IN] channel     The last Tx channel index
 */
static void CalculateBackOff( uint8_t channel );

/*
 * \brief Alternates the datarate of the channel for the join request.
 *
 * \param [IN] nbTrials    Number of performed join requests.
 * \retval Datarate to apply
 */
static int8_t AlternateDatarate( uint16_t nbTrials );

/*!
 * \brief LoRaMAC layer prepared frame buffer transmission with channel specification
 *
 * \remark PrepareFrame must be called at least once before calling this
 *         function.
 *
 * \param [IN] channel     Channel parameters
 * \retval status          Status of the operation.
 */
LoRaMacStatus_t SendFrameOnChannel( ChannelParams_t channel );
bool check_rf_freq( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
    return true;
}

/*!
 * \brief Resets MAC specific parameters to default
 */
void memcpyr( uint8_t *dst, const uint8_t *src, uint16_t size )
{
   dst = dst + ( size - 1 );
   while( size-- )
   {
       *dst-- = *src++;
   }
}
static void ResetMacParameters( void );

void OnRadioTxDone(netdev2_t *netdev)
{
    TimerTime_t curTime = xtimer_now_usec64();
    netopt_state_t state = NETOPT_STATE_SLEEP;
    if( dev->LoRaMacDeviceClass != CLASS_C )
    {
        netdev->driver->set(netdev, NETOPT_STATE, &state, sizeof(netopt_state_t));
    }
    else
    {
        OnRxWindow2TimerEvent(netdev);
    }

    // Store last Tx channel
    dev->LastTxChannel = dev->Channel;
    // Update last tx done time for the current channel
    Bands[Channels[dev->LastTxChannel].Band].LastTxDoneTime = curTime;
    // Update Aggregated last tx done time
    dev->AggregatedLastTxDoneTime = curTime;

    if( dev->IsRxWindowsEnabled == true )
    {
        dev->RxWindowTimer1.msg.type = LORAWAN_TIMER_RX_WINDOW1;
        xtimer_set_msg(&(dev->RxWindowTimer1.dev), xtimer_ticks_from_usec(dev->RxWindow1Delay*1000-4000).ticks32, &(dev->RxWindowTimer1.msg), dev->RxWindowTimer1.pid);
        //TimerSetValue( &RxWindowTimer1, dev->RxWindow1Delay, LORAWAN_TIMER_RX_WINDOW1);
        //TimerStart( &RxWindowTimer1, 0 );
        if( dev->LoRaMacDeviceClass != CLASS_C )
        {
            dev->RxWindowTimer2.msg.type = LORAWAN_TIMER_RX_WINDOW2;
        xtimer_set_msg(&(dev->RxWindowTimer2.dev), xtimer_ticks_from_usec(dev->RxWindow2Delay*1000).ticks32, &(dev->RxWindowTimer2.msg), dev->RxWindowTimer2.pid);
            //TimerSetValue( &RxWindowTimer2, dev->RxWindow2Delay, LORAWAN_TIMER_RX_WINDOW2);
            //TimerStart( &RxWindowTimer2, 0);
        }
        if( ( dev->LoRaMacDeviceClass == CLASS_C ) || ( dev->NodeAckRequested == true ) )
        {
            dev->AckTimeoutTimer.msg.type = LORAWAN_TIMER_ACK_TIMEOUT;
            xtimer_set_msg(&(dev->AckTimeoutTimer.dev), (dev->RxWindow2Delay + ACK_TIMEOUT + random_uint32_range( -ACK_TIMEOUT_RND, ACK_TIMEOUT_RND))*1000, &(dev->AckTimeoutTimer.msg), dev->AckTimeoutTimer.pid);
            //TimerSetValue( &AckTimeoutTimer, dev->RxWindow2Delay + ACK_TIMEOUT +
             //                                randr( -ACK_TIMEOUT_RND, ACK_TIMEOUT_RND ), LORAWAN_TIMER_ACK_TIMEOUT);
            //TimerStart( &AckTimeoutTimer, 0);
        }
    }
    else
    {
        dev->frame_status = LORAMAC_EVENT_INFO_STATUS_OK;
//TODO: Below there's an MlmeConfirm event. Fix this.
        //dev->frame_status = LORAMAC_EVENT_INFO_STATUS_RX2_TIMEOUT;

        if( dev->LoRaMacFlags.Value == 0 )
        {
           dev->LoRaMacFlags.Bits.McpsReq = 1;
        }
        dev->LoRaMacFlags.Bits.MacDone = 1;
    }

    if( dev->NodeAckRequested == false )
    {
        dev->frame_status = LORAMAC_EVENT_INFO_STATUS_OK;
        dev->ChannelsNbRepCounter++;
    }
}

static void PrepareRxDoneAbort(netdev2_t *netdev)
{
    dev->LoRaMacState |= MAC_RX_ABORT;

    if( dev->NodeAckRequested )
    {
        OnAckTimeoutTimerEvent(netdev);
    }

    if( ( dev->RxSlot == 0 ) && ( dev->LoRaMacDeviceClass == CLASS_C ) )
    {
        OnRxWindow2TimerEvent(netdev);
    }

    dev->LoRaMacFlags.Bits.McpsInd = 1;
    dev->LoRaMacFlags.Bits.MacDone = 1;

    // Trig OnMacCheckTimerEvent call as soon as possible
    dev->MacStateCheckTimer.msg.type = LORAWAN_TIMER_MAC_STATE;
    xtimer_set_msg(&(dev->MacStateCheckTimer.dev), xtimer_ticks_from_usec(1000).ticks32, &(dev->MacStateCheckTimer.msg), dev->MacStateCheckTimer.pid);
    //TimerSetValue( &MacStateCheckTimer, 1 , LORAWAN_TIMER_MAC_STATE);
    //TimerStart( &MacStateCheckTimer, 0);
}

void lorawan_set_pointer(netdev2_lorawan_t* netdev)
{
    dev = netdev;
}

void OnRadioRxDone(netdev2_t *netdev, uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    LoRaMacHeader_t macHdr;
    LoRaMacFrameCtrl_t fCtrl;
    bool skipIndication = false;

    uint8_t pktHeaderLen = 0;
    uint32_t address = 0;
    uint8_t appPayloadStartIndex = 0;
    uint8_t port = 0xFF;
    uint8_t frameLen = 0;
    uint32_t mic = 0;
    uint32_t micRx = 0;

    uint16_t sequenceCounter = 0;
    uint16_t sequenceCounterPrev = 0;
    uint16_t sequenceCounterDiff = 0;
    uint32_t downLinkCounter = 0;

    MulticastParams_t *curMulticastParams = NULL;

    uint8_t multicast = 0;

    bool isMicOk = false;

    netopt_state_t state = NETOPT_STATE_SLEEP;
    dev->ack_received = false;
    dev->McpsIndication.Rssi = rssi;
    dev->McpsIndication.Snr = snr;
    dev->McpsIndication.RxSlot = dev->RxSlot;
    dev->McpsIndication.Port = 0;
    dev->McpsIndication.Multicast = 0;
    dev->McpsIndication.FramePending = 0;
    dev->McpsIndication.Buffer = NULL;
    dev->McpsIndication.BufferSize = 0;
    dev->McpsIndication.RxData = false;
    dev->McpsIndication.DownLinkCounter = 0;
    dev->McpsIndication.McpsIndication = MCPS_UNCONFIRMED;

    if( dev->LoRaMacDeviceClass != CLASS_C )
    {
        netdev2_t *netdev = (netdev2_t*) dev;
        netdev->driver->set(netdev, NETOPT_STATE, &state, sizeof(netopt_state_t));
    }
    //TimerStop( &RxWindowTimer2 );
    xtimer_remove(&dev->RxWindowTimer2.dev);

    macHdr.Value = payload[pktHeaderLen++];

    switch( macHdr.Bits.MType )
    {
        case FRAME_TYPE_JOIN_ACCEPT:
            if( dev->lorawan.tx_rx.nwk_status == true )
            {
                break;
            }
            LoRaMacJoinDecrypt( payload + 1, size - 1, dev->app_key, dev->LoRaMacRxPayload + 1 );

            dev->LoRaMacRxPayload[0] = macHdr.Value;

            LoRaMacJoinComputeMic( dev->LoRaMacRxPayload, size - LORAMAC_MFR_LEN, dev->app_key, &mic );

            micRx |= ( uint32_t )dev->LoRaMacRxPayload[size - LORAMAC_MFR_LEN];
            micRx |= ( ( uint32_t )dev->LoRaMacRxPayload[size - LORAMAC_MFR_LEN + 1] << 8 );
            micRx |= ( ( uint32_t )dev->LoRaMacRxPayload[size - LORAMAC_MFR_LEN + 2] << 16 );
            micRx |= ( ( uint32_t )dev->LoRaMacRxPayload[size - LORAMAC_MFR_LEN + 3] << 24 );

            if( micRx == mic )
            {
                LoRaMacJoinComputeSKeys( dev->app_key, dev->LoRaMacRxPayload + 1, dev->dev_nonce, dev->lorawan.nwk_skey, dev->lorawan.nwk_skey );

                dev->lorawan.net_id = ( uint32_t )dev->LoRaMacRxPayload[4];
                dev->lorawan.net_id |= ( ( uint32_t )dev->LoRaMacRxPayload[5] << 8 );
                dev->lorawan.net_id |= ( ( uint32_t )dev->LoRaMacRxPayload[6] << 16 );

                dev->lorawan.dev_addr = ( uint32_t )dev->LoRaMacRxPayload[7];
                dev->lorawan.dev_addr |= ( ( uint32_t )dev->LoRaMacRxPayload[8] << 8 );
                dev->lorawan.dev_addr |= ( ( uint32_t )dev->LoRaMacRxPayload[9] << 16 );
                dev->lorawan.dev_addr |= ( ( uint32_t )dev->LoRaMacRxPayload[10] << 24 );

                // DLSettings
                dev->LoRaMacParams.Rx1DrOffset = ( dev->LoRaMacRxPayload[11] >> 4 ) & 0x07;
                dev->LoRaMacParams.Rx2Channel.Datarate = dev->LoRaMacRxPayload[11] & 0x0F;
#if defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )
                /*
                 * WARNING: To be removed once Semtech server implementation
                 *          is corrected.
                 */
                if( dev->LoRaMacParams.Rx2Channel.Datarate == DR_3 )
                {
                    dev->LoRaMacParams.Rx2Channel.Datarate = DR_8;
                }
#endif
                // RxDelay
                dev->LoRaMacParams.ReceiveDelay1 = ( dev->LoRaMacRxPayload[12] & 0x0F );
                if( dev->LoRaMacParams.ReceiveDelay1 == 0 )
                {
                    dev->LoRaMacParams.ReceiveDelay1 = 1;
                }
                dev->LoRaMacParams.ReceiveDelay1 *= 1e3;
                dev->LoRaMacParams.ReceiveDelay2 = dev->LoRaMacParams.ReceiveDelay1 + 1e3;

#if !( defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID ) )
                //CFList
                if( ( size - 1 ) > 16 )
                {
                    ChannelParams_t param;
                    param.DrRange.Value = ( DR_5 << 4 ) | DR_0;

                    LoRaMacState |= MAC_TX_CONFIG;
                    for( uint8_t i = 3, j = 0; i < ( 5 + 3 ); i++, j += 3 )
                    {
                        param.Frequency = ( ( uint32_t )LoRaMacRxPayload[13 + j] | ( ( uint32_t )LoRaMacRxPayload[14 + j] << 8 ) | ( ( uint32_t )LoRaMacRxPayload[15 + j] << 16 ) ) * 100;
                        LoRaMacChannelAdd( i, param );
                    }
                    LoRaMacState &= ~MAC_TX_CONFIG;
                }
#endif
                dev->frame_status = LORAMAC_EVENT_INFO_STATUS_OK;
                dev->lorawan.tx_rx.nwk_status = true;
                dev->LoRaMacParams.ChannelsDatarate = dev->LoRaMacParamsDefaults.ChannelsDatarate;
            }
            else
            {
                dev->frame_status = LORAMAC_EVENT_INFO_STATUS_JOIN_FAIL;
            }
            break;
        case FRAME_TYPE_DATA_CONFIRMED_DOWN:
        case FRAME_TYPE_DATA_UNCONFIRMED_DOWN:
            {
                address = payload[pktHeaderLen++];
                address |= ( (uint32_t)payload[pktHeaderLen++] << 8 );
                address |= ( (uint32_t)payload[pktHeaderLen++] << 16 );
                address |= ( (uint32_t)payload[pktHeaderLen++] << 24 );

                if( address != dev->lorawan.dev_addr )
                {
                    curMulticastParams = dev->MulticastChannels;
                    while( curMulticastParams != NULL )
                    {
                        if( address == curMulticastParams->Address )
                        {
                            multicast = 1;
                            downLinkCounter = curMulticastParams->DownLinkCounter;
                            break;
                        }
                        curMulticastParams = curMulticastParams->Next;
                    }
                    if( multicast == 0 )
                    {
                        // We are not the destination of this frame.
                        dev->McpsIndication.Status = LORAMAC_EVENT_INFO_STATUS_ADDRESS_FAIL;
                        PrepareRxDoneAbort(netdev);
                        return;
                    }
                }
                else
                {
                    multicast = 0;
                    downLinkCounter = dev->DownLinkCounter;
                }

                fCtrl.Value = payload[pktHeaderLen++];

                sequenceCounter = ( uint16_t )payload[pktHeaderLen++];
                sequenceCounter |= ( uint16_t )payload[pktHeaderLen++] << 8;

                appPayloadStartIndex = 8 + fCtrl.Bits.FOptsLen;

                micRx |= ( uint32_t )payload[size - LORAMAC_MFR_LEN];
                micRx |= ( ( uint32_t )payload[size - LORAMAC_MFR_LEN + 1] << 8 );
                micRx |= ( ( uint32_t )payload[size - LORAMAC_MFR_LEN + 2] << 16 );
                micRx |= ( ( uint32_t )payload[size - LORAMAC_MFR_LEN + 3] << 24 );

                sequenceCounterPrev = ( uint16_t )downLinkCounter;
                sequenceCounterDiff = ( sequenceCounter - sequenceCounterPrev );

                if( sequenceCounterDiff < ( 1 << 15 ) )
                {
                    downLinkCounter += sequenceCounterDiff;
                    LoRaMacComputeMic( payload, size - LORAMAC_MFR_LEN, dev->lorawan.nwk_skey, address, DOWN_LINK, downLinkCounter, &mic );
                    if( micRx == mic )
                    {
                        isMicOk = true;
                    }
                }
                else
                {
                    // check for sequence roll-over
                    uint32_t  downLinkCounterTmp = downLinkCounter + 0x10000 + ( int16_t )sequenceCounterDiff;
                    LoRaMacComputeMic( payload, size - LORAMAC_MFR_LEN, dev->lorawan.nwk_skey, address, DOWN_LINK, downLinkCounterTmp, &mic );
                    if( micRx == mic )
                    {
                        isMicOk = true;
                        downLinkCounter = downLinkCounterTmp;
                    }
                }

                // Check for a the maximum allowed counter difference
                if( sequenceCounterDiff >= MAX_FCNT_GAP )
                {
                    dev->McpsIndication.Status = LORAMAC_EVENT_INFO_STATUS_DOWNLINK_TOO_MANY_FRAMES_LOSS;
                    dev->McpsIndication.DownLinkCounter = downLinkCounter;
                    PrepareRxDoneAbort(netdev);
                    return;
                }

                if( isMicOk == true )
                {
                    dev->McpsIndication.Status = LORAMAC_EVENT_INFO_STATUS_OK;
                    dev->McpsIndication.Multicast = multicast;
                    dev->McpsIndication.FramePending = fCtrl.Bits.FPending;
                    dev->McpsIndication.Buffer = NULL;
                    dev->McpsIndication.BufferSize = 0;
                    dev->McpsIndication.DownLinkCounter = downLinkCounter;

                    dev->frame_status = LORAMAC_EVENT_INFO_STATUS_OK;

                    dev->AdrAckCounter = 0;
                    dev->MacCommandsBufferToRepeatIndex = 0;

                    // Update 32 bits downlink counter
                    if( multicast == 1 )
                    {
                        dev->McpsIndication.McpsIndication = MCPS_MULTICAST;

                        if( ( curMulticastParams->DownLinkCounter == downLinkCounter ) &&
                            ( curMulticastParams->DownLinkCounter != 0 ) )
                        {
                            dev->McpsIndication.Status = LORAMAC_EVENT_INFO_STATUS_DOWNLINK_REPEATED;
                            dev->McpsIndication.DownLinkCounter = downLinkCounter;
                            PrepareRxDoneAbort(netdev);
                            return;
                        }
                        curMulticastParams->DownLinkCounter = downLinkCounter;
                    }
                    else
                    {
                        if( macHdr.Bits.MType == FRAME_TYPE_DATA_CONFIRMED_DOWN )
                        {
                            dev->SrvAckRequested = true;
                            dev->McpsIndication.McpsIndication = MCPS_CONFIRMED;

                            if( ( dev->DownLinkCounter == downLinkCounter ) &&
                                ( dev->DownLinkCounter != 0 ) )
                            {
                                // Duplicated confirmed downlink. Skip indication.
                                skipIndication = true;
                            }
                        }
                        else
                        {
                            dev->SrvAckRequested = false;
                            dev->McpsIndication.McpsIndication = MCPS_UNCONFIRMED;

                            if( ( dev->DownLinkCounter == downLinkCounter ) &&
                                ( dev->DownLinkCounter != 0 ) )
                            {
                                dev->McpsIndication.Status = LORAMAC_EVENT_INFO_STATUS_DOWNLINK_REPEATED;
                                dev->McpsIndication.DownLinkCounter = downLinkCounter;
                                PrepareRxDoneAbort(netdev);
                                return;
                            }
                        }
                        dev->DownLinkCounter = downLinkCounter;
                    }

                    // Check if the frame is an acknowledgement
                    if( fCtrl.Bits.Ack == 1 )
                    {
                        dev->ack_received = true;

                        // Stop the AckTimeout timer as no more retransmissions
                        // are needed.
                        //TimerStop( &AckTimeoutTimer );
                        xtimer_remove(&dev->AckTimeoutTimer.dev);
                    }
                    else
                    {
                        dev->ack_received = false;

                        if( dev->AckTimeoutRetriesCounter > dev->AckTimeoutRetries )
                        {
                            // Stop the AckTimeout timer as no more retransmissions
                            // are needed.
                            //TimerStop( &AckTimeoutTimer );
                            xtimer_remove(&dev->AckTimeoutTimer.dev);
                        }
                    }

                    if( ( ( size - 4 ) - appPayloadStartIndex ) > 0 )
                    {
                        port = payload[appPayloadStartIndex++];
                        frameLen = ( size - 4 ) - appPayloadStartIndex;

                        dev->McpsIndication.Port = port;

                        if( port == 0 )
                        {
                            if( fCtrl.Bits.FOptsLen == 0 )
                            {
                                 LoRaMacPayloadDecrypt( payload + appPayloadStartIndex,
                                                       frameLen,
                                                       dev->lorawan.nwk_skey,
                                                       address,
                                                       DOWN_LINK,
                                                       downLinkCounter,
                                                       dev->LoRaMacRxPayload );

                                // Decode frame payload MAC commands
                                ProcessMacCommands( dev->LoRaMacRxPayload, 0, frameLen, snr );
                            }
                            else
                            {
                                skipIndication = true;
                            }
                        }
                        else
                        {
                            if( fCtrl.Bits.FOptsLen > 0 )
                            {
                                // Decode Options field MAC commands. Omit the fPort.
                                ProcessMacCommands( payload, 8, appPayloadStartIndex - 1, snr );
                            }

                             LoRaMacPayloadDecrypt( payload + appPayloadStartIndex,
                                                   frameLen,
                                                   dev->lorawan.app_skey,
                                                   address,
                                                   DOWN_LINK,
                                                   downLinkCounter,
                                                   dev->LoRaMacRxPayload );

                            if( skipIndication == false )
                            {
                                dev->McpsIndication.Buffer = dev->LoRaMacRxPayload;
                                dev->McpsIndication.BufferSize = frameLen;
                                dev->McpsIndication.RxData = true;
                            }
                        }
                    }
                    else
                    {
                        if( fCtrl.Bits.FOptsLen > 0 )
                        {
                            // Decode Options field MAC commands
                            ProcessMacCommands( payload, 8, appPayloadStartIndex, snr );
                        }
                    }

                    if( skipIndication == false )
                    {
                        dev->LoRaMacFlags.Bits.McpsInd = 1;
                    }
                }
                else
                {
                    dev->McpsIndication.Status = LORAMAC_EVENT_INFO_STATUS_MIC_FAIL;

                    PrepareRxDoneAbort(netdev);
                    return;
                }
            }
            break;
        case FRAME_TYPE_PROPRIETARY:
            {
                memcpy( dev->LoRaMacRxPayload, &payload[pktHeaderLen], size );

                dev->McpsIndication.McpsIndication = MCPS_PROPRIETARY;
                dev->McpsIndication.Status = LORAMAC_EVENT_INFO_STATUS_OK;
                dev->McpsIndication.Buffer = dev->LoRaMacRxPayload;
                dev->McpsIndication.BufferSize = size - pktHeaderLen;

                dev->LoRaMacFlags.Bits.McpsInd = 1;
                break;
            }
        default:
            dev->McpsIndication.Status = LORAMAC_EVENT_INFO_STATUS_ERROR;
            PrepareRxDoneAbort(netdev);
            break;
    }

    if( ( dev->RxSlot == 0 ) && ( dev->LoRaMacDeviceClass == CLASS_C ) )
    {
        OnRxWindow2TimerEvent(netdev);
    }
    dev->LoRaMacFlags.Bits.MacDone = 1;

    // Trig OnMacCheckTimerEvent call as soon as possible
    dev->MacStateCheckTimer.msg.type = LORAWAN_TIMER_MAC_STATE;
    xtimer_set_msg(&(dev->MacStateCheckTimer.dev), xtimer_ticks_from_usec(1000).ticks32, &(dev->MacStateCheckTimer.msg), dev->MacStateCheckTimer.pid);
    //TimerSetValue( &MacStateCheckTimer, 1, LORAWAN_TIMER_MAC_STATE);
    //TimerStart( &MacStateCheckTimer, 0);
}

void OnRadioTxTimeout( netdev2_t *netdev )
{
    netopt_state_t state = NETOPT_STATE_SLEEP;
    if( dev->LoRaMacDeviceClass != CLASS_C )
    {
        netdev->driver->set(netdev, NETOPT_STATE, &state, sizeof(netopt_state_t));
    }
    else
    {
        OnRxWindow2TimerEvent(netdev);
    }

    dev->frame_status = LORAMAC_EVENT_INFO_STATUS_TX_TIMEOUT;
    dev->frame_status = LORAMAC_EVENT_INFO_STATUS_TX_TIMEOUT;
    dev->LoRaMacFlags.Bits.MacDone = 1;
}

void OnRadioRxError(netdev2_t *netdev)
{
    netopt_state_t state = NETOPT_STATE_SLEEP;
    if( dev->LoRaMacDeviceClass != CLASS_C )
    {
        netdev->driver->set(netdev, NETOPT_STATE, &state, sizeof(netopt_state_t));
    }
    else
    {
        OnRxWindow2TimerEvent(netdev);
    }

    if( dev->RxSlot == 1 )
    {
        //TODO: Check this
        /*
        if( dev->NodeAckRequested == true )
        {
            dev->frame_status = LORAMAC_EVENT_INFO_STATUS_RX2_ERROR;
        }
        */
        dev->frame_status = LORAMAC_EVENT_INFO_STATUS_RX2_ERROR;
        dev->LoRaMacFlags.Bits.MacDone = 1;
    }
}

void OnRadioRxTimeout(netdev2_t *netdev)
{
    netopt_state_t state = NETOPT_STATE_SLEEP;
    if( dev->LoRaMacDeviceClass != CLASS_C )
    {
        netdev->driver->set(netdev, NETOPT_STATE, &state, sizeof(netopt_state_t));
    }
    else
    {
        OnRxWindow2TimerEvent(netdev);
    }

    if( dev->RxSlot == 1 )
    {
        if( dev->NodeAckRequested == true )
        {
            dev->frame_status = LORAMAC_EVENT_INFO_STATUS_RX2_TIMEOUT;
        }
        dev->frame_status = LORAMAC_EVENT_INFO_STATUS_RX2_TIMEOUT;
        dev->LoRaMacFlags.Bits.MacDone = 1;
    }
}

void OnMacStateCheckTimerEvent(netdev2_t *netdev)
{
    //TimerStop( &MacStateCheckTimer );
    xtimer_remove(&dev->MacStateCheckTimer.dev);
    bool txTimeout = false;
    printf("%i\n", dev->frame_status);

    if( dev->LoRaMacFlags.Bits.MacDone == 1 )
    {
        if( ( dev->LoRaMacState & MAC_RX_ABORT ) == MAC_RX_ABORT )
        {
            dev->LoRaMacState &= ~MAC_RX_ABORT;
            dev->LoRaMacState &= ~MAC_TX_RUNNING;
        }

        if( ( dev->LoRaMacFlags.Bits.MlmeReq == 1 ) || ( ( dev->LoRaMacFlags.Bits.McpsReq == 1 ) ) )
        {
            if( ( dev->frame_status == LORAMAC_EVENT_INFO_STATUS_TX_TIMEOUT ) ||
                ( dev->frame_status == LORAMAC_EVENT_INFO_STATUS_TX_TIMEOUT ) )
            {
                // Stop transmit cycle due to tx timeout.
                dev->LoRaMacState &= ~MAC_TX_RUNNING;
                dev->n_retries = dev->AckTimeoutRetriesCounter;
                dev->ack_received = false;
                //dev->McpsConfirm.TxTimeOnAir = 0;
                txTimeout = true;
            }
        }

        if( ( dev->NodeAckRequested == false ) && ( txTimeout == false ) )
        {
            if( dev->LoRaMacFlags.Bits.MlmeReq == 1 )
            {
                if( dev->last_frame == FRAME_TYPE_JOIN_REQ )
                {
                    // Retransmit only if the answer is not OK
                    dev->ChannelsNbRepCounter = 0;

                    if( dev->frame_status == LORAMAC_EVENT_INFO_STATUS_OK )
                    {
                        // Stop retransmission
                        dev->ChannelsNbRepCounter = dev->LoRaMacParams.ChannelsNbRep;
                        dev->uplink_counter = 0;
                    }
                }
            }
            if( ( dev->LoRaMacFlags.Bits.MlmeReq == 1 ) || ( ( dev->LoRaMacFlags.Bits.McpsReq == 1 ) ) )
            {
                if( ( dev->ChannelsNbRepCounter >= dev->LoRaMacParams.ChannelsNbRep ) || ( dev->LoRaMacFlags.Bits.McpsInd == 1 ) )
                {
                    dev->ChannelsNbRepCounter = 0;

                    dev->AdrAckCounter++;
                    if( dev->IsUpLinkCounterFixed == false )
                    {
                        dev->uplink_counter++;
                    }

                    dev->LoRaMacState &= ~MAC_TX_RUNNING;
                }
                else
                {
                    dev->LoRaMacFlags.Bits.MacDone = 0;
                    #if defined HACK_OTA
                        /* Hack so retransmited package is re-built*/
                        if(dev->lorawan.tx_rx.nwk_status == false)
                        {
                            LoRaMacHeader_t macHdr;
                            LoRaMacFrameCtrl_t fCtrl;

                            macHdr.Value = 0;
                            macHdr.Bits.MType = FRAME_TYPE_JOIN_REQ;

                            fCtrl.Value = 0;
                            fCtrl.Bits.Adr = dev->lorawan.tx_rx.adr_ctrl;

                            /* In case of a join request retransmission, the stack must prepare
                             * the frame again, because the network server keeps track of the random
                             * dev->dev_nonce values to prevent reply attacks. */
                            PrepareFrame( &macHdr, &fCtrl, 0, NULL, 0 );
                            /* End of*/
                        }
                    #endif
                    ScheduleTx( );
                }
            }
        }

        if( dev->LoRaMacFlags.Bits.McpsInd == 1 )
        {
            if( ( dev->ack_received == true ) || ( dev->AckTimeoutRetriesCounter > dev->AckTimeoutRetries ) )
            {
                dev->AckTimeoutRetry = false;
                dev->NodeAckRequested = false;
                if( dev->IsUpLinkCounterFixed == false )
                {
                    dev->uplink_counter++;
                }
                dev->n_retries = dev->AckTimeoutRetriesCounter;

                dev->LoRaMacState &= ~MAC_TX_RUNNING;
            }
        }

        if( ( dev->AckTimeoutRetry == true ) && ( ( dev->LoRaMacState & MAC_TX_DELAYED ) == 0 ) )
        {
            dev->AckTimeoutRetry = false;
            if( ( dev->AckTimeoutRetriesCounter < dev->AckTimeoutRetries ) && ( dev->AckTimeoutRetriesCounter <= MAX_ACK_RETRIES ) )
            {
                dev->AckTimeoutRetriesCounter++;

                if( ( dev->AckTimeoutRetriesCounter % 2 ) == 1 )
                {
                    dev->LoRaMacParams.ChannelsDatarate = MAX( dev->LoRaMacParams.ChannelsDatarate - 1, LORAMAC_TX_MIN_DATARATE );
                }
                dev->LoRaMacFlags.Bits.MacDone = 0;
                // Sends the same frame again
                ScheduleTx( );
            }
            else
            {
#if defined( USE_BAND_433 ) || defined( USE_BAND_780 ) || defined( USE_BAND_868 )
                // Re-enable default channels LC1, LC2, LC3
                dev->LoRaMacParams.ChannelsMask[0] = dev->LoRaMacParams.ChannelsMask[0] | ( LC( 1 ) + LC( 2 ) + LC( 3 ) );
#elif defined( USE_BAND_915 )
                // Re-enable default channels
                dev->LoRaMacParams.ChannelsMask[0] = 0xFFFF;
                dev->LoRaMacParams.ChannelsMask[1] = 0xFFFF;
                dev->LoRaMacParams.ChannelsMask[2] = 0xFFFF;
                dev->LoRaMacParams.ChannelsMask[3] = 0xFFFF;
                dev->LoRaMacParams.ChannelsMask[4] = 0x00FF;
                dev->LoRaMacParams.ChannelsMask[5] = 0x0000;
#elif defined( USE_BAND_915_HYBRID )
                // Re-enable default channels
                ReenableChannels( dev->LoRaMacParams.ChannelsMask[4], dev->LoRaMacParams.ChannelsMask );
                dev->LoRaMacParams.ChannelsMask[0] = 0x3C3C;
                dev->LoRaMacParams.ChannelsMask[1] = 0x0000;
                dev->LoRaMacParams.ChannelsMask[2] = 0x0000;
                dev->LoRaMacParams.ChannelsMask[3] = 0x0000;
                dev->LoRaMacParams.ChannelsMask[4] = 0x0001;
                dev->LoRaMacParams.ChannelsMask[5] = 0x0000;
#else
    #error "Please define a frequency band in the compiler options."
#endif
                dev->LoRaMacState &= ~MAC_TX_RUNNING;

                dev->NodeAckRequested = false;
                dev->ack_received = false;
                dev->n_retries = dev->AckTimeoutRetriesCounter;
                if( dev->IsUpLinkCounterFixed == false )
                {
                    dev->uplink_counter++;
                }
            }
        }
    }
    // Handle reception for Class B and Class C
    if( ( dev->LoRaMacState & MAC_RX ) == MAC_RX )
    {
        dev->LoRaMacState &= ~MAC_RX;
    }
    if( dev->LoRaMacState == MAC_IDLE )
    {
        if( dev->LoRaMacFlags.Bits.McpsReq == 1 )
        {
            dev->LoRaMacPrimitives->MacMcpsConfirm();
            dev->LoRaMacFlags.Bits.McpsReq = 0;
        }

        if( dev->LoRaMacFlags.Bits.MlmeReq == 1 )
        {
            if(dev->last_frame == FRAME_TYPE_JOIN_REQ)
                dev->LoRaMacPrimitives->MacMlmeConfirm( MLME_JOIN );
            else if (dev->last_command == MOTE_MAC_LINK_CHECK_REQ)
                dev->LoRaMacPrimitives->MacMlmeConfirm(MLME_LINK_CHECK);

            dev->LoRaMacFlags.Bits.MlmeReq = 0;
        }

        dev->LoRaMacFlags.Bits.MacDone = 0;
    }
    else
    {
        // Operation not finished restart timer
        dev->MacStateCheckTimer.msg.type = LORAWAN_TIMER_MAC_STATE;
        xtimer_set_msg(&(dev->MacStateCheckTimer.dev), xtimer_ticks_from_usec(MAC_STATE_CHECK_TIMEOUT*1000).ticks32, &(dev->MacStateCheckTimer.msg), dev->MacStateCheckTimer.pid);
        //TimerSetValue( &MacStateCheckTimer, MAC_STATE_CHECK_TIMEOUT, LORAWAN_TIMER_MAC_STATE);
        //TimerStart( &MacStateCheckTimer, 0);
    }

    if( dev->LoRaMacFlags.Bits.McpsInd == 1 )
    {
        dev->LoRaMacPrimitives->MacMcpsIndication( &dev->McpsIndication );
        dev->LoRaMacFlags.Bits.McpsInd = 0;
    }
}

void OnTxDelayedTimerEvent(netdev2_t *netdev)
{
    LoRaMacHeader_t macHdr;
    LoRaMacFrameCtrl_t fCtrl;

    //TimerStop( &TxDelayedTimer );
    xtimer_remove(&dev->TxDelayedTimer.dev);
    dev->LoRaMacState &= ~MAC_TX_DELAYED;

    if( ( dev->LoRaMacFlags.Bits.MlmeReq == 1 ) && ( dev->last_frame == FRAME_TYPE_JOIN_REQ ) )
    {
        macHdr.Value = 0;
        macHdr.Bits.MType = FRAME_TYPE_JOIN_REQ;

        fCtrl.Value = 0;
        fCtrl.Bits.Adr = dev->lorawan.tx_rx.adr_ctrl;

        /* In case of a join request retransmission, the stack must prepare
         * the frame again, because the network server keeps track of the random
         * dev->dev_nonce values to prevent reply attacks. */
        PrepareFrame( &macHdr, &fCtrl, 0, NULL, 0 );
    }

    ScheduleTx( );
}

void OnRxWindow1TimerEvent(netdev2_t *netdev)
{
    uint16_t symbTimeout = 5; // DR_2, DR_1, DR_0
    int8_t datarate = 0;
    uint32_t bandwidth = 0; // LoRa 125 kHz

    //TimerStop( &RxWindowTimer1 );
    xtimer_remove(&dev->RxWindowTimer1.dev);
    dev->RxSlot = 0;

    netopt_state_t state = NETOPT_STATE_STANDBY;
    if( dev->LoRaMacDeviceClass == CLASS_C )
    {
        netdev->driver->set(netdev, NETOPT_STATE, &state, sizeof(netopt_state_t));
    }

#if defined( USE_BAND_433 ) || defined( USE_BAND_780 ) || defined( USE_BAND_868 )
    datarate = dev->LoRaMacParams.ChannelsDatarate - dev->LoRaMacParams.Rx1DrOffset;
    if( datarate < 0 )
    {
        datarate = DR_0;
    }

    // For higher datarates, we increase the number of symbols generating a Rx Timeout
    if( ( datarate == DR_3 ) || ( datarate == DR_4 ) )
    { // DR_4, DR_3
        symbTimeout = 8;
    }
    else if( datarate == DR_5 )
    {
        symbTimeout = 10;
    }
    else if( datarate == DR_6 )
    {// LoRa 250 kHz
        bandwidth  = 1;
        symbTimeout = 14;
    }
    RxWindowSetup( Channels[Channel].Frequency, datarate, bandwidth, symbTimeout, false );
#elif ( defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID ) )
    datarate = datarateOffsets[dev->LoRaMacParams.ChannelsDatarate][dev->LoRaMacParams.Rx1DrOffset];
    if( datarate < 0 )
    {
        datarate = DR_0;
    }
    // For higher datarates, we increase the number of symbols generating a Rx Timeout
    switch( datarate )
    {
        case DR_0:       // SF10 - BW125
            symbTimeout = 5;
            break;

        case DR_1:       // SF9  - BW125
        case DR_2:       // SF8  - BW125
        case DR_8:       // SF12 - BW500
        case DR_9:       // SF11 - BW500
        case DR_10:      // SF10 - BW500
            symbTimeout = 8;
            break;

        case DR_3:       // SF7  - BW125
        case DR_11:      // SF9  - BW500
            symbTimeout = 10;
            break;

        case DR_4:       // SF8  - BW500
        case DR_12:      // SF8  - BW500
            symbTimeout = 14;
            break;

        case DR_13:      // SF7  - BW500
            symbTimeout = 16;
            break;
        default:
            break;
    }
    if( datarate >= DR_4 )
    {// LoRa 500 kHz
        bandwidth  = 2;
    }
    RxWindowSetup( LORAMAC_FIRST_RX2_CHANNEL + ( dev->Channel % 8 ) * LORAMAC_STEPWIDTH_RX2_CHANNEL, datarate, bandwidth, symbTimeout, false );
#else
    #error "Please define a frequency band in the compiler options."
#endif
}

void OnRxWindow2TimerEvent(netdev2_t *netdev)
{
    uint16_t symbTimeout = 5; // DR_2, DR_1, DR_0
    uint32_t bandwidth = 0; // LoRa 125 kHz

    //TimerStop( &RxWindowTimer2 );
    xtimer_remove(&dev->RxWindowTimer2.dev);
    dev->RxSlot = 1;

#if defined( USE_BAND_433 ) || defined( USE_BAND_780 ) || defined( USE_BAND_868 )
    // For higher datarates, we increase the number of symbols generating a Rx Timeout
    if( ( dev->LoRaMacParams.Rx2Channel.Datarate == DR_3 ) || ( dev->LoRaMacParams.Rx2Channel.Datarate == DR_4 ) )
    { // DR_4, DR_3
        symbTimeout = 8;
    }
    else if( dev->LoRaMacParams.Rx2Channel.Datarate == DR_5 )
    {
        symbTimeout = 10;
    }
    else if( dev->LoRaMacParams.Rx2Channel.Datarate == DR_6 )
    {// LoRa 250 kHz
        bandwidth  = 1;
        symbTimeout = 14;
    }
#elif ( defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID ) )
    // For higher datarates, we increase the number of symbols generating a Rx Timeout
    switch( dev->LoRaMacParams.Rx2Channel.Datarate )
    {
        case DR_0:       // SF10 - BW125
            symbTimeout = 5;
            break;

        case DR_1:       // SF9  - BW125
        case DR_2:       // SF8  - BW125
        case DR_8:       // SF12 - BW500
        case DR_9:       // SF11 - BW500
        case DR_10:      // SF10 - BW500
            symbTimeout = 8;
            break;

        case DR_3:       // SF7  - BW125
        case DR_11:      // SF9  - BW500
            symbTimeout = 10;
            break;

        case DR_4:       // SF8  - BW500
        case DR_12:      // SF8  - BW500
            symbTimeout = 14;
            break;

        case DR_13:      // SF7  - BW500
            symbTimeout = 16;
            break;
        default:
            break;
    }
    if( dev->LoRaMacParams.Rx2Channel.Datarate >= DR_4 )
    {// LoRa 500 kHz
        bandwidth  = 2;
    }
#else
    #error "Please define a frequency band in the compiler options."
#endif
    if( dev->LoRaMacDeviceClass != CLASS_C )
    {
        RxWindowSetup( dev->LoRaMacParams.Rx2Channel.Frequency, dev->LoRaMacParams.Rx2Channel.Datarate, bandwidth, symbTimeout, false );
    }
    else
    {
        RxWindowSetup( dev->LoRaMacParams.Rx2Channel.Frequency, dev->LoRaMacParams.Rx2Channel.Datarate, bandwidth, symbTimeout, true );
    }
}

void OnAckTimeoutTimerEvent(netdev2_t *netdev)
{
    //TimerStop( &AckTimeoutTimer );
    xtimer_remove(&dev->AckTimeoutTimer.dev);

    if( dev->NodeAckRequested == true )
    {
        dev->AckTimeoutRetry = true;
        dev->LoRaMacState &= ~MAC_ACK_REQ;
    }
    if( dev->LoRaMacDeviceClass == CLASS_C )
    {
        dev->LoRaMacFlags.Bits.MacDone = 1;
    }
}

static bool SetNextChannel( TimerTime_t* time )
{
    uint8_t nbEnabledChannels = 0;
    uint8_t delayTx = 0;
    uint8_t enabledChannels[LORA_MAX_NB_CHANNELS];
    TimerTime_t nextTxDelay = ( TimerTime_t )( -1 );

    memset( enabledChannels, 0, LORA_MAX_NB_CHANNELS );

#if defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )
    if( CountNbEnabled125kHzChannels( dev->ChannelsMaskRemaining ) == 0 )
    { // Restore default channels
        memcpy( ( uint8_t* ) dev->ChannelsMaskRemaining, ( uint8_t* ) dev->LoRaMacParams.ChannelsMask, 8 );
    }
    if( ( dev->LoRaMacParams.ChannelsDatarate >= DR_4 ) && ( ( dev->ChannelsMaskRemaining[4] & 0x00FF ) == 0 ) )
    { // Make sure, that the channels are activated
        dev->ChannelsMaskRemaining[4] = dev->LoRaMacParams.ChannelsMask[4];
    }
#else
    if( CountBits( dev->LoRaMacParams.ChannelsMask[0], 16 ) == 0 )
    {
        // Re-enable default channels, if no channel is enabled
        dev->LoRaMacParams.ChannelsMask[0] = dev->LoRaMacParams.ChannelsMask[0] | ( LC( 1 ) + LC( 2 ) + LC( 3 ) );
    }
#endif

    // Update Aggregated duty cycle
    if( dev->AggregatedTimeOff <= (xtimer_now_usec64()-dev->AggregatedLastTxDoneTime ) )
    {
        dev->AggregatedTimeOff = 0;

        // Update bands Time OFF
        for( uint8_t i = 0; i < LORA_MAX_NB_BANDS; i++ )
        {
            if( dev->DutyCycleOn == true )
            {
                if( Bands[i].TimeOff <= (xtimer_now_usec64() - Bands[i].LastTxDoneTime ) )
                {
                    Bands[i].TimeOff = 0;
                }
                if( Bands[i].TimeOff != 0 )
                {
                    nextTxDelay = MIN( Bands[i].TimeOff -
                                       (xtimer_now_usec64()- Bands[i].LastTxDoneTime ),
                                       nextTxDelay );
                }
            }
            else
            {
                nextTxDelay = 0;
                Bands[i].TimeOff = 0;
            }
        }

        // Search how many channels are enabled
        for( uint8_t i = 0, k = 0; i < LORA_MAX_NB_CHANNELS; i += 16, k++ )
        {
            for( uint8_t j = 0; j < 16; j++ )
            {
#if defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )
                if( ( dev->ChannelsMaskRemaining[k] & ( 1 << j ) ) != 0 )
#else
                if( ( dev->LoRaMacParams.ChannelsMask[k] & ( 1 << j ) ) != 0 )
#endif
                {
                    if( Channels[i + j].Frequency == 0 )
                    { // Check if the channel is enabled
                        continue;
                    }
#if defined( USE_BAND_868 ) || defined( USE_BAND_433 ) || defined( USE_BAND_780 )
                    if( dev->lorawan.tx_rx.nwk_status == false )
                    {
                        if( ( JOIN_CHANNELS & ( 1 << j ) ) == 0 )
                        {
                            continue;
                        }
                    }
#endif
                    if( ( ( Channels[i + j].DrRange.Fields.Min <= dev->LoRaMacParams.ChannelsDatarate ) &&
                          ( dev->LoRaMacParams.ChannelsDatarate <= Channels[i + j].DrRange.Fields.Max ) ) == false )
                    { // Check if the current channel selection supports the given datarate
                        continue;
                    }
                    if( Bands[Channels[i + j].Band].TimeOff > 0 )
                    { // Check if the band is available for transmission
                        delayTx++;
                        continue;
                    }
                    enabledChannels[nbEnabledChannels++] = i + j;
                }
            }
        }
    }
    else
    {
        delayTx++;
        nextTxDelay = dev->AggregatedTimeOff - (xtimer_now_usec64() - dev->AggregatedLastTxDoneTime );
    }

    if( nbEnabledChannels > 0 )
    {
        dev->Channel = enabledChannels[random_uint32_range( 0, nbEnabledChannels)];
#if defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )
        if( dev->Channel < ( LORA_MAX_NB_CHANNELS - 8 ) )
        {
            DisableChannelInMask( dev->Channel, dev->ChannelsMaskRemaining );
        }
#endif
        *time = 0;
        return true;
    }
    else
    {
        if( delayTx > 0 )
        {
            // Delay transmission due to dev->AggregatedTimeOff or to a band time off
            *time = nextTxDelay;
            return true;
        }
        // Datarate not supported by any channel
        *time = 0;
        return false;
    }
}

static void SetPublicNetwork( bool enable )
{
    netdev2_t *netdev = (netdev2_t*) dev;
    dev->lorawan.public = enable;
    netopt_enable_t lm = NETOPT_ENABLE;
    netdev->driver->set(netdev, NETOPT_LORA_MODE, &lm, sizeof(netopt_enable_t));
    uint8_t sw;
    if( dev->lorawan.public == true )
    {
        // Change LoRa modem SyncWord
        sw = LORA_MAC_PUBLIC_SYNCWORD;
    }
    else
    {
        // Change LoRa modem SyncWord
        sw = LORA_MAC_PRIVATE_SYNCWORD;
    }
    netdev->driver->set(netdev, NETOPT_LORA_SYNCWORD, &sw, sizeof(uint8_t));
}

void _set_rx_config( RadioModems_t modem, uint32_t bandwidth,
                         uint32_t datarate, uint8_t coderate,
                         uint32_t bandwidthAfc, uint16_t preambleLen,
                         uint16_t symbTimeout, bool fixLen,
                         uint8_t payloadLen,
                         bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                         bool iqInverted, bool rxContinuous )
{
    netdev2_t *netdev = (netdev2_t*) dev;

    netopt_enable_t _modem = modem;
    netdev->driver->set(netdev, NETOPT_LORA_MODE, &_modem, sizeof(netopt_enable_t));
    (void) bandwidthAfc;

    bool freq_hop_on = freqHopOn;
    bool iq_invert = iqInverted;
    uint8_t rx_single = rxContinuous ? false : true;
    uint32_t tx_timeout = 3 * 1000 * 1000;
    uint8_t bw = bandwidth + 7;
    uint8_t cr = coderate;
    uint8_t sf = datarate;
    bool implicit = fixLen;
    bool crc = crcOn;
    uint16_t rx_timeout = symbTimeout;
    uint16_t preamble = preambleLen;
    uint8_t payload_len = payloadLen;
    uint8_t hop_period = hopPeriod;
    uint8_t power = 14;

    netdev->driver->set(netdev, NETOPT_LORA_HOP, &freq_hop_on, sizeof(bool));
    netdev->driver->set(netdev, NETOPT_LORA_IQ_INVERT, &iq_invert, sizeof(bool));
    netdev->driver->set(netdev, NETOPT_LORA_SINGLE_RECEIVE, &rx_single, sizeof(uint8_t));
    netdev->driver->set(netdev, NETOPT_LORA_TX_TIMEOUT, &tx_timeout, sizeof(uint32_t));

    netdev->driver->set(netdev, NETOPT_LORA_BANDWIDTH, &bw, sizeof(uint8_t));
    netdev->driver->set(netdev, NETOPT_LORA_CODING_RATE, &cr, sizeof(uint8_t));
    netdev->driver->set(netdev, NETOPT_LORA_SPREADING_FACTOR, &sf, sizeof(uint8_t));
    netdev->driver->set(netdev, NETOPT_LORA_IMPLICIT, &implicit, sizeof(bool));
    netdev->driver->set(netdev, NETOPT_CRC, &crc, sizeof(bool));
    netdev->driver->set(netdev, NETOPT_LORA_SYMBOL_TIMEOUT, &rx_timeout, sizeof(uint16_t));
    netdev->driver->set(netdev, NETOPT_LORA_PREAMBLE_LENGTH, &preamble, sizeof(uint16_t));
    netdev->driver->set(netdev, NETOPT_LORA_PAYLOAD_LENGTH, &payload_len, sizeof(uint8_t));
    netdev->driver->set(netdev, NETOPT_LORA_HOP_PERIOD, &hop_period, sizeof(uint8_t));
    netdev->driver->set(netdev, NETOPT_TX_POWER, &power, sizeof(uint8_t));
}
static void RxWindowSetup( uint32_t freq, int8_t datarate, uint32_t bandwidth, uint16_t timeout, bool rxContinuous )
{
    netdev2_t *netdev = (netdev2_t*) dev;
    uint8_t downlinkDatarate = Datarates[datarate];
    RadioModems_t modem;

    netopt_state_t state;
    netdev->driver->get(netdev, NETOPT_STATE, &state, sizeof(netopt_state_t)); 

    netopt_enable_t lm;
    uint8_t max_payload;

    if( state == NETOPT_STATE_SLEEP || state == NETOPT_STATE_STANDBY )
    {
        netdev->driver->set(netdev, NETOPT_CHANNEL, &freq, sizeof(uint32_t));

        // Store downlink datarate
        dev->McpsIndication.RxDatarate = ( uint8_t ) datarate;

#if defined( USE_BAND_433 ) || defined( USE_BAND_780 ) || defined( USE_BAND_868 )
        if( datarate == DR_7 )
        {
            modem = MODEM_FSK;
            _set_rx_config( modem, 50e3, downlinkDatarate * 1e3, 0, 83.333e3, 5, 0, false, 0, true, 0, 0, false, rxContinuous );
        }
        else
        {
            modem = MODEM_LORA;
            _set_rx_config( modem, bandwidth, downlinkDatarate, 1, 0, 8, timeout, false, 0, false, 0, 0, true, rxContinuous );
        }
#elif defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )
        modem = MODEM_LORA;

        #if defined HACK_OTA
            /* Hack Begin*/
            if(dev->lorawan.tx_rx.nwk_status == false)
            {
                (void) timeout;
                _set_rx_config( modem, bandwidth, downlinkDatarate, 1, 0, 8, 128, false, 0, false, 0, 0, true, rxContinuous );
            }
            else
                _set_rx_config( modem, bandwidth, downlinkDatarate, 1, 0, 8, timeout, false, 0, false, 0, 0, true, rxContinuous );
            /* Hack End*/
        #else
            _set_rx_config( modem, bandwidth, downlinkDatarate, 1, 0, 8, timeout, false, 0, false, 0, 0, true, rxContinuous );
        #endif

#endif

        lm = modem == MODEM_LORA ? NETOPT_ENABLE : NETOPT_DISABLE;
        netdev->driver->set(netdev, NETOPT_LORA_MODE, &lm, sizeof(netopt_enable_t));
        if( dev->RepeaterSupport == true )
        {
            max_payload = MaxPayloadOfDatarateRepeater[datarate] + LORA_MAC_FRMPAYLOAD_OVERHEAD;
        }
        else
        {
            max_payload = MaxPayloadOfDatarate[datarate] + LORA_MAC_FRMPAYLOAD_OVERHEAD ;
        }

        netdev->driver->set(netdev, NETOPT_LORA_MAX_PAYLOAD, &max_payload, sizeof(uint8_t));
        uint32_t val;
        netopt_state_t state = NETOPT_STATE_RX;
        if( rxContinuous == false )
        {
            val = dev->LoRaMacParams.MaxRxWindow*1000;
        }
        else
        {
            val = 0;
        }
        netdev->driver->set(netdev, NETOPT_LORA_RX_TIMEOUT, &val, sizeof(uint32_t));
        netdev->driver->set(netdev, NETOPT_STATE, &state, sizeof(netopt_state_t));
    }
}

static bool Rx2FreqInRange( uint32_t freq )
{
#if defined( USE_BAND_433 ) || defined( USE_BAND_780 ) || defined( USE_BAND_868 )
    if( check_rf_freq( freq ) == true )
#elif ( defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID ) )
    if( ( check_rf_freq( freq ) == true ) &&
        ( freq >= LORAMAC_FIRST_RX2_CHANNEL ) &&
        ( freq <= LORAMAC_LAST_RX2_CHANNEL ) &&
        ( ( ( freq - ( uint32_t ) LORAMAC_FIRST_RX2_CHANNEL ) % ( uint32_t ) LORAMAC_STEPWIDTH_RX2_CHANNEL ) == 0 ) )
#endif
    {
        return true;
    }
    return false;
}

static bool ValidatePayloadLength( uint8_t lenN, int8_t datarate, uint8_t fOptsLen )
{
    uint16_t maxN = 0;
    uint16_t payloadSize = 0;

    // Get the maximum payload length
    if( dev->RepeaterSupport == true )
    {
        maxN = MaxPayloadOfDatarateRepeater[datarate];
    }
    else
    {
        maxN = MaxPayloadOfDatarate[datarate];
    }

    // Calculate the resulting payload size
    payloadSize = ( lenN + fOptsLen );

    // Validation of the application payload size
    if( ( payloadSize <= maxN ) && ( payloadSize <= LORAMAC_PHY_MAXPAYLOAD ) )
    {
        return true;
    }
    return false;
}

static uint8_t CountBits( uint16_t mask, uint8_t nbBits )
{
    uint8_t nbActiveBits = 0;

    for( uint8_t j = 0; j < nbBits; j++ )
    {
        if( ( mask & ( 1 << j ) ) == ( 1 << j ) )
        {
            nbActiveBits++;
        }
    }
    return nbActiveBits;
}

#if defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )
static uint8_t CountNbEnabled125kHzChannels( uint16_t *channelsMask )
{
    uint8_t nb125kHzChannels = 0;

    for( uint8_t i = 0, k = 0; i < LORA_MAX_NB_CHANNELS - 8; i += 16, k++ )
    {
        nb125kHzChannels += CountBits( channelsMask[k], 16 );
    }

    return nb125kHzChannels;
}

#if defined( USE_BAND_915_HYBRID )
static void ReenableChannels( uint16_t mask, uint16_t* channelMask )
{
    uint16_t blockMask = mask;

    for( uint8_t i = 0, j = 0; i < 4; i++, j += 2 )
    {
        channelMask[i] = 0;
        if( ( blockMask & ( 1 << j ) ) != 0 )
        {
            channelMask[i] |= 0x00FF;
        }
        if( ( blockMask & ( 1 << ( j + 1 ) ) ) != 0 )
        {
            channelMask[i] |= 0xFF00;
        }
    }
    channelMask[4] = blockMask;
    channelMask[5] = 0x0000;
}

static bool ValidateChannelMask( uint16_t* channelMask )
{
    bool chanMaskState = false;
    uint16_t block1 = 0;
    uint16_t block2 = 0;
    uint8_t index = 0;

    for( uint8_t i = 0; i < 4; i++ )
    {
        block1 = channelMask[i] & 0x00FF;
        block2 = channelMask[i] & 0xFF00;

        if( ( CountBits( block1, 16 ) > 5 ) && ( chanMaskState == false ) )
        {
            channelMask[i] &= block1;
            channelMask[4] = 1 << ( i * 2 );
            chanMaskState = true;
            index = i;
        }
        else if( ( CountBits( block2, 16 ) > 5 ) && ( chanMaskState == false ) )
        {
            channelMask[i] &= block2;
            channelMask[4] = 1 << ( i * 2 + 1 );
            chanMaskState = true;
            index = i;
        }
    }

    // Do only change the channel mask, if we have found a valid block.
    if( chanMaskState == true )
    {
        for( uint8_t i = 0; i < 4; i++ )
        {
            if( i != index )
            {
                channelMask[i] = 0;
            }
        }
    }
    return chanMaskState;
}
#endif
#endif

static int8_t LimitTxPower( int8_t txPower )
{
    int8_t resultTxPower = txPower;
#if defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )
    if( ( dev->LoRaMacParams.ChannelsDatarate == DR_4 ) ||
        ( ( dev->LoRaMacParams.ChannelsDatarate >= DR_8 ) && ( dev->LoRaMacParams.ChannelsDatarate <= DR_13 ) ) )
    {// Limit tx power to max 26dBm
        resultTxPower =  MAX( txPower, TX_POWER_26_DBM );
    }
    else
    {
        if( CountNbEnabled125kHzChannels( dev->LoRaMacParams.ChannelsMask ) < 50 )
        {// Limit tx power to max 21dBm
            resultTxPower = MAX( txPower, TX_POWER_20_DBM );
        }
    }
#endif
    return resultTxPower;
}

static bool ValueInRange( int8_t value, int8_t min, int8_t max )
{
    if( ( value >= min ) && ( value <= max ) )
    {
        return true;
    }
    return false;
}

static bool DisableChannelInMask( uint8_t id, uint16_t* mask )
{
    uint8_t index = 0;
    index = id / 16;

    if( ( index > 4 ) || ( id >= LORA_MAX_NB_CHANNELS ) )
    {
        return false;
    }

    // Deactivate channel
    mask[index] &= ~( 1 << ( id % 16 ) );

    return true;
}

static bool AdrNextDr( bool adrEnabled, bool updateChannelMask, int8_t* datarateOut )
{
    bool adrAckReq = false;
    int8_t datarate = dev->LoRaMacParams.ChannelsDatarate;

    if( adrEnabled == true )
    {
        if( datarate == LORAMAC_TX_MIN_DATARATE )
        {
            dev->AdrAckCounter = 0;
            adrAckReq = false;
        }
        else
        {
            if( dev->AdrAckCounter >= ADR_ACK_LIMIT )
            {
                adrAckReq = true;
            }
            else
            {
                adrAckReq = false;
            }
            if( dev->AdrAckCounter >= ( ADR_ACK_LIMIT + ADR_ACK_DELAY ) )
            {
                if( ( dev->AdrAckCounter % ADR_ACK_DELAY ) == 0 )
                {
#if defined( USE_BAND_433 ) || defined( USE_BAND_780 ) || defined( USE_BAND_868 )
                    if( datarate > LORAMAC_TX_MIN_DATARATE )
                    {
                        datarate--;
                    }
                    if( datarate == LORAMAC_TX_MIN_DATARATE )
                    {
                        if( updateChannelMask == true )
                        {
                            // Re-enable default channels LC1, LC2, LC3
                            dev->LoRaMacParams.ChannelsMask[0] = dev->LoRaMacParams.ChannelsMask[0] | ( LC( 1 ) + LC( 2 ) + LC( 3 ) );
                        }
                    }
#elif defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )
                    if( ( datarate > LORAMAC_TX_MIN_DATARATE ) && ( datarate == DR_8 ) )
                    {
                        datarate = DR_4;
                    }
                    else if( datarate > LORAMAC_TX_MIN_DATARATE )
                    {
                        datarate--;
                    }
                    if( datarate == LORAMAC_TX_MIN_DATARATE )
                    {
                        if( updateChannelMask == true )
                        {
#if defined( USE_BAND_915 )
                            // Re-enable default channels
                            dev->LoRaMacParams.ChannelsMask[0] = 0xFFFF;
                            dev->LoRaMacParams.ChannelsMask[1] = 0xFFFF;
                            dev->LoRaMacParams.ChannelsMask[2] = 0xFFFF;
                            dev->LoRaMacParams.ChannelsMask[3] = 0xFFFF;
                            dev->LoRaMacParams.ChannelsMask[4] = 0x00FF;
                            dev->LoRaMacParams.ChannelsMask[5] = 0x0000;
#else // defined( USE_BAND_915_HYBRID )
                            // Re-enable default channels
                            ReenableChannels( dev->LoRaMacParams.ChannelsMask[4], dev->LoRaMacParams.ChannelsMask );
                            dev->LoRaMacParams.ChannelsMask[0] = 0x3C3C;
                            dev->LoRaMacParams.ChannelsMask[1] = 0x0000;
                            dev->LoRaMacParams.ChannelsMask[2] = 0x0000;
                            dev->LoRaMacParams.ChannelsMask[3] = 0x0000;
                            dev->LoRaMacParams.ChannelsMask[4] = 0x0001;
                            dev->LoRaMacParams.ChannelsMask[5] = 0x0000;
#endif
                        }
                    }
#else
#error "Please define a frequency band in the compiler options."
#endif
                }
            }
        }
    }

    *datarateOut = datarate;

    return adrAckReq;
}

static LoRaMacStatus_t AddMacCommand( uint8_t cmd, uint8_t p1, uint8_t p2 )
{
    LoRaMacStatus_t status = LORAMAC_STATUS_BUSY;
    // The maximum buffer length must take MAC commands to re-send into account.
    uint8_t bufLen = LORA_MAC_COMMAND_MAX_LENGTH - dev->MacCommandsBufferToRepeatIndex;

    switch( cmd )
    {
        case MOTE_MAC_LINK_CHECK_REQ:
            if( dev->MacCommandsBufferIndex < bufLen )
            {
                dev->MacCommandsBuffer[dev->MacCommandsBufferIndex++] = cmd;
                // No payload for this command
                status = LORAMAC_STATUS_OK;
            }
            break;
        case MOTE_MAC_LINK_ADR_ANS:
            if( dev->MacCommandsBufferIndex < ( bufLen - 1 ) )
            {
                dev->MacCommandsBuffer[dev->MacCommandsBufferIndex++] = cmd;
                // Margin
                dev->MacCommandsBuffer[dev->MacCommandsBufferIndex++] = p1;
                status = LORAMAC_STATUS_OK;
            }
            break;
        case MOTE_MAC_DUTY_CYCLE_ANS:
            if( dev->MacCommandsBufferIndex < bufLen )
            {
                dev->MacCommandsBuffer[dev->MacCommandsBufferIndex++] = cmd;
                // No payload for this answer
                status = LORAMAC_STATUS_OK;
            }
            break;
        case MOTE_MAC_RX_PARAM_SETUP_ANS:
            if( dev->MacCommandsBufferIndex < ( bufLen - 1 ) )
            {
                dev->MacCommandsBuffer[dev->MacCommandsBufferIndex++] = cmd;
                // Status: Datarate ACK, Channel ACK
                dev->MacCommandsBuffer[dev->MacCommandsBufferIndex++] = p1;
                status = LORAMAC_STATUS_OK;
            }
            break;
        case MOTE_MAC_DEV_STATUS_ANS:
            if( dev->MacCommandsBufferIndex < ( bufLen - 2 ) )
            {
                dev->MacCommandsBuffer[dev->MacCommandsBufferIndex++] = cmd;
                // 1st byte Battery
                // 2nd byte Margin
                dev->MacCommandsBuffer[dev->MacCommandsBufferIndex++] = p1;
                dev->MacCommandsBuffer[dev->MacCommandsBufferIndex++] = p2;
                status = LORAMAC_STATUS_OK;
            }
            break;
        case MOTE_MAC_NEW_CHANNEL_ANS:
            if( dev->MacCommandsBufferIndex < ( bufLen - 1 ) )
            {
                dev->MacCommandsBuffer[dev->MacCommandsBufferIndex++] = cmd;
                // Status: Datarate range OK, Channel frequency OK
                dev->MacCommandsBuffer[dev->MacCommandsBufferIndex++] = p1;
                status = LORAMAC_STATUS_OK;
            }
            break;
        case MOTE_MAC_RX_TIMING_SETUP_ANS:
            if( dev->MacCommandsBufferIndex < bufLen )
            {
                dev->MacCommandsBuffer[dev->MacCommandsBufferIndex++] = cmd;
                // No payload for this answer
                status = LORAMAC_STATUS_OK;
            }
            break;
        default:
            return LORAMAC_STATUS_SERVICE_UNKNOWN;
    }
    if( status == LORAMAC_STATUS_OK )
    {
        dev->MacCommandsInNextTx = true;
    }
    return status;
}

static uint8_t ParseMacCommandsToRepeat( uint8_t* cmdBufIn, uint8_t length, uint8_t* cmdBufOut )
{
    uint8_t i = 0;
    uint8_t cmdCount = 0;

    if( ( cmdBufIn == NULL ) || ( cmdBufOut == NULL ) )
    {
        return 0;
    }

    for( i = 0; i < length; i++ )
    {
        switch( cmdBufIn[i] )
        {
            case MOTE_MAC_RX_PARAM_SETUP_ANS:
            {
                cmdBufOut[cmdCount++] = cmdBufIn[i++];
                cmdBufOut[cmdCount++] = cmdBufIn[i++];
                cmdBufOut[cmdCount++] = cmdBufIn[i];
                break;
            }
            case MOTE_MAC_RX_TIMING_SETUP_ANS:
            {
                cmdBufOut[cmdCount++] = cmdBufIn[i];
                break;
            }
            default:
                break;
        }
    }

    return cmdCount;
}

static void ProcessMacCommands( uint8_t *payload, uint8_t macIndex, uint8_t commandsSize, uint8_t snr )
{
    while( macIndex < commandsSize )
    {
        // Decode Frame MAC commands
        switch( payload[macIndex++] )
        {
            case SRV_MAC_LINK_CHECK_ANS:
                dev->frame_status = LORAMAC_EVENT_INFO_STATUS_OK;
                dev->demod_margin = payload[macIndex++];
                dev->number_of_gateways = payload[macIndex++];
                break;
            case SRV_MAC_LINK_ADR_REQ:
                {
                    uint8_t i;
                    uint8_t status = 0x07;
                    uint16_t chMask;
                    int8_t txPower = 0;
                    int8_t datarate = 0;
                    uint8_t nbRep = 0;
                    uint8_t chMaskCntl = 0;
                    uint16_t channelsMask[6] = { 0, 0, 0, 0, 0, 0 };

                    // Initialize local copy of the channels mask array
                    for( i = 0; i < 6; i++ )
                    {
                        channelsMask[i] = dev->LoRaMacParams.ChannelsMask[i];
                    }
                    datarate = payload[macIndex++];
                    txPower = datarate & 0x0F;
                    datarate = ( datarate >> 4 ) & 0x0F;

                    if( ( dev->lorawan.tx_rx.adr_ctrl == false ) &&
                        ( ( dev->LoRaMacParams.ChannelsDatarate != datarate ) || ( dev->LoRaMacParams.ChannelsTxPower != txPower ) ) )
                    { // ADR disabled don't handle ADR requests if server tries to change datarate or txpower
                        // Answer the server with fail status
                        // Power ACK     = 0
                        // Data rate ACK = 0
                        // Channel mask  = 0
                        AddMacCommand( MOTE_MAC_LINK_ADR_ANS, 0, 0 );
                        macIndex += 3;  // Skip over the remaining bytes of the request
                        break;
                    }
                    chMask = ( uint16_t )payload[macIndex++];
                    chMask |= ( uint16_t )payload[macIndex++] << 8;

                    nbRep = payload[macIndex++];
                    chMaskCntl = ( nbRep >> 4 ) & 0x07;
                    nbRep &= 0x0F;
                    if( nbRep == 0 )
                    {
                        nbRep = 1;
                    }
#if defined( USE_BAND_433 ) || defined( USE_BAND_780 ) || defined( USE_BAND_868 )
                    if( ( chMaskCntl == 0 ) && ( chMask == 0 ) )
                    {
                        status &= 0xFE; // Channel mask KO
                    }
                    else if( ( ( chMaskCntl >= 1 ) && ( chMaskCntl <= 5 )) ||
                             ( chMaskCntl >= 7 ) )
                    {
                        // RFU
                        status &= 0xFE; // Channel mask KO
                    }
                    else
                    {
                        for( i = 0; i < LORA_MAX_NB_CHANNELS; i++ )
                        {
                            if( chMaskCntl == 6 )
                            {
                                if( Channels[i].Frequency != 0 )
                                {
                                    chMask |= 1 << i;
                                }
                            }
                            else
                            {
                                if( ( ( chMask & ( 1 << i ) ) != 0 ) &&
                                    ( Channels[i].Frequency == 0 ) )
                                {// Trying to enable an undefined channel
                                    status &= 0xFE; // Channel mask KO
                                }
                            }
                        }
                        channelsMask[0] = chMask;
                    }
#elif defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )
                    if( chMaskCntl == 6 )
                    {
                        // Enable all 125 kHz channels
                        for( uint8_t i = 0, k = 0; i < LORA_MAX_NB_CHANNELS - 8; i += 16, k++ )
                        {
                            for( uint8_t j = 0; j < 16; j++ )
                            {
                                if( Channels[i + j].Frequency != 0 )
                                {
                                    channelsMask[k] |= 1 << j;
                                }
                            }
                        }
                    }
                    else if( chMaskCntl == 7 )
                    {
                        // Disable all 125 kHz channels
                        channelsMask[0] = 0x0000;
                        channelsMask[1] = 0x0000;
                        channelsMask[2] = 0x0000;
                        channelsMask[3] = 0x0000;
                    }
                    else if( chMaskCntl == 5 )
                    {
                        // RFU
                        status &= 0xFE; // Channel mask KO
                    }
                    else
                    {
                        for( uint8_t i = 0; i < 16; i++ )
                        {
                            if( ( ( chMask & ( 1 << i ) ) != 0 ) &&
                                ( Channels[chMaskCntl * 16 + i].Frequency == 0 ) )
                            {// Trying to enable an undefined channel
                                status &= 0xFE; // Channel mask KO
                            }
                        }
                        channelsMask[chMaskCntl] = chMask;

                        if( CountNbEnabled125kHzChannels( channelsMask ) < 6 )
                        {
                            status &= 0xFE; // Channel mask KO
                        }

#if defined( USE_BAND_915_HYBRID )
                        if( ValidateChannelMask( channelsMask ) == false )
                        {
                            status &= 0xFE; // Channel mask KO
                        }
#endif
                    }
#else
    #error "Please define a frequency band in the compiler options."
#endif
                    if( ValueInRange( datarate, LORAMAC_TX_MIN_DATARATE, LORAMAC_TX_MAX_DATARATE ) == false )
                    {
                        status &= 0xFD; // Datarate KO
                    }

                    //
                    // Remark MaxTxPower = 0 and MinTxPower = 5
                    //
                    if( ValueInRange( txPower, LORAMAC_MAX_TX_POWER, LORAMAC_MIN_TX_POWER ) == false )
                    {
                        status &= 0xFB; // TxPower KO
                    }
                    if( ( status & 0x07 ) == 0x07 )
                    {
                        dev->LoRaMacParams.ChannelsDatarate = datarate;
                        dev->LoRaMacParams.ChannelsTxPower = txPower;

                        dev->LoRaMacParams.ChannelsMask[0] = channelsMask[0];
                        dev->LoRaMacParams.ChannelsMask[1] = channelsMask[1];
                        dev->LoRaMacParams.ChannelsMask[2] = channelsMask[2];
                        dev->LoRaMacParams.ChannelsMask[3] = channelsMask[3];
                        dev->LoRaMacParams.ChannelsMask[4] = channelsMask[4];
                        dev->LoRaMacParams.ChannelsMask[5] = channelsMask[5];

                        dev->LoRaMacParams.ChannelsNbRep = nbRep;
                    }
                    AddMacCommand( MOTE_MAC_LINK_ADR_ANS, status, 0 );
                }
                break;
            case SRV_MAC_DUTY_CYCLE_REQ:
                dev->MaxDCycle = payload[macIndex++];
                dev->AggregatedDCycle = 1 << dev->MaxDCycle;
                AddMacCommand( MOTE_MAC_DUTY_CYCLE_ANS, 0, 0 );
                break;
            case SRV_MAC_RX_PARAM_SETUP_REQ:
                {
                    uint8_t status = 0x07;
                    int8_t datarate = 0;
                    int8_t drOffset = 0;
                    uint32_t freq = 0;

                    drOffset = ( payload[macIndex] >> 4 ) & 0x07;
                    datarate = payload[macIndex] & 0x0F;
                    macIndex++;

                    freq =  ( uint32_t )payload[macIndex++];
                    freq |= ( uint32_t )payload[macIndex++] << 8;
                    freq |= ( uint32_t )payload[macIndex++] << 16;
                    freq *= 100;

                    if( Rx2FreqInRange( freq ) == false )
                    {
                        status &= 0xFE; // Channel frequency KO
                    }

                    if( ValueInRange( datarate, LORAMAC_RX_MIN_DATARATE, LORAMAC_RX_MAX_DATARATE ) == false )
                    {
                        status &= 0xFD; // Datarate KO
                    }
#if ( defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID ) )
                    if( ( ValueInRange( datarate, DR_5, DR_7 ) == true ) ||
                        ( datarate > DR_13 ) )
                    {
                        status &= 0xFD; // Datarate KO
                    }
#endif
                    if( ValueInRange( drOffset, LORAMAC_MIN_RX1_DR_OFFSET, LORAMAC_MAX_RX1_DR_OFFSET ) == false )
                    {
                        status &= 0xFB; // Rx1DrOffset range KO
                    }

                    if( ( status & 0x07 ) == 0x07 )
                    {
                        dev->LoRaMacParams.Rx2Channel.Datarate = datarate;
                        dev->LoRaMacParams.Rx2Channel.Frequency = freq;
                        dev->LoRaMacParams.Rx1DrOffset = drOffset;
                    }
                    AddMacCommand( MOTE_MAC_RX_PARAM_SETUP_ANS, status, 0 );
                }
                break;
            case SRV_MAC_NEW_CHANNEL_REQ:
                {
                    uint8_t status = 0x03;

#if ( defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID ) )
                    status &= 0xFC; // Channel frequency and datarate KO
                    macIndex += 5;
#else
                    int8_t channelIndex = 0;
                    ChannelParams_t chParam;

                    channelIndex = payload[macIndex++];
                    chParam.Frequency = ( uint32_t )payload[macIndex++];
                    chParam.Frequency |= ( uint32_t )payload[macIndex++] << 8;
                    chParam.Frequency |= ( uint32_t )payload[macIndex++] << 16;
                    chParam.Frequency *= 100;
                    chParam.DrRange.Value = payload[macIndex++];

                    dev->LoRaMacState |= MAC_TX_CONFIG;
                    if( chParam.Frequency == 0 )
                    {
                        if( channelIndex < 3 )
                        {
                            status &= 0xFC;
                        }
                        else
                        {
                            if( LoRaMacChannelRemove( channelIndex ) != LORAMAC_STATUS_OK )
                            {
                                status &= 0xFC;
                            }
                        }
                    }
                    else
                    {
                        switch( LoRaMacChannelAdd( channelIndex, chParam ) )
                        {
                            case LORAMAC_STATUS_OK:
                            {
                                break;
                            }
                            case LORAMAC_STATUS_FREQUENCY_INVALID:
                            {
                                status &= 0xFE;
                                break;
                            }
                            case LORAMAC_STATUS_DATARATE_INVALID:
                            {
                                status &= 0xFD;
                                break;
                            }
                            case LORAMAC_STATUS_FREQ_AND_DR_INVALID:
                            {
                                status &= 0xFC;
                                break;
                            }
                            default:
                            {
                                status &= 0xFC;
                                break;
                            }
                        }
                    }
                    dev->LoRaMacState &= ~MAC_TX_CONFIG;
#endif
                    AddMacCommand( MOTE_MAC_NEW_CHANNEL_ANS, status, 0 );
                }
                break;
            case SRV_MAC_RX_TIMING_SETUP_REQ:
                {
                    uint8_t delay = payload[macIndex++] & 0x0F;

                    if( delay == 0 )
                    {
                        delay++;
                    }
                    dev->LoRaMacParams.ReceiveDelay1 = delay * 1e3;
                    dev->LoRaMacParams.ReceiveDelay2 = dev->LoRaMacParams.ReceiveDelay1 + 1e3;
                    AddMacCommand( MOTE_MAC_RX_TIMING_SETUP_ANS, 0, 0 );
                }
                break;
            default:
                // Unknown command. ABORT MAC commands processing
                return;
        }
    }
}

LoRaMacStatus_t Send( LoRaMacHeader_t *macHdr, uint8_t fPort, void *fBuffer, uint16_t fBufferSize )
{
    LoRaMacFrameCtrl_t fCtrl;
    LoRaMacStatus_t status = LORAMAC_STATUS_PARAMETER_INVALID;

    fCtrl.Value = 0;
    fCtrl.Bits.FOptsLen      = 0;
    fCtrl.Bits.FPending      = 0;
    fCtrl.Bits.Ack           = false;
    fCtrl.Bits.AdrAckReq     = false;
    fCtrl.Bits.Adr           = dev->lorawan.tx_rx.adr_ctrl;

    // Prepare the frame
    status = PrepareFrame( macHdr, &fCtrl, fPort, fBuffer, fBufferSize );

    // Validate status
    if( status != LORAMAC_STATUS_OK )
    {
        return status;
    }

    // Reset confirm parameters
    dev->n_retries = 0;
    dev->ack_received = false;
    //dev->McpsConfirm.uplink_counter = dev->UpLinkCounter;

    status = ScheduleTx( );

    return status;
}

static LoRaMacStatus_t ScheduleTx(void)
{
    TimerTime_t dutyCycleTimeOff = 0;

    // Check if the device is off
    if( dev->MaxDCycle == 255 )
    {
        return LORAMAC_STATUS_DEVICE_OFF;
    }
    if( dev->MaxDCycle == 0 )
    {
        dev->AggregatedTimeOff = 0;
    }

    CalculateBackOff( dev->LastTxChannel );

    // Select channel
    while( SetNextChannel( &dutyCycleTimeOff ) == false )
    {
        // Set the default datarate
        dev->LoRaMacParams.ChannelsDatarate = dev->LoRaMacParamsDefaults.ChannelsDatarate;

#if defined( USE_BAND_433 ) || defined( USE_BAND_780 ) || defined( USE_BAND_868 )
        // Re-enable default channels LC1, LC2, LC3
        dev->LoRaMacParams.ChannelsMask[0] = dev->LoRaMacParams.ChannelsMask[0] | ( LC( 1 ) + LC( 2 ) + LC( 3 ) );
#endif
    }

    // Schedule transmission of frame
    if( dutyCycleTimeOff == 0 )
    {
        // Try to send now
        return SendFrameOnChannel( Channels[dev->Channel] );
    }
    else
    {
        // Send later - prepare timer
        dev->LoRaMacState |= MAC_TX_DELAYED;
        dev->TxDelayedTimer.msg.type = LORAWAN_TIMER_TX_DELAYED;
        xtimer_set_msg(&(dev->TxDelayedTimer.dev), xtimer_ticks_from_usec(dutyCycleTimeOff*1000).ticks32, &(dev->TxDelayedTimer.msg), dev->TxDelayedTimer.pid);
        //TimerSetValue( &dev->TxDelayedTimer, dutyCycleTimeOff, LORAWAN_TIMER_TX_DELAYED);
        //TimerStart( &dev->TxDelayedTimer, 0);

        return LORAMAC_STATUS_OK;
    }
}

static uint16_t RetransmissionDutyCylce( void )
{
    uint16_t dutyCycle = 0;

#if defined( USE_BAND_868 ) || defined( USE_BAND_433 ) || defined( USE_BAND_780 )
    TimerTime_t timeElapsed = xtimer_now_usec64();

    if( timeElapsed < 3600000 )
    {
        dutyCycle = BACKOFF_DC_1_HOUR;
    }
    else if( timeElapsed < ( 3600000 + 36000000 ) )
    {
        dutyCycle = BACKOFF_DC_10_HOURS;
    }
    else
    {
        dutyCycle = BACKOFF_DC_24_HOURS;
    }
#endif
    return dutyCycle;
}

static void CalculateBackOff( uint8_t channel )
{
    uint16_t dutyCycle = Bands[Channels[channel].Band].DCycle;
    uint16_t joinDutyCycle = 0;
    bool rndTimeOff = false;

    if( dev->lorawan.tx_rx.nwk_status == false )
    {
        joinDutyCycle = RetransmissionDutyCylce( );
        dutyCycle = MAX( dutyCycle, joinDutyCycle );

        // Make sure to not apply the random back-off to the first TX
        if( dev->TxTimeOnAir > 0 )
        {
            rndTimeOff = true;
        }
    }

    // Update Band Time OFF
    if( dev->DutyCycleOn == true )
    {
        Bands[Channels[channel].Band].TimeOff = dev->TxTimeOnAir * dutyCycle - dev->TxTimeOnAir;
    }
    else
    {
        Bands[Channels[channel].Band].TimeOff = 0;
    }

    if( rndTimeOff == true )
    {
        Bands[Channels[channel].Band].TimeOff = random_uint32_range( Bands[Channels[channel].Band].TimeOff,
                                                       Bands[Channels[channel].Band].TimeOff + BACKOFF_RND_OFFSET + 1);
    }

    // Update Aggregated Time OFF
    dev->AggregatedTimeOff = dev->AggregatedTimeOff + ( dev->TxTimeOnAir * dev->AggregatedDCycle - dev->TxTimeOnAir );
}

static int8_t AlternateDatarate( uint16_t nbTrials )
{
    int8_t datarate = LORAMAC_TX_MIN_DATARATE;
#if defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )
#if defined( USE_BAND_915 )
    // Re-enable 500 kHz default channels
    dev->LoRaMacParams.ChannelsMask[4] = 0x00FF;
#else // defined( USE_BAND_915_HYBRID )
    // Re-enable 500 kHz default channels
    //ReenableChannels( dev->LoRaMacParams.ChannelsMask[4], dev->LoRaMacParams.ChannelsMask );
    dev->LoRaMacParams.ChannelsMask[4] = 0x0001;
#endif

    if( ( nbTrials & 0x01 ) == 0x01 )
    {
        datarate = DR_4;
    }
    else
    {
        datarate = DR_1;
    }
#else
    if( ( nbTrials % 48 ) == 0 )
    {
        datarate = DR_0;
    }
    else if( ( nbTrials % 32 ) == 0 )
    {
        datarate = DR_1;
    }
    else if( ( nbTrials % 24 ) == 0 )
    {
        datarate = DR_2;
    }
    else if( ( nbTrials % 16 ) == 0 )
    {
        datarate = DR_3;
    }
    else if( ( nbTrials % 8 ) == 0 )
    {
        datarate = DR_4;
    }
    else
    {
        datarate = DR_5;
    }
#endif
    return datarate;
}

static void ResetMacParameters( void )
{
    dev->lorawan.tx_rx.nwk_status = false;

    // Counters
    dev->uplink_counter = 1;
    dev->DownLinkCounter = 0;
    dev->AdrAckCounter = 0;

    dev->ChannelsNbRepCounter = 0;

    dev->AckTimeoutRetries = 1;
    dev->AckTimeoutRetriesCounter = 1;
    dev->AckTimeoutRetry = false;

    dev->MaxDCycle = 0;
    dev->AggregatedDCycle = 1;

    dev->MacCommandsBufferIndex = 0;
    dev->MacCommandsBufferToRepeatIndex = 0;

    dev->IsRxWindowsEnabled = true;

    dev->LoRaMacParams.ChannelsTxPower = dev->LoRaMacParamsDefaults.ChannelsTxPower;
    dev->LoRaMacParams.ChannelsDatarate = dev->LoRaMacParamsDefaults.ChannelsDatarate;

    dev->LoRaMacParams.MaxRxWindow = dev->LoRaMacParamsDefaults.MaxRxWindow;
    dev->LoRaMacParams.ReceiveDelay1 = dev->LoRaMacParamsDefaults.ReceiveDelay1;
    dev->LoRaMacParams.ReceiveDelay2 = dev->LoRaMacParamsDefaults.ReceiveDelay2;
    dev->LoRaMacParams.JoinAcceptDelay1 = dev->LoRaMacParamsDefaults.JoinAcceptDelay1;
    dev->LoRaMacParams.JoinAcceptDelay2 = dev->LoRaMacParamsDefaults.JoinAcceptDelay2;

    dev->LoRaMacParams.Rx1DrOffset = dev->LoRaMacParamsDefaults.Rx1DrOffset;
    dev->LoRaMacParams.ChannelsNbRep = dev->LoRaMacParamsDefaults.ChannelsNbRep;

    dev->LoRaMacParams.Rx2Channel = dev->LoRaMacParamsDefaults.Rx2Channel;

    memcpy( ( uint8_t* ) dev->LoRaMacParams.ChannelsMask, ( uint8_t* ) dev->LoRaMacParamsDefaults.ChannelsMask, sizeof( dev->LoRaMacParams.ChannelsMask ) );

#if defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )
    memcpy( ( uint8_t* ) dev->ChannelsMaskRemaining, ( uint8_t* ) dev->LoRaMacParamsDefaults.ChannelsMask, sizeof( dev->LoRaMacParams.ChannelsMask ) );
#endif


    dev->NodeAckRequested = false;
    dev->SrvAckRequested = false;
    dev->MacCommandsInNextTx = false;

    // Reset Multicast downlink counters
    MulticastParams_t *cur = dev->MulticastChannels;
    while( cur != NULL )
    {
        cur->DownLinkCounter = 0;
        cur = cur->Next;
    }

    // Initialize channel index.
    dev->Channel = LORA_MAX_NB_CHANNELS;
}

LoRaMacStatus_t PrepareFrame( LoRaMacHeader_t *macHdr, LoRaMacFrameCtrl_t *fCtrl, uint8_t fPort, void *fBuffer, uint16_t fBufferSize )
{
    netdev2_t *netdev = (netdev2_t*) dev;
    uint32_t random;
    uint16_t i;
    uint8_t pktHeaderLen = 0;
    uint32_t mic = 0;
    const void* payload = fBuffer;
    uint8_t payloadSize = fBufferSize;
    uint8_t framePort = fPort;

    dev->LoRaMacBufferPktLen = 0;

    dev->NodeAckRequested = false;

    if( fBuffer == NULL )
    {
        fBufferSize = 0;
    }

    dev->LoRaMacBuffer[pktHeaderLen++] = macHdr->Value;

    switch( macHdr->Bits.MType )
    {
        case FRAME_TYPE_JOIN_REQ:
            dev->RxWindow1Delay = dev->LoRaMacParams.JoinAcceptDelay1 - RADIO_WAKEUP_TIME;
            dev->RxWindow2Delay = dev->LoRaMacParams.JoinAcceptDelay2 - RADIO_WAKEUP_TIME;

            dev->LoRaMacBufferPktLen = pktHeaderLen;

            memcpyr( dev->LoRaMacBuffer + dev->LoRaMacBufferPktLen, dev->app_eui, 8 );
            dev->LoRaMacBufferPktLen += 8;
            memcpyr( dev->LoRaMacBuffer + dev->LoRaMacBufferPktLen, dev->dev_eui, 8 );
            dev->LoRaMacBufferPktLen += 8;

            netdev->driver->get(netdev, NETOPT_LORA_RANDOM, &random, sizeof(uint32_t));
            dev->dev_nonce = random;

            dev->LoRaMacBuffer[dev->LoRaMacBufferPktLen++] = dev->dev_nonce & 0xFF;
            dev->LoRaMacBuffer[dev->LoRaMacBufferPktLen++] = ( dev->dev_nonce >> 8 ) & 0xFF;

            LoRaMacJoinComputeMic( dev->LoRaMacBuffer, dev->LoRaMacBufferPktLen & 0xFF, dev->app_key, &mic );

            dev->LoRaMacBuffer[dev->LoRaMacBufferPktLen++] = mic & 0xFF;
            dev->LoRaMacBuffer[dev->LoRaMacBufferPktLen++] = ( mic >> 8 ) & 0xFF;
            dev->LoRaMacBuffer[dev->LoRaMacBufferPktLen++] = ( mic >> 16 ) & 0xFF;
            dev->LoRaMacBuffer[dev->LoRaMacBufferPktLen++] = ( mic >> 24 ) & 0xFF;

            break;
        case FRAME_TYPE_DATA_CONFIRMED_UP:
            dev->NodeAckRequested = true;
            //Intentional falltrough
        case FRAME_TYPE_DATA_UNCONFIRMED_UP:
            if( dev->lorawan.tx_rx.nwk_status == false )
            {
                return LORAMAC_STATUS_NO_NETWORK_JOINED; // No network has been joined yet
            }

            fCtrl->Bits.AdrAckReq = AdrNextDr( fCtrl->Bits.Adr, true, &dev->LoRaMacParams.ChannelsDatarate );

            if( ValidatePayloadLength( fBufferSize, dev->LoRaMacParams.ChannelsDatarate, dev->MacCommandsBufferIndex ) == false )
            {
                return LORAMAC_STATUS_LENGTH_ERROR;
            }

            dev->RxWindow1Delay = dev->LoRaMacParams.ReceiveDelay1 - RADIO_WAKEUP_TIME;
            dev->RxWindow2Delay = dev->LoRaMacParams.ReceiveDelay2 - RADIO_WAKEUP_TIME;

            if( dev->SrvAckRequested == true )
            {
                dev->SrvAckRequested = false;
                fCtrl->Bits.Ack = 1;
            }

            dev->LoRaMacBuffer[pktHeaderLen++] = ( dev->lorawan.dev_addr ) & 0xFF;
            dev->LoRaMacBuffer[pktHeaderLen++] = ( dev->lorawan.dev_addr >> 8 ) & 0xFF;
            dev->LoRaMacBuffer[pktHeaderLen++] = ( dev->lorawan.dev_addr >> 16 ) & 0xFF;
            dev->LoRaMacBuffer[pktHeaderLen++] = ( dev->lorawan.dev_addr >> 24 ) & 0xFF;

            dev->LoRaMacBuffer[pktHeaderLen++] = fCtrl->Value;

            dev->LoRaMacBuffer[pktHeaderLen++] = dev->uplink_counter & 0xFF;
            dev->LoRaMacBuffer[pktHeaderLen++] = ( dev->uplink_counter >> 8 ) & 0xFF;

            // Copy the MAC commands which must be re-send into the MAC command buffer
            memcpy( &dev->MacCommandsBuffer[dev->MacCommandsBufferIndex], dev->MacCommandsBufferToRepeat, dev->MacCommandsBufferToRepeatIndex );
            dev->MacCommandsBufferIndex += dev->MacCommandsBufferToRepeatIndex;

            if( ( payload != NULL ) && ( payloadSize > 0 ) )
            {
                if( ( dev->MacCommandsBufferIndex <= LORA_MAC_COMMAND_MAX_LENGTH ) && ( dev->MacCommandsInNextTx == true ) )
                {
                    fCtrl->Bits.FOptsLen += dev->MacCommandsBufferIndex;

                    // Update FCtrl field with new value of OptionsLength
                    dev->LoRaMacBuffer[0x05] = fCtrl->Value;
                    for( i = 0; i < dev->MacCommandsBufferIndex; i++ )
                    {
                        dev->LoRaMacBuffer[pktHeaderLen++] = dev->MacCommandsBuffer[i];
                    }
                }
            }
            else
            {
                if( ( dev->MacCommandsBufferIndex > 0 ) && ( dev->MacCommandsInNextTx ) )
                {
                    payloadSize = dev->MacCommandsBufferIndex;
                    payload = dev->MacCommandsBuffer;
                    framePort = 0;
                }
            }
            dev->MacCommandsInNextTx = false;
            // Store MAC commands which must be re-send in case the device does not receive a downlink anymore
            dev->MacCommandsBufferToRepeatIndex = ParseMacCommandsToRepeat( dev->MacCommandsBuffer, dev->MacCommandsBufferIndex, dev->MacCommandsBufferToRepeat );
            if( dev->MacCommandsBufferToRepeatIndex > 0 )
            {
                dev->MacCommandsInNextTx = true;
            }
            dev->MacCommandsBufferIndex = 0;

            if( ( payload != NULL ) && ( payloadSize > 0 ) )
            {
                dev->LoRaMacBuffer[pktHeaderLen++] = framePort;

                if( framePort == 0 )
                {
                     LoRaMacPayloadEncrypt( (uint8_t* ) payload, payloadSize, dev->lorawan.nwk_skey, dev->lorawan.dev_addr, UP_LINK, dev->uplink_counter, dev->LoRaMacPayload );
                }
                else
                {
                     LoRaMacPayloadEncrypt( (uint8_t* ) payload, payloadSize, dev->lorawan.nwk_skey, dev->lorawan.dev_addr, UP_LINK, dev->uplink_counter, dev->LoRaMacPayload );
                }
                memcpy( dev->LoRaMacBuffer + pktHeaderLen,  dev->LoRaMacPayload, payloadSize );
            }
            dev->LoRaMacBufferPktLen = pktHeaderLen + payloadSize;

            LoRaMacComputeMic( dev->LoRaMacBuffer, dev->LoRaMacBufferPktLen, dev->lorawan.nwk_skey, dev->lorawan.dev_addr, UP_LINK, dev->uplink_counter, &mic );

            dev->LoRaMacBuffer[dev->LoRaMacBufferPktLen + 0] = mic & 0xFF;
            dev->LoRaMacBuffer[dev->LoRaMacBufferPktLen + 1] = ( mic >> 8 ) & 0xFF;
            dev->LoRaMacBuffer[dev->LoRaMacBufferPktLen + 2] = ( mic >> 16 ) & 0xFF;
            dev->LoRaMacBuffer[dev->LoRaMacBufferPktLen + 3] = ( mic >> 24 ) & 0xFF;

            dev->LoRaMacBufferPktLen += LORAMAC_MFR_LEN;

            break;
        case FRAME_TYPE_PROPRIETARY:
            if( ( fBuffer != NULL ) && ( fBufferSize > 0 ) )
            {
                memcpy( dev->LoRaMacBuffer + pktHeaderLen, ( uint8_t* ) fBuffer, fBufferSize );
                dev->LoRaMacBufferPktLen = pktHeaderLen + fBufferSize;
            }
            break;
        default:
            return LORAMAC_STATUS_SERVICE_UNKNOWN;
    }

    return LORAMAC_STATUS_OK;
}

void _set_tx_config( RadioModems_t modem, int8_t power, uint32_t fdev,
                        uint32_t bandwidth, uint32_t datarate,
                        uint8_t coderate, uint16_t preambleLen,
                        bool fixLen, bool crcOn, bool freqHopOn,
                        uint8_t hopPeriod, bool iqInverted, uint32_t timeout )
{
    (void) fdev;

    netdev2_t *netdev = (netdev2_t*) dev;

    netopt_enable_t _modem = modem;
    netdev->driver->set(netdev, NETOPT_LORA_MODE, &_modem, sizeof(netopt_enable_t));

    bool freq_hop_on = freqHopOn;
    bool iq_invert = iqInverted;
    uint8_t rx_single = false;
    uint32_t tx_timeout = timeout * 1000;
    uint8_t bw = bandwidth + 7;
    uint8_t cr = coderate;
    uint8_t sf = datarate;
    bool implicit = fixLen;
    bool crc = crcOn;
    uint16_t rx_timeout = 10;
    uint16_t preamble = preambleLen;
    uint8_t payload_len = 0;
    uint8_t hop_period = hopPeriod;

    netdev->driver->set(netdev, NETOPT_LORA_HOP, &freq_hop_on, sizeof(bool));
    netdev->driver->set(netdev, NETOPT_LORA_IQ_INVERT, &iq_invert, sizeof(bool));
    netdev->driver->set(netdev, NETOPT_LORA_SINGLE_RECEIVE, &rx_single, sizeof(uint8_t));
    netdev->driver->set(netdev, NETOPT_LORA_TX_TIMEOUT, &tx_timeout, sizeof(uint32_t));

    netdev->driver->set(netdev, NETOPT_LORA_BANDWIDTH, &bw, sizeof(uint8_t));
    netdev->driver->set(netdev, NETOPT_LORA_CODING_RATE, &cr, sizeof(uint8_t));
    netdev->driver->set(netdev, NETOPT_LORA_SPREADING_FACTOR, &sf, sizeof(uint8_t));
    netdev->driver->set(netdev, NETOPT_LORA_IMPLICIT, &implicit, sizeof(bool));
    netdev->driver->set(netdev, NETOPT_CRC, &crc, sizeof(bool));
    netdev->driver->set(netdev, NETOPT_LORA_SYMBOL_TIMEOUT, &rx_timeout, sizeof(uint16_t));
    netdev->driver->set(netdev, NETOPT_LORA_PREAMBLE_LENGTH, &preamble, sizeof(uint16_t));
    netdev->driver->set(netdev, NETOPT_LORA_PAYLOAD_LENGTH, &payload_len, sizeof(uint8_t));
    netdev->driver->set(netdev, NETOPT_LORA_HOP_PERIOD, &hop_period, sizeof(uint8_t));
    netdev->driver->set(netdev, NETOPT_TX_POWER, &power, sizeof(uint8_t));

}
LoRaMacStatus_t SendFrameOnChannel( ChannelParams_t channel )
{
    netdev2_t *netdev = (netdev2_t*) dev;
    int8_t datarate = Datarates[dev->LoRaMacParams.ChannelsDatarate];
    int8_t txPowerIndex = 0;
    int8_t txPower = 0;
    uint8_t max_payload;

    txPowerIndex = LimitTxPower( dev->LoRaMacParams.ChannelsTxPower );
    txPower = TxPowers[txPowerIndex];

    dev->frame_status = LORAMAC_EVENT_INFO_STATUS_ERROR;
    dev->datarate = dev->LoRaMacParams.ChannelsDatarate;
    dev->tx_power = txPowerIndex;

    netdev->driver->set(netdev, NETOPT_CHANNEL, &channel.Frequency, sizeof(uint32_t));
    netopt_enable_t lm;

    max_payload = dev->LoRaMacBufferPktLen;
    uint8_t pkt_len = (uint8_t) dev->LoRaMacBufferPktLen;
    uint32_t time_on_air;
    netdev->driver->set(netdev, NETOPT_LORA_TIME_ON_AIR, &pkt_len, sizeof(uint8_t));
#if defined( USE_BAND_433 ) || defined( USE_BAND_780 ) || defined( USE_BAND_868 )
    if( dev->LoRaMacParams.ChannelsDatarate == DR_7 )
    { // High Speed FSK channel
        lm = NETOPT_DISABLE;
        netdev->driver->set(netdev, NETOPT_LORA_MODE, &lm, sizeof(netopt_enable_t));
        netdev->driver->set(netdev, NETOPT_LORA_MAX_PAYLOAD, &max_payload, sizeof(uint8_t));
        _set_tx_config( MODEM_FSK, txPower, 25e3, 0, datarate * 1e3, 0, 5, false, true, 0, 0, false, 3e3 );
        netdev->driver->get(netdev, NETOPT_LORA_TIME_ON_AIR, &time_on_air, sizeof(uint32_t));
        dev->TxTimeOnAir = time_on_air;

    }
    else if( dev->LoRaMacParams.ChannelsDatarate == DR_6 )
    { // High speed LoRa channel
        lm = NETOPT_ENABLE;
        netdev->driver->set(netdev, NETOPT_LORA_MODE, &lm, sizeof(netopt_enable_t));
        netdev->driver->set(netdev, NETOPT_LORA_MAX_PAYLOAD, &max_payload, sizeof(uint8_t));
        _set_tx_config( MODEM_LORA, txPower, 0, 1, datarate, 1, 8, false, true, 0, 0, false, 3e3 );
        netdev->driver->get(netdev, NETOPT_LORA_TIME_ON_AIR, &time_on_air, sizeof(uint32_t));
        dev->TxTimeOnAir = time_on_air;
    }
    else
    { // Normal LoRa channel
        lm = NETOPT_ENABLE;
        netdev->driver->set(netdev, NETOPT_LORA_MODE, &lm, sizeof(netopt_enable_t));
        netdev->driver->set(netdev, NETOPT_LORA_MAX_PAYLOAD, &max_payload, sizeof(uint8_t));
        _set_tx_config( MODEM_LORA, txPower, 0, 0, datarate, 1, 8, false, true, 0, 0, false, 3e3 );
        netdev->driver->get(netdev, NETOPT_LORA_TIME_ON_AIR, &time_on_air, sizeof(uint32_t));
        dev->TxTimeOnAir = time_on_air;
    }
#elif defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )
    lm = NETOPT_ENABLE;
    netdev->driver->set(netdev, NETOPT_LORA_MODE, &lm, sizeof(netopt_enable_t));
    netdev->driver->set(netdev, NETOPT_LORA_MAX_PAYLOAD, &max_payload, sizeof(uint8_t));
    if( dev->LoRaMacParams.ChannelsDatarate >= DR_4 )
    { // High speed LoRa channel BW500 kHz
        _set_tx_config( MODEM_LORA, txPower, 0, 2, datarate, 1, 8, false, true, 0, 0, false, 3e3 );
        netdev->driver->get(netdev, NETOPT_LORA_TIME_ON_AIR, &time_on_air, sizeof(uint32_t));
        dev->TxTimeOnAir = time_on_air;
    }
    else
    { // Normal LoRa channel
        _set_tx_config( MODEM_LORA, txPower, 0, 0, datarate, 1, 8, false, true, 0, 0, false, 3e3 );
        netdev->driver->get(netdev, NETOPT_LORA_TIME_ON_AIR, &time_on_air, sizeof(uint32_t));
        dev->TxTimeOnAir = time_on_air;
    }
#else
    #error "Please define a frequency band in the compiler options."
#endif

    // Store the time on air
    //dev->McpsConfirm.TxTimeOnAir = dev->TxTimeOnAir;
    //dev->mlme_confirm.TxTimeOnAir = dev->TxTimeOnAir;

    // Starts the MAC layer status check timer
    dev->MacStateCheckTimer.msg.type = LORAWAN_TIMER_MAC_STATE;
    xtimer_set_msg(&(dev->MacStateCheckTimer.dev), xtimer_ticks_from_usec(MAC_STATE_CHECK_TIMEOUT*1000).ticks32, &(dev->MacStateCheckTimer.msg), dev->MacStateCheckTimer.pid);
    //TimerSetValue( &MacStateCheckTimer, MAC_STATE_CHECK_TIMEOUT, LORAWAN_TIMER_MAC_STATE);
    //TimerStart( &MacStateCheckTimer, 0);

    // Send now
    struct iovec vec[1];
    vec[0].iov_base = dev->LoRaMacBuffer;
    vec[0].iov_len = dev->LoRaMacBufferPktLen;
    netdev->driver->send(netdev, vec, 1); 

    dev->LoRaMacState |= MAC_TX_RUNNING;

    return LORAMAC_STATUS_OK;
}

LoRaMacStatus_t LoRaMacInitialization( LoRaMacPrimitives_t *primitives, kernel_pid_t mac_pid)
{
    if( primitives == NULL )
    {
        return LORAMAC_STATUS_PARAMETER_INVALID;
    }

    if( ( primitives->MacMcpsConfirm == NULL ) ||
        ( primitives->MacMcpsIndication == NULL ) ||
        ( primitives->MacMlmeConfirm == NULL ))
    {
        return LORAMAC_STATUS_PARAMETER_INVALID;
    }

    dev->MulticastChannels = NULL;
    dev->LoRaMacBufferPktLen = 0;
    dev->uplink_counter = 0;
    dev->DownLinkCounter = 0;
    dev->IsUpLinkCounterFixed = false;
    dev->IsRxWindowsEnabled = true;
    dev->AdrAckCounter = 0;
    dev->NodeAckRequested = false;
    dev->SrvAckRequested = false;
    dev->MacCommandsInNextTx = false;
    dev->MacCommandsBufferIndex = 0;
    dev->MacCommandsBufferToRepeatIndex = 0;
    dev->ChannelsNbRepCounter = 0;
    dev->MaxDCycle = 0;
    dev->LoRaMacState = MAC_IDLE;
    dev->AckTimeoutRetries = 1;
    dev->AckTimeoutRetriesCounter = 1;
    dev->AckTimeoutRetry = false;
    dev->TxTimeOnAir = 0;
    dev->RxSlot = 0;
    dev->LoRaMacPrimitives = primitives;

    dev->LoRaMacFlags.Value = 0;

    dev->LoRaMacDeviceClass = CLASS_A;
    dev->LoRaMacState = MAC_IDLE;

    dev->JoinRequestTrials = 0;
    dev->RepeaterSupport = false;

    // Reset duty cycle times
    dev->AggregatedLastTxDoneTime = 0;
    dev->AggregatedTimeOff = 0;

    // Duty cycle
#if defined( USE_BAND_433 )
    dev->DutyCycleOn = false;
#elif defined( USE_BAND_780 )
    dev->DutyCycleOn = false;
#elif defined( USE_BAND_868 )
    dev->DutyCycleOn = true;
#elif defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )
    dev->DutyCycleOn = false;
#else
    #error "Please define a frequency band in the compiler options."
#endif

    // Reset to defaults
    dev->LoRaMacParamsDefaults.ChannelsTxPower = LORAMAC_DEFAULT_TX_POWER;
    dev->LoRaMacParamsDefaults.ChannelsDatarate = LORAMAC_DEFAULT_DATARATE;

    dev->LoRaMacParamsDefaults.MaxRxWindow = MAX_RX_WINDOW;
    dev->LoRaMacParamsDefaults.ReceiveDelay1 = RECEIVE_DELAY1;
    dev->LoRaMacParamsDefaults.ReceiveDelay2 = RECEIVE_DELAY2;
    dev->LoRaMacParamsDefaults.JoinAcceptDelay1 = JOIN_ACCEPT_DELAY1;
    dev->LoRaMacParamsDefaults.JoinAcceptDelay2 = JOIN_ACCEPT_DELAY2;

    dev->LoRaMacParamsDefaults.ChannelsNbRep = 1;
    dev->LoRaMacParamsDefaults.Rx1DrOffset = 0;

    dev->LoRaMacParamsDefaults.Rx2Channel = ( Rx2ChannelParams_t )RX_WND_2_CHANNEL;

    // Channel mask
#if defined( USE_BAND_433 )
    dev->LoRaMacParamsDefaults.ChannelsMask[0] = LC( 1 ) + LC( 2 ) + LC( 3 );
#elif defined( USE_BAND_780 )
    dev->LoRaMacParamsDefaults.ChannelsMask[0] = LC( 1 ) + LC( 2 ) + LC( 3 );
#elif defined( USE_BAND_868 )
    dev->LoRaMacParamsDefaults.ChannelsMask[0] = LC( 1 ) + LC( 2 ) + LC( 3 );
#elif defined( USE_BAND_915 )
    dev->LoRaMacParamsDefaults.ChannelsMask[0] = 0xFFFF;
    dev->LoRaMacParamsDefaults.ChannelsMask[1] = 0xFFFF;
    dev->LoRaMacParamsDefaults.ChannelsMask[2] = 0xFFFF;
    dev->LoRaMacParamsDefaults.ChannelsMask[3] = 0xFFFF;
    dev->LoRaMacParamsDefaults.ChannelsMask[4] = 0x00FF;
    dev->LoRaMacParamsDefaults.ChannelsMask[5] = 0x0000;
#elif defined( USE_BAND_915_HYBRID )
    //ReenableChannels( dev->LoRaMacParams.ChannelsMask[4], dev->LoRaMacParams.ChannelsMask );
    dev->LoRaMacParamsDefaults.ChannelsMask[0] = 0x3C3C;
    dev->LoRaMacParamsDefaults.ChannelsMask[1] = 0x0000;
    dev->LoRaMacParamsDefaults.ChannelsMask[2] = 0x0000;
    dev->LoRaMacParamsDefaults.ChannelsMask[3] = 0x0000;
    dev->LoRaMacParamsDefaults.ChannelsMask[4] = 0x0001;
    dev->LoRaMacParamsDefaults.ChannelsMask[5] = 0x0000;
#else
    #error "Please define a frequency band in the compiler options."
#endif

#if defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )
    // 125 kHz channels
    for( uint8_t i = 0; i < LORA_MAX_NB_CHANNELS - 8; i++ )
    {
        Channels[i].Frequency = 902.3e6 + i * 200e3;
        Channels[i].DrRange.Value = ( DR_3 << 4 ) | DR_0;
        Channels[i].Band = 0;
    }
    // 500 kHz channels
    for( uint8_t i = LORA_MAX_NB_CHANNELS - 8; i < LORA_MAX_NB_CHANNELS; i++ )
    {
        Channels[i].Frequency = 903.0e6 + ( i - ( LORA_MAX_NB_CHANNELS - 8 ) ) * 1.6e6;
        Channels[i].DrRange.Value = ( DR_4 << 4 ) | DR_4;
        Channels[i].Band = 0;
    }
#endif

    ResetMacParameters( );

    // Initialize timers
    //TimerInit( &MacStateCheckTimer, OnMacStateCheckTimerEvent, mac_pid );
    dev->MacStateCheckTimer.dev.target = 0;
    dev->MacStateCheckTimer.dev.callback =  (void*) OnMacStateCheckTimerEvent;
    dev->MacStateCheckTimer.pid = mac_pid;
    //TimerSetValue( &MacStateCheckTimer, MAC_STATE_CHECK_TIMEOUT, LORAWAN_TIMER_MAC_STATE);

    //TimerInit( &dev->TxDelayedTimer, OnTxDelayedTimerEvent, mac_pid );
    dev->TxDelayedTimer.dev.target = 0;
    dev->TxDelayedTimer.dev.callback =  (void*) OnTxDelayedTimerEvent;
    dev->TxDelayedTimer.pid =  mac_pid;

    //TimerInit( &dev->RxWindowTimer1, OnRxWindow1TimerEvent, mac_pid );
    dev->RxWindowTimer1.dev.target = 0;
    dev->RxWindowTimer1.dev.callback =  (void*) OnRxWindow1TimerEvent;
    dev->RxWindowTimer1.pid = mac_pid;

    //TimerInit( &RxWindowTimer2, OnRxWindow2TimerEvent, mac_pid );
    dev->RxWindowTimer2.dev.target = 0;
    dev->RxWindowTimer2.dev.callback =  (void*) OnRxWindow2TimerEvent;
    dev->RxWindowTimer2.pid = mac_pid;

    //TimerInit( &AckTimeoutTimer, OnAckTimeoutTimerEvent, mac_pid );
    dev->AckTimeoutTimer.dev.target = 0;
    dev->AckTimeoutTimer.dev.callback =  (void*) OnAckTimeoutTimerEvent;
    dev->AckTimeoutTimer.pid = mac_pid;


    // Random seed initialization

    netdev2_t *netdev = (netdev2_t*) dev;
    uint32_t random;
    netdev->driver->get(netdev, NETOPT_LORA_RANDOM, &random, sizeof(uint32_t));
    random_init(random);

    dev->lorawan.public = true;
    SetPublicNetwork( dev->lorawan.public );
    netopt_state_t state = NETOPT_STATE_SLEEP;
    netdev->driver->set(netdev, NETOPT_STATE, &state, sizeof(netopt_state_t));

    return LORAMAC_STATUS_OK;
}

LoRaMacStatus_t LoRaMacQueryTxPossible( uint8_t size, LoRaMacTxInfo_t* txInfo )
{
    int8_t datarate = dev->LoRaMacParamsDefaults.ChannelsDatarate;
    uint8_t fOptLen = dev->MacCommandsBufferIndex + dev->MacCommandsBufferToRepeatIndex;

    if( txInfo == NULL )
    {
        return LORAMAC_STATUS_PARAMETER_INVALID;
    }

    AdrNextDr( dev->lorawan.tx_rx.adr_ctrl, false, &datarate );

    if( dev->RepeaterSupport == true )
    {
        txInfo->CurrentPayloadSize = MaxPayloadOfDatarateRepeater[datarate];
    }
    else
    {
        txInfo->CurrentPayloadSize = MaxPayloadOfDatarate[datarate];
    }

    if( txInfo->CurrentPayloadSize >= fOptLen )
    {
        txInfo->MaxPossiblePayload = txInfo->CurrentPayloadSize - fOptLen;
    }
    else
    {
        return LORAMAC_STATUS_MAC_CMD_LENGTH_ERROR;
    }

    if( ValidatePayloadLength( size, datarate, 0 ) == false )
    {
        return LORAMAC_STATUS_LENGTH_ERROR;
    }

    if( ValidatePayloadLength( size, datarate, fOptLen ) == false )
    {
        return LORAMAC_STATUS_MAC_CMD_LENGTH_ERROR;
    }

    return LORAMAC_STATUS_OK;
}

LoRaMacStatus_t LoRaMacMibGetRequestConfirm( MibRequestConfirm_t *mibGet )
{
    LoRaMacStatus_t status = LORAMAC_STATUS_OK;

    if( mibGet == NULL )
    {
        return LORAMAC_STATUS_PARAMETER_INVALID;
    }

    switch( mibGet->Type )
    {
        case MIB_NETWORK_JOINED:
        {
            mibGet->Param.IsNetworkJoined = dev->lorawan.tx_rx.nwk_status;
            break;
        }
        case MIB_ADR:
        {
            mibGet->Param.AdrEnable = dev->lorawan.tx_rx.adr_ctrl;
            break;
        }
        case MIB_REPEATER_SUPPORT:
        {
            mibGet->Param.EnableRepeaterSupport = dev->RepeaterSupport;
            break;
        }
        case MIB_CHANNELS:
        {
            mibGet->Param.ChannelList = Channels;
            break;
        }
        case MIB_RX2_CHANNEL:
        {
            mibGet->Param.Rx2Channel = dev->LoRaMacParams.Rx2Channel;
            break;
        }
        case MIB_CHANNELS_MASK:
        {
            mibGet->Param.ChannelsMask = dev->LoRaMacParams.ChannelsMask;
            break;
        }
        case MIB_CHANNELS_NB_REP:
        {
            mibGet->Param.ChannelNbRep = dev->LoRaMacParams.ChannelsNbRep;
            break;
        }
        case MIB_MAX_RX_WINDOW_DURATION:
        {
            mibGet->Param.MaxRxWindow = dev->LoRaMacParams.MaxRxWindow;
            break;
        }
        case MIB_RECEIVE_DELAY_1:
        {
            mibGet->Param.ReceiveDelay1 = dev->LoRaMacParams.ReceiveDelay1;
            break;
        }
        case MIB_RECEIVE_DELAY_2:
        {
            mibGet->Param.ReceiveDelay2 = dev->LoRaMacParams.ReceiveDelay2;
            break;
        }
        case MIB_JOIN_ACCEPT_DELAY_1:
        {
            mibGet->Param.JoinAcceptDelay1 = dev->LoRaMacParams.JoinAcceptDelay1;
            break;
        }
        case MIB_JOIN_ACCEPT_DELAY_2:
        {
            mibGet->Param.JoinAcceptDelay2 = dev->LoRaMacParams.JoinAcceptDelay2;
            break;
        }
        case MIB_CHANNELS_DEFAULT_DATARATE:
        {
            mibGet->Param.ChannelsDefaultDatarate = dev->LoRaMacParamsDefaults.ChannelsDatarate;
            break;
        }
        case MIB_CHANNELS_DATARATE:
        {
            mibGet->Param.ChannelsDatarate = dev->LoRaMacParams.ChannelsDatarate;
            break;
        }
        case MIB_CHANNELS_TX_POWER:
        {
            mibGet->Param.ChannelsTxPower = dev->LoRaMacParams.ChannelsTxPower;
            break;
        }
        case MIB_UPLINK_COUNTER:
        {
            mibGet->Param.UpLinkCounter = dev->uplink_counter;
            break;
        }
        case MIB_DOWNLINK_COUNTER:
        {
            mibGet->Param.DownLinkCounter = dev->DownLinkCounter;
            break;
        }
        case MIB_MULTICAST_CHANNEL:
        {
            mibGet->Param.MulticastList = dev->MulticastChannels;
            break;
        }
        default:
            status = LORAMAC_STATUS_SERVICE_UNKNOWN;
            break;
    }

    return status;
}

void lorawan_set_class(netdev2_lorawan_t *dev, int class)
{
    netdev2_t *netdev = (netdev2_t*) dev;
    netopt_state_t state = NETOPT_STATE_SLEEP;
    dev->LoRaMacDeviceClass = class;
    switch(class)
    {
        case CLASS_A:
        {
            // Set the radio into sleep to setup a defined state
            netdev->driver->set(netdev, NETOPT_STATE, &state, sizeof(netopt_state_t));
            break;
        }
        case CLASS_B:
        {
            break;
        }
        case CLASS_C:
        {
            // Set the dev->NodeAckRequested indicator to default
            dev->NodeAckRequested = false;
            /* Currently there's no netdev pointer here... it should be fixed later. Let's put an assert for now */
            assert(false);
            //OnRxWindow2TimerEvent(netdev);
            break;
        }
    }
}
LoRaMacStatus_t LoRaMacMibSetRequestConfirm( MibRequestConfirm_t *mibSet )
{
    LoRaMacStatus_t status = LORAMAC_STATUS_OK;

    if( mibSet == NULL )
    {
        return LORAMAC_STATUS_PARAMETER_INVALID;
    }
    if( ( dev->LoRaMacState & MAC_TX_RUNNING ) == MAC_TX_RUNNING )
    {
        return LORAMAC_STATUS_BUSY;
    }

    switch( mibSet->Type )
    {
        case MIB_NETWORK_JOINED:
        {
            dev->lorawan.tx_rx.nwk_status = mibSet->Param.IsNetworkJoined;
            break;
        }
        case MIB_ADR:
        {
            dev->lorawan.tx_rx.adr_ctrl = mibSet->Param.AdrEnable;
            break;
        }
        case MIB_REPEATER_SUPPORT:
        {
             dev->RepeaterSupport = mibSet->Param.EnableRepeaterSupport;
            break;
        }
        case MIB_RX2_CHANNEL:
        {
            dev->LoRaMacParams.Rx2Channel = mibSet->Param.Rx2Channel;
            break;
        }
        case MIB_CHANNELS_MASK:
        {
            if( mibSet->Param.ChannelsMask )
            {
#if defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )
                bool chanMaskState = true;

#if defined( USE_BAND_915_HYBRID )
                chanMaskState = ValidateChannelMask( mibSet->Param.ChannelsMask );
#endif
                if( chanMaskState == true )
                {
                    if( ( CountNbEnabled125kHzChannels( mibSet->Param.ChannelsMask ) < 6 ) &&
                        ( CountNbEnabled125kHzChannels( mibSet->Param.ChannelsMask ) > 0 ) )
                    {
                        status = LORAMAC_STATUS_PARAMETER_INVALID;
                    }
                    else
                    {
                        memcpy( ( uint8_t* ) dev->LoRaMacParams.ChannelsMask,
                                 ( uint8_t* ) mibSet->Param.ChannelsMask, sizeof( dev->LoRaMacParams.ChannelsMask ) );
                        for ( uint8_t i = 0; i < sizeof( dev->LoRaMacParams.ChannelsMask ) / 2; i++ )
                        {
                            // Disable channels which are no longer available
                            dev->ChannelsMaskRemaining[i] &= dev->LoRaMacParams.ChannelsMask[i];
                        }
                    }
                }
                else
                {
                    status = LORAMAC_STATUS_PARAMETER_INVALID;
                }
#else
                memcpy( ( uint8_t* ) dev->LoRaMacParams.ChannelsMask,
                         ( uint8_t* ) mibSet->Param.ChannelsMask, 2 );
#endif
            }
            else
            {
                status = LORAMAC_STATUS_PARAMETER_INVALID;
            }
            break;
        }
        case MIB_CHANNELS_NB_REP:
        {
            if( ( mibSet->Param.ChannelNbRep >= 1 ) &&
                ( mibSet->Param.ChannelNbRep <= 15 ) )
            {
                dev->LoRaMacParams.ChannelsNbRep = mibSet->Param.ChannelNbRep;
            }
            else
            {
                status = LORAMAC_STATUS_PARAMETER_INVALID;
            }
            break;
        }
        case MIB_MAX_RX_WINDOW_DURATION:
        {
            dev->LoRaMacParams.MaxRxWindow = mibSet->Param.MaxRxWindow;
            break;
        }
        case MIB_RECEIVE_DELAY_1:
        {
            dev->LoRaMacParams.ReceiveDelay1 = mibSet->Param.ReceiveDelay1;
            break;
        }
        case MIB_RECEIVE_DELAY_2:
        {
            dev->LoRaMacParams.ReceiveDelay2 = mibSet->Param.ReceiveDelay2;
            break;
        }
        case MIB_JOIN_ACCEPT_DELAY_1:
        {
            dev->LoRaMacParams.JoinAcceptDelay1 = mibSet->Param.JoinAcceptDelay1;
            break;
        }
        case MIB_JOIN_ACCEPT_DELAY_2:
        {
            dev->LoRaMacParams.JoinAcceptDelay2 = mibSet->Param.JoinAcceptDelay2;
            break;
        }
        case MIB_CHANNELS_DEFAULT_DATARATE:
        {
            if( ValueInRange( mibSet->Param.ChannelsDefaultDatarate,
                              LORAMAC_TX_MIN_DATARATE, LORAMAC_TX_MAX_DATARATE ) )
            {
                dev->LoRaMacParamsDefaults.ChannelsDatarate = mibSet->Param.ChannelsDefaultDatarate;
            }
            else
            {
                status = LORAMAC_STATUS_PARAMETER_INVALID;
            }
            break;
        }
        case MIB_CHANNELS_DATARATE:
        {
            if( ValueInRange( mibSet->Param.ChannelsDatarate,
                              LORAMAC_TX_MIN_DATARATE, LORAMAC_TX_MAX_DATARATE ) )
            {
                dev->LoRaMacParams.ChannelsDatarate = mibSet->Param.ChannelsDatarate;
            }
            else
            {
                status = LORAMAC_STATUS_PARAMETER_INVALID;
            }
            break;
        }
        case MIB_CHANNELS_TX_POWER:
        {
            if( ValueInRange( mibSet->Param.ChannelsTxPower,
                              LORAMAC_MAX_TX_POWER, LORAMAC_MIN_TX_POWER ) )
            {
                dev->LoRaMacParams.ChannelsTxPower = mibSet->Param.ChannelsTxPower;
            }
            else
            {
                status = LORAMAC_STATUS_PARAMETER_INVALID;
            }
            break;
        }
        case MIB_UPLINK_COUNTER:
        {
            dev->uplink_counter = mibSet->Param.UpLinkCounter;
            break;
        }
        case MIB_DOWNLINK_COUNTER:
        {
            dev->DownLinkCounter = mibSet->Param.DownLinkCounter;
            break;
        }
        default:
            status = LORAMAC_STATUS_SERVICE_UNKNOWN;
            break;
    }

    return status;
}

LoRaMacStatus_t LoRaMacChannelAdd( uint8_t id, ChannelParams_t params )
{
#if ( defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID ) )
    return LORAMAC_STATUS_PARAMETER_INVALID;
#else
    bool datarateInvalid = false;
    bool frequencyInvalid = false;
    uint8_t band = 0;

    // The id must not exceed LORA_MAX_NB_CHANNELS
    if( id >= LORA_MAX_NB_CHANNELS )
    {
        return LORAMAC_STATUS_PARAMETER_INVALID;
    }
    // Validate if the MAC is in a correct state
    if( ( dev->LoRaMacState & MAC_TX_RUNNING ) == MAC_TX_RUNNING )
    {
        if( ( dev->LoRaMacState & MAC_TX_CONFIG ) != MAC_TX_CONFIG )
        {
            return LORAMAC_STATUS_BUSY;
        }
    }
    // Validate the datarate
    if( ( params.DrRange.Fields.Min > params.DrRange.Fields.Max ) ||
        ( ValueInRange( params.DrRange.Fields.Min, LORAMAC_TX_MIN_DATARATE,
                        LORAMAC_TX_MAX_DATARATE ) == false ) ||
        ( ValueInRange( params.DrRange.Fields.Max, LORAMAC_TX_MIN_DATARATE,
                        LORAMAC_TX_MAX_DATARATE ) == false ) )
    {
        datarateInvalid = true;
    }

#if defined( USE_BAND_433 ) || defined( USE_BAND_780 ) || defined( USE_BAND_868 )
    if( id < 3 )
    {
        if( params.Frequency != Channels[id].Frequency )
        {
            frequencyInvalid = true;
        }

        if( params.DrRange.Fields.Min > dev->LoRaMacParamsDefaults.ChannelsDatarate )
        {
            datarateInvalid = true;
        }
        if( ValueInRange( params.DrRange.Fields.Max, DR_5, LORAMAC_TX_MAX_DATARATE ) == false )
        {
            datarateInvalid = true;
        }
    }
#endif

    // Validate the frequency
    if( ( check_rf_freq( params.Frequency ) == true ) && ( params.Frequency > 0 ) && ( frequencyInvalid == false ) )
    {
#if defined( USE_BAND_868 )
        if( ( params.Frequency >= 865000000 ) && ( params.Frequency <= 868000000 ) )
        {
            band = BAND_G1_0;
        }
        else if( ( params.Frequency > 868000000 ) && ( params.Frequency <= 868600000 ) )
        {
            band = BAND_G1_1;
        }
        else if( ( params.Frequency >= 868700000 ) && ( params.Frequency <= 869200000 ) )
        {
            band = BAND_G1_2;
        }
        else if( ( params.Frequency >= 869400000 ) && ( params.Frequency <= 869650000 ) )
        {
            band = BAND_G1_3;
        }
        else if( ( params.Frequency >= 869700000 ) && ( params.Frequency <= 870000000 ) )
        {
            band = BAND_G1_4;
        }
        else
        {
            frequencyInvalid = true;
        }
#endif
    }
    else
    {
        frequencyInvalid = true;
    }

    if( ( datarateInvalid == true ) && ( frequencyInvalid == true ) )
    {
        return LORAMAC_STATUS_FREQ_AND_DR_INVALID;
    }
    if( datarateInvalid == true )
    {
        return LORAMAC_STATUS_DATARATE_INVALID;
    }
    if( frequencyInvalid == true )
    {
        return LORAMAC_STATUS_FREQUENCY_INVALID;
    }

    // Every parameter is valid, activate the channel
    Channels[id] = params;
    Channels[id].Band = band;
    dev->LoRaMacParams.ChannelsMask[0] |= ( 1 << id );

    return LORAMAC_STATUS_OK;
#endif
}

LoRaMacStatus_t LoRaMacChannelRemove( uint8_t id )
{
#if defined( USE_BAND_433 ) || defined( USE_BAND_780 ) || defined( USE_BAND_868 )
    if( ( dev->LoRaMacState & MAC_TX_RUNNING ) == MAC_TX_RUNNING )
    {
        if( ( dev->LoRaMacState & MAC_TX_CONFIG ) != MAC_TX_CONFIG )
        {
            return LORAMAC_STATUS_BUSY;
        }
    }

    if( ( id < 3 ) || ( id >= LORA_MAX_NB_CHANNELS ) )
    {
        return LORAMAC_STATUS_PARAMETER_INVALID;
    }
    else
    {
        // Remove the channel from the list of channels
        Channels[id] = ( ChannelParams_t ){ 0, { 0 }, 0 };

        // Disable the channel as it doesn't exist anymore
        if( DisableChannelInMask( id, dev->LoRaMacParams.ChannelsMask ) == false )
        {
            return LORAMAC_STATUS_PARAMETER_INVALID;
        }
    }
    return LORAMAC_STATUS_OK;
#elif ( defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID ) )
    return LORAMAC_STATUS_PARAMETER_INVALID;
#endif
}

LoRaMacStatus_t LoRaMacMulticastChannelLink( MulticastParams_t *channelParam )
{
    if( channelParam == NULL )
    {
        return LORAMAC_STATUS_PARAMETER_INVALID;
    }
    if( ( dev->LoRaMacState & MAC_TX_RUNNING ) == MAC_TX_RUNNING )
    {
        return LORAMAC_STATUS_BUSY;
    }

    // Reset downlink counter
    channelParam->DownLinkCounter = 0;

    if( dev->MulticastChannels == NULL )
    {
        // New node is the fist element
        dev->MulticastChannels = channelParam;
    }
    else
    {
        MulticastParams_t *cur = dev->MulticastChannels;

        // Search the last node in the list
        while( cur->Next != NULL )
        {
            cur = cur->Next;
        }
        // This function always finds the last node
        cur->Next = channelParam;
    }

    return LORAMAC_STATUS_OK;
}

LoRaMacStatus_t LoRaMacMulticastChannelUnlink( MulticastParams_t *channelParam )
{
    if( channelParam == NULL )
    {
        return LORAMAC_STATUS_PARAMETER_INVALID;
    }
    if( ( dev->LoRaMacState & MAC_TX_RUNNING ) == MAC_TX_RUNNING )
    {
        return LORAMAC_STATUS_BUSY;
    }

    if( dev->MulticastChannels != NULL )
    {
        if( dev->MulticastChannels == channelParam )
        {
          // First element
          dev->MulticastChannels = channelParam->Next;
        }
        else
        {
            MulticastParams_t *cur = dev->MulticastChannels;

            // Search the node in the list
            while( cur->Next && cur->Next != channelParam )
            {
                cur = cur->Next;
            }
            // If we found the node, remove it
            if( cur->Next )
            {
                cur->Next = channelParam->Next;
            }
        }
        channelParam->Next = NULL;
    }

    return LORAMAC_STATUS_OK;
}

LoRaMacStatus_t join_request(void)
{
    LoRaMacStatus_t status = LORAMAC_STATUS_SERVICE_UNKNOWN;
    LoRaMacHeader_t macHdr;

    if( ( dev->LoRaMacState & MAC_TX_RUNNING ) == MAC_TX_RUNNING )
    {
        return LORAMAC_STATUS_BUSY;
    }

    dev->frame_status = LORAMAC_EVENT_INFO_STATUS_ERROR;

    if( ( dev->LoRaMacState & MAC_TX_DELAYED ) == MAC_TX_DELAYED )
    {
        return LORAMAC_STATUS_BUSY;
    }

    //dev->mlme_confirm.MlmeRequest = MLME_JOIN;
    dev->last_frame = FRAME_TYPE_JOIN_REQ;
    dev->LoRaMacFlags.Bits.MlmeReq = 1;

    macHdr.Value = 0;
    macHdr.Bits.MType  = FRAME_TYPE_JOIN_REQ;

    ResetMacParameters( );

    dev->JoinRequestTrials++;
    dev->LoRaMacParams.ChannelsDatarate = AlternateDatarate( dev->JoinRequestTrials );

    status = Send( &macHdr, 0, NULL, 0 );

    if( status != LORAMAC_STATUS_OK )
    {
        dev->NodeAckRequested = false;
        dev->LoRaMacFlags.Bits.MlmeReq = 0;
    }

    return status;
}

LoRaMacStatus_t link_check(void)
{
    LoRaMacStatus_t status = LORAMAC_STATUS_SERVICE_UNKNOWN;

    if( ( dev->LoRaMacState & MAC_TX_RUNNING ) == MAC_TX_RUNNING )
    {
        return LORAMAC_STATUS_BUSY;
    }

    dev->frame_status = LORAMAC_EVENT_INFO_STATUS_ERROR;

    dev->LoRaMacFlags.Bits.MlmeReq = 1;
    // LoRaMac will send this command piggy-pack
    //dev->mlme_confirm.MlmeRequest = MLME_LINK_CHECK;
    dev->last_command = MOTE_MAC_LINK_CHECK_REQ;

    status = AddMacCommand( MOTE_MAC_LINK_CHECK_REQ, 0, 0 );

    if( status != LORAMAC_STATUS_OK )
    {
        dev->NodeAckRequested = false;
        dev->LoRaMacFlags.Bits.MlmeReq = 0;
    }

    return status;
}

LoRaMacStatus_t LoRaMacMcpsRequest( McpsReq_t *mcpsRequest )
{
    LoRaMacStatus_t status = LORAMAC_STATUS_SERVICE_UNKNOWN;
    LoRaMacHeader_t macHdr;
    uint8_t fPort = 0;
    void *fBuffer;
    uint16_t fBufferSize;
    int8_t datarate;
    bool readyToSend = false;

    if( mcpsRequest == NULL )
    {
        return LORAMAC_STATUS_PARAMETER_INVALID;
    }
    if( ( ( dev->LoRaMacState & MAC_TX_RUNNING ) == MAC_TX_RUNNING ) ||
        ( ( dev->LoRaMacState & MAC_TX_DELAYED ) == MAC_TX_DELAYED ) )
    {
        return LORAMAC_STATUS_BUSY;
    }

    macHdr.Value = 0;
    dev->frame_status = LORAMAC_EVENT_INFO_STATUS_ERROR;

    switch( mcpsRequest->Type )
    {
        case MCPS_UNCONFIRMED:
        {
            readyToSend = true;
            dev->AckTimeoutRetries = 1;

            macHdr.Bits.MType = FRAME_TYPE_DATA_UNCONFIRMED_UP;
            fPort = mcpsRequest->Req.Unconfirmed.fPort;
            fBuffer = mcpsRequest->Req.Unconfirmed.fBuffer;
            fBufferSize = mcpsRequest->Req.Unconfirmed.fBufferSize;
            datarate = mcpsRequest->Req.Unconfirmed.Datarate;
            break;
        }
        case MCPS_CONFIRMED:
        {
            readyToSend = true;
            dev->AckTimeoutRetriesCounter = 1;
            dev->AckTimeoutRetries = mcpsRequest->Req.Confirmed.NbTrials;

            macHdr.Bits.MType = FRAME_TYPE_DATA_CONFIRMED_UP;
            fPort = mcpsRequest->Req.Confirmed.fPort;
            fBuffer = mcpsRequest->Req.Confirmed.fBuffer;
            fBufferSize = mcpsRequest->Req.Confirmed.fBufferSize;
            datarate = mcpsRequest->Req.Confirmed.Datarate;
            break;
        }
        case MCPS_PROPRIETARY:
        {
            readyToSend = true;
            dev->AckTimeoutRetries = 1;

            macHdr.Bits.MType = FRAME_TYPE_PROPRIETARY;
            fBuffer = mcpsRequest->Req.Proprietary.fBuffer;
            fBufferSize = mcpsRequest->Req.Proprietary.fBufferSize;
            datarate = mcpsRequest->Req.Proprietary.Datarate;
            break;
        }
        default:
            break;
    }

    if( readyToSend == true )
    {
        if( dev->lorawan.tx_rx.adr_ctrl == false )
        {
            if( ValueInRange( datarate, LORAMAC_TX_MIN_DATARATE, LORAMAC_TX_MAX_DATARATE ) == true )
            {
                dev->LoRaMacParams.ChannelsDatarate = datarate;
            }
            else
            {
                return LORAMAC_STATUS_PARAMETER_INVALID;
            }
        }

        status = Send( &macHdr, fPort, fBuffer, fBufferSize );
        if( status == LORAMAC_STATUS_OK )
        {
            //dev->McpsConfirm.McpsRequest = mcpsRequest->Type;
            dev->LoRaMacFlags.Bits.McpsReq = 1;
        }
        else
        {
            dev->NodeAckRequested = false;
        }
    }

    return status;
}

void LoRaMacTestRxWindowsOn( bool enable )
{
    dev->IsRxWindowsEnabled = enable;
}

void LoRaMacTestSetMic( uint16_t txPacketCounter )
{
    dev->uplink_counter = txPacketCounter;
    dev->IsUpLinkCounterFixed = true;
}

