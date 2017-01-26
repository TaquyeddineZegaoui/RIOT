#include "net/lorawan/board_definitions.h"
#include "sx1276.h"

/*!
 * \ Sx1276 driver for use of LoRaMac
 */

static sx1276_t* dev_ptr;

/*
 * Radio driver functions implementation wrappers, the netdev2 object
 * is known within the scope of the function
 */

sx1276_t* radio_get_ptr(void)
{
    return dev_ptr;
}

void radio_set_ptr(sx1276_t* ptr)
{
    dev_ptr = ptr;
}

void SX1276Init( RadioEvents_t *events )
{
    assert(false);
    //RadioEvents = events;
    //TODO: The final code is not using this function for driver init
    //sx1276_init(dev_ptr);
}

RadioState_t SX1276GetStatus( void )
{
    assert(false);
    return 0;
    //return (RadioState_t) sx1276_get_status(dev_ptr);
}

void SX1276SetModem( RadioModems_t modem)
{
    assert(false);
    //sx1276_set_modem(dev_ptr, (sx1276_radio_modems_t) modem);
}

void SX1276SetChannel( uint32_t freq )
{
    assert(false);
    //sx1276_set_channel(dev_ptr, freq);
}

bool SX1276IsChannelFree( RadioModems_t modem, uint32_t freq, int16_t rssiThresh )
{
    assert(false);
    //return sx1276_is_channel_free(dev_ptr, freq, rssiThresh);
}

uint32_t SX1276Random( void )
{
     assert(false);
     return sx1276_random(dev_ptr);
}

void SX1276SetRxConfig( RadioModems_t modem, uint32_t bandwidth,
                         uint32_t datarate, uint8_t coderate,
                         uint32_t bandwidthAfc, uint16_t preambleLen,
                         uint16_t symbTimeout, bool fixLen,
                         uint8_t payloadLen,
                         bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                         bool iqInverted, bool rxContinuous )
{
    assert(false);
}

void SX1276SetTxConfig( RadioModems_t modem, int8_t power, uint32_t fdev,
                        uint32_t bandwidth, uint32_t datarate,
                        uint8_t coderate, uint16_t preambleLen,
                        bool fixLen, bool crcOn, bool freqHopOn,
                        uint8_t hopPeriod, bool iqInverted, uint32_t timeout )
{
    assert(false);
}

uint32_t SX1276GetTimeOnAir( RadioModems_t modem, uint8_t pktLen )
{
    assert(false);
    return 0;
    //return sx1276_get_time_on_air(dev_ptr, (sx1276_radio_modems_t) modem,
     //                           pktLen);
}

void SX1276Send( uint8_t *buffer, uint8_t size )
{
    assert(false);
}

void SX1276SetSleep( void )
{
    assert(false);
}

void SX1276SetStby( void )
{
    assert(false);
}

void SX1276SetRx( uint32_t timeout )
{
    assert(false);
}


void SX1276StartCad( void )
{
    assert(false);
    //sx1276_start_cad(dev_ptr);
}

int16_t SX1276ReadRssi( RadioModems_t modem )
{
    assert(false);
    //dev_ptr->settings.modem = modem;
    //return sx1276_read_rssi(dev_ptr);
}

//TODO: Dummy function. Just used to set SYNCWORD
void SX1276Write( uint8_t addr, uint8_t data )
{
    assert(false);
}

//TODO: Dummy function
uint8_t SX1276Read( uint8_t addr )
{
    assert(false);
    return 0;
}

//TODO: Dummy function
void SX1276WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    //sx1276_reg_write_burst(dev_ptr, addr, buffer,
     //                       size);
}

void SX1276ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    //sx1276_reg_read_burst(dev_ptr, addr, buffer,
    //                       size);
}

void SX1276SetMaxPayloadLength( RadioModems_t modem, uint8_t max )
{
    assert(false);
    //sx1276_set_max_payload_len(dev_ptr, max);
}

bool SX1276CheckRfFrequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
    assert(false);
    return true;
}

/**
 * LoRa function callbacks
 */
const struct Radio_s Radio =
{
    SX1276Init,
    SX1276GetStatus,
    SX1276SetModem,
    SX1276SetChannel,
    SX1276IsChannelFree,
    SX1276Random,
    SX1276SetRxConfig,
    SX1276SetTxConfig,
    SX1276CheckRfFrequency,
    SX1276GetTimeOnAir,
    SX1276Send,
    SX1276SetSleep,
    SX1276SetStby, 
    SX1276SetRx,
    SX1276StartCad,
    SX1276ReadRssi,
    SX1276Write,
    SX1276Read,
    SX1276WriteBuffer,
    SX1276ReadBuffer,
    SX1276SetMaxPayloadLength
};
