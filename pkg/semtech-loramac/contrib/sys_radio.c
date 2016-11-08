#include "sx1276.h"
#include "radio.h"


/*!
 * \Extern netdev2_radio for use of LoRaMac
 */

typedef struct {
    struct Radio_s rad;    
    sx1276_t* dev;
} netdev2_radio_t;

static netdev2_radio_t netdev2_radio;
static sx1276_t sx1276;

/**
 * LoRa function callbacks
 */
const struct Radio_s radio =
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

// set-up reference pointers
netdev2_radio->dev = sx1276;
netdev2_radio->rad = radio; 

/*
 * Radio driver functions implementation wrappers
 */

void SX1276Init( RadioEvents_t *events )
{
    sx1276_init(netdev2_radio->dev);
}

RadioState_t SX1276GetStatus( void )
{
    sx1276_get_status(netdev2_radio->dev);
}

void SX1276SetModem( RadioModems_t modem)
{
    sx1276_set_modem(netdev2_radio->dev, (sx1276_radio_modems_t) modem);
}

void SX1276SetChannel( uint32_t freq )
{
    sx1276_set_channel(netdev2_radio->dev, freq);
}

bool SX1276IsChannelFree( RadioModems_t modem, uint32_t freq, int16_t rssiThresh )
{
    sx1276_is_channel_free(netdev2_radio->dev, freq, rssi_thresh);
}

uint32_t SX1276Random( void )
{
     sx1276_random(netdev2_radio->dev);
}

void SX1276SetRxConfig( RadioModems_t modem, uint32_t bandwidth,
                         uint32_t datarate, uint8_t coderate,
                         uint32_t bandwidthAfc, uint16_t preambleLen,
                         uint16_t symbTimeout, bool fixLen,
                         uint8_t payloadLen,
                         bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                         bool iqInverted, bool rxContinuous )
{
    sx1276_lora_settings_t settings;
    (void) fdev;
    (void) modem;
    (void) bandwidthAfc;
    sx1276_lora_settings_t settings;
    settings.bandwidth = bandwidth;
    settings.coderate = coderate;
    settings.datarate = datarate;
    settings.crc_on = crcOn;
    settings.freq_hop_on = freqHopOn;
    settings.hop_period = hopPeriod;
    settings.implicit_header = false;
    settings.iq_inverted = iqInverted;
    settings.low_datarate_optimize = fixLen;
    settings.payload_len = payloadLen;
    settings.power = power;
    settings.preamble_len = preambleLen;
    settings.rx_continuous = rxContinuous;
    settings.tx_timeout = 30; // 30 sec
    settings.rx_timeout = symbTimeout;
    sx1276_configure_lora(netdev2_radio->dev, settings);
    sx1276_configure_lora(netdev2_radio->dev, settings);
}

void SX1276SetTxConfig( RadioModems_t modem, int8_t power, uint32_t fdev,
                        uint32_t bandwidth, uint32_t datarate,
                        uint8_t coderate, uint16_t preambleLen,
                        bool fixLen, bool crcOn, bool freqHopOn,
                        uint8_t hopPeriod, bool iqInverted, uint32_t timeout )
{
    (void) fdev;
    (void) modem;
    sx1276_lora_settings_t settings;
    settings.bandwidth = bandwidth;
    settings.coderate = coderate;
    settings.datarate = datarate;
    settings.crc_on = crcOn;
    settings.freq_hop_on = freqHopOn;
    settings.hop_period = hopPeriod;
    settings.implicit_header = false;
    settings.iq_inverted = iqInverted;
    settings.low_datarate_optimize = fixLen;
    settings.payload_len = 0;
    settings.power = power;
    settings.preamble_len = preambleLen;
    settings.rx_continuous = true;
    settings.tx_timeout = timeout; // 30 sec
    settings.rx_timeout = 10;
    sx1276_configure_lora(netdev2_radio->dev, settings);
}

uint32_t SX1276GetTimeOnAir( RadioModems_t modem, uint8_t pktLen )
{
    sx1276_get_time_on_air(netdev2_radio->dev, (sx1276_radio_modems_t) modem,
                                pkt_len);
}

void SX1276Send( uint8_t *buffer, uint8_t size )
{
    sx1276_send(netdev2_radio->dev, buffer, size);
}

void SX1276SetSleep( void )
{
    void sx1276_set_sleep(netdev2_radio->dev);
}

void SX1276SetStby( void )
{
    sx1276_set_standby(netdev2_radio->dev);
}

void SX1276SetRx( uint32_t timeout )
{
    sx1276_set_rx(netdev2_radio->dev, timeout);
}


void SX1276StartCad( void )
{
    sx1276_start_cad(netdev2_radio->dev);
}

int16_t SX1276ReadRssi( RadioModems_t modem )
{
    (void) modem;
    sx1276_read_rssi(netdev2_radio->dev);
}

void SX1276Write( uint8_t addr, uint8_t data )
{
    sx1276_reg_write(netdev2_radio->dev, addr, data);
}

uint8_t SX1276Read( uint8_t addr )
{
    sx1276_reg_read(netdev2_radio->dev, addr);
}

void SX1276WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    void sx1276_reg_write_burst(netdev2_radio->dev, addr, buffer,
                            size);
}

void SX1276ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    void sx1276_reg_read_burst(netdev2_radio->dev, addr, buffer,
                           size);
}

void SX1276SetMaxPayloadLength( RadioModems_t modem, uint8_t max )
{
    sx1276_set_max_payload_len(netdev2_radio->dev, (sx1276_radio_modems_t) modem, maxlen);
}

