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
    //RadioEvents = events;
    //TODO: The final code is not using this function for driver init
    //sx1276_init(dev_ptr);
}

RadioState_t SX1276GetStatus( void )
{
    return (RadioState_t) sx1276_get_status(dev_ptr);
}

void SX1276SetModem( RadioModems_t modem)
{
    sx1276_set_modem(dev_ptr, (sx1276_radio_modems_t) modem);
}

void SX1276SetChannel( uint32_t freq )
{
    sx1276_set_channel(dev_ptr, freq);
}

bool SX1276IsChannelFree( RadioModems_t modem, uint32_t freq, int16_t rssiThresh )
{
    return sx1276_is_channel_free(dev_ptr, freq, rssiThresh);
}

uint32_t SX1276Random( void )
{
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
    netdev2_t *netdev = (netdev2_t*) dev_ptr;
    dev_ptr->settings.modem = modem;

    netopt_enable_t _modem = modem;
    netdev->driver->set(netdev, NETOPT_LORA_MODE, &_modem, sizeof(netopt_enable_t));
    (void) bandwidthAfc;

    bool freq_hop_on = freqHopOn;
    bool iq_invert = iqInverted;
    uint8_t rx_single = rxContinuous ? false : true;
    uint32_t tx_timeout = 3 * 1000 * 1000;
    sx1276_lora_bandwidth_t bw = bandwidth + 7;
    sx1276_lora_coding_rate_t cr = coderate;
    sx1276_lora_spreading_factor_t sf = datarate;
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

    netdev->driver->set(netdev, NETOPT_LORA_BANDWIDTH, &bw, sizeof(sx1276_lora_bandwidth_t));
    netdev->driver->set(netdev, NETOPT_LORA_CODING_RATE, &cr, sizeof(sx1276_lora_coding_rate_t));
    netdev->driver->set(netdev, NETOPT_LORA_SPREADING_FACTOR, &sf, sizeof(sx1276_lora_spreading_factor_t));
    netdev->driver->set(netdev, NETOPT_LORA_IMPLICIT, &implicit, sizeof(bool));
    netdev->driver->set(netdev, NETOPT_CRC, &crc, sizeof(bool));
    netdev->driver->set(netdev, NETOPT_LORA_SYMBOL_TIMEOUT, &rx_timeout, sizeof(uint16_t));
    netdev->driver->set(netdev, NETOPT_LORA_PREAMBLE_LENGTH, &preamble, sizeof(uint16_t));
    netdev->driver->set(netdev, NETOPT_LORA_PAYLOAD_LENGTH, &payload_len, sizeof(uint8_t));
    netdev->driver->set(netdev, NETOPT_LORA_HOP_PERIOD, &hop_period, sizeof(uint8_t));
    netdev->driver->set(netdev, NETOPT_TX_POWER, &power, sizeof(uint8_t));

}

void SX1276SetTxConfig( RadioModems_t modem, int8_t power, uint32_t fdev,
                        uint32_t bandwidth, uint32_t datarate,
                        uint8_t coderate, uint16_t preambleLen,
                        bool fixLen, bool crcOn, bool freqHopOn,
                        uint8_t hopPeriod, bool iqInverted, uint32_t timeout )
{
    (void) fdev;

    netdev2_t *netdev = (netdev2_t*) dev_ptr;
    dev_ptr->settings.modem = modem;

    netopt_enable_t _modem = modem;
    netdev->driver->set(netdev, NETOPT_LORA_MODE, &_modem, sizeof(netopt_enable_t));

    bool freq_hop_on = freqHopOn;
    bool iq_invert = iqInverted;
    uint8_t rx_single = false;
    uint32_t tx_timeout = timeout * 1000;
    sx1276_lora_bandwidth_t bw = bandwidth + 7;
    sx1276_lora_coding_rate_t cr = coderate;
    sx1276_lora_spreading_factor_t sf = datarate;
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

    netdev->driver->set(netdev, NETOPT_LORA_BANDWIDTH, &bw, sizeof(sx1276_lora_bandwidth_t));
    netdev->driver->set(netdev, NETOPT_LORA_CODING_RATE, &cr, sizeof(sx1276_lora_coding_rate_t));
    netdev->driver->set(netdev, NETOPT_LORA_SPREADING_FACTOR, &sf, sizeof(sx1276_lora_spreading_factor_t));
    netdev->driver->set(netdev, NETOPT_LORA_IMPLICIT, &implicit, sizeof(bool));
    netdev->driver->set(netdev, NETOPT_CRC, &crc, sizeof(bool));
    netdev->driver->set(netdev, NETOPT_LORA_SYMBOL_TIMEOUT, &rx_timeout, sizeof(uint16_t));
    netdev->driver->set(netdev, NETOPT_LORA_PREAMBLE_LENGTH, &preamble, sizeof(uint16_t));
    netdev->driver->set(netdev, NETOPT_LORA_PAYLOAD_LENGTH, &payload_len, sizeof(uint8_t));
    netdev->driver->set(netdev, NETOPT_LORA_HOP_PERIOD, &hop_period, sizeof(uint8_t));
    netdev->driver->set(netdev, NETOPT_TX_POWER, &power, sizeof(uint8_t));

}

uint32_t SX1276GetTimeOnAir( RadioModems_t modem, uint8_t pktLen )
{
    return sx1276_get_time_on_air(dev_ptr, (sx1276_radio_modems_t) modem,
                                pktLen);
}

void SX1276Send( uint8_t *buffer, uint8_t size )
{
    sx1276_send(dev_ptr, buffer, size);
}

void SX1276SetSleep( void )
{
    sx1276_set_sleep(dev_ptr);
}

void SX1276SetStby( void )
{
    sx1276_set_standby(dev_ptr);
}

void SX1276SetRx( uint32_t timeout )
{
    sx1276_set_rx(dev_ptr, timeout * 1000); // base unit us, LoRaMAC ms
}


void SX1276StartCad( void )
{
    sx1276_start_cad(dev_ptr);
}

int16_t SX1276ReadRssi( RadioModems_t modem )
{
    dev_ptr->settings.modem = modem;
    return sx1276_read_rssi(dev_ptr);
}

//TODO: Dummy function. Just used to set SYNCWORD
void SX1276Write( uint8_t addr, uint8_t data )
{
    (void) data;
    sx1276_set_syncword(dev_ptr, data);
}

//TODO: Dummy function
uint8_t SX1276Read( uint8_t addr )
{
    //return sx1276_reg_read(dev_ptr, addr);
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
    sx1276_set_max_payload_len(dev_ptr, (sx1276_radio_modems_t) modem, max);
}

bool SX1276CheckRfFrequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
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
