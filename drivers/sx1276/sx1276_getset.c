#include "sx1276.h"
#include "sx1276_registers.h"
#include "sx1276_internal.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>

sx1276_lora_bandwidth_t sx1276_get_bandwidth(sx1276_t *dev)
{
    return dev->settings.lora.bandwidth;
}

sx1276_lora_spreading_factor_t sx1276_get_spreading_factor(sx1276_t *dev)
{
    return dev->settings.lora.datarate;
}

sx1276_lora_coding_rate_t sx1276_get_coding_rate(sx1276_t *dev)
{
    return dev->settings.lora.coderate;
}

sx1276_radio_state_t sx1276_get_status(sx1276_t *dev)
{
    return dev->settings.state;
}

uint32_t sx1276_get_channel(sx1276_t *dev)
{
    return ((sx1276_reg_read(dev, SX1276_REG_FRFMSB) << 16) | (sx1276_reg_read(dev, SX1276_REG_FRFMID) << 8) | sx1276_reg_read(dev, SX1276_REG_FRFLSB)) * SX1276_FREQ_STEP;

}

uint8_t sx1276_get_syncword(sx1276_t *dev)
{
    return sx1276_reg_read(dev, SX1276_REG_LR_SYNCWORD);
}

void sx1276_set_syncword(sx1276_t *dev, uint8_t syncword)
{
    sx1276_reg_write(dev, SX1276_REG_LR_SYNCWORD, syncword);
}

void sx1276_set_channel(sx1276_t *dev, uint32_t freq)
{
    /* Save current operating mode */
    uint8_t prev_mode = sx1276_reg_read(dev, SX1276_REG_OPMODE);

    sx1276_set_op_mode(dev, SX1276_RF_OPMODE_STANDBY);

    freq = (uint32_t)((double) freq / (double) SX1276_FREQ_STEP);

    /* Write frequency settings into chip */
    sx1276_reg_write(dev, SX1276_REG_FRFMSB, (uint8_t)((freq >> 16) & 0xFF));
    sx1276_reg_write(dev, SX1276_REG_FRFMID, (uint8_t)((freq >> 8) & 0xFF));
    sx1276_reg_write(dev, SX1276_REG_FRFLSB, (uint8_t)(freq & 0xFF));

    /* Restore previous operating mode */
    sx1276_reg_write(dev, SX1276_REG_OPMODE, prev_mode);
}

void sx1276_set_modem(sx1276_t *dev, sx1276_radio_modems_t modem)
{
    dev->settings.modem = modem;

    switch (dev->settings.modem) {
        case SX1276_MODEM_LORA:
            sx1276_set_op_mode(dev, SX1276_RF_OPMODE_SLEEP);
            sx1276_reg_write(dev,
                             SX1276_REG_OPMODE,
                             (sx1276_reg_read(dev, SX1276_REG_OPMODE)
                              & SX1276_RF_LORA_OPMODE_LONGRANGEMODE_MASK)
                             | SX1276_RF_LORA_OPMODE_LONGRANGEMODE_ON);

            sx1276_reg_write(dev, SX1276_REG_DIOMAPPING1, 0x00);
            sx1276_reg_write(dev, SX1276_REG_DIOMAPPING2, 0x10); /* DIO5=ClkOut */
            break;

        case SX1276_MODEM_FSK:
            sx1276_set_op_mode(dev, SX1276_RF_OPMODE_SLEEP);
            sx1276_reg_write(dev,
                             SX1276_REG_OPMODE,
                             (sx1276_reg_read(dev, SX1276_REG_OPMODE)
                              & SX1276_RF_LORA_OPMODE_LONGRANGEMODE_MASK)
                             | SX1276_RF_LORA_OPMODE_LONGRANGEMODE_OFF);

            sx1276_reg_write(dev, SX1276_REG_DIOMAPPING1, 0x00);
            break;
        default:
            break;
    }
}

uint32_t sx1276_get_time_on_air(sx1276_t *dev, sx1276_radio_modems_t modem,
                                uint8_t pkt_len)
{
    uint32_t air_time = 0;

    switch (modem) {
        case SX1276_MODEM_FSK:
            break;

        case SX1276_MODEM_LORA:
        {
            double bw = 0.0;

            /* Note: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported. */
            switch (dev->settings.lora.bandwidth) {
                case 7: /* 125 kHz */
                    bw = 125e3;
                    break;
                case 8: /* 250 kHz */
                    bw = 250e3;
                    break;
                case 9: /* 500 kHz */
                    bw = 500e3;
                    break;
            }

            /* Symbol rate : time for one symbol [secs] */
            double rs = bw / (1 << dev->settings.lora.datarate);
            double ts = 1 / rs;

            /* time of preamble */
            double t_preamble = (dev->settings.lora.preamble_len + 4.25) * ts;

            /* Symbol length of payload and time */
            double tmp =
                ceil(
                    (8 * pkt_len - 4 * dev->settings.lora.datarate + 28
                     + 16 * dev->settings.lora.crc_on
                     - (!dev->settings.lora.implicit_header ? 20 : 0))
                    / (double) (4 * dev->settings.lora.datarate
                                - ((dev->settings.lora.low_datarate_optimize
                                    > 0) ? 2 : 0)))
                * (dev->settings.lora.coderate + 4);
            double n_payload = 8 + ((tmp > 0) ? tmp : 0);
            double t_payload = n_payload * ts;

            /* Time on air */
            double t_on_air = t_preamble + t_payload;

            /* return seconds */
            air_time = floor(t_on_air * 1e6 + 0.999);
        }
        break;
    }

    return air_time;
}

uint8_t sx1276_get_op_mode(sx1276_t *dev)
{
    return sx1276_reg_read(dev, SX1276_REG_OPMODE) & ~SX1276_RF_OPMODE_MASK;
}

void sx1276_set_op_mode(sx1276_t *dev, uint8_t op_mode)
{
    static uint8_t op_mode_prev = 0;

    op_mode_prev = sx1276_get_op_mode(dev);

    if (op_mode != op_mode_prev) {
        /* Replace previous mode value and setup new mode value */
        sx1276_reg_write(dev, SX1276_REG_OPMODE, (op_mode_prev & SX1276_RF_OPMODE_MASK) | op_mode);
    }
}

void sx1276_set_max_payload_len(sx1276_t *dev, sx1276_radio_modems_t modem, uint8_t maxlen)
{
    sx1276_set_modem(dev, modem);

    switch (modem) {
        case SX1276_MODEM_FSK:
            break;

        case SX1276_MODEM_LORA:
            sx1276_reg_write(dev, SX1276_REG_LR_PAYLOADMAXLENGTH, maxlen);
            break;
    }
}

void sx1276_set_rx_single(sx1276_t *dev, uint8_t single)
{
    dev->settings.lora.rx_continuous = single ? false : true;
}

uint8_t sx1276_get_rx_single(sx1276_t *dev)
{
    return dev->settings.lora.rx_continuous ? false : true;
}

bool sx1276_get_crc(sx1276_t *dev)
{
    return sx1276_reg_read(dev, SX1276_REG_LR_MODEMCONFIG2) & SX1276_RF_LORA_MODEMCONFIG2_RXPAYLOADCRC_MASK;
}

void sx1276_set_crc(sx1276_t *dev, bool crc)
{
    uint8_t tmp = sx1276_reg_read(dev, SX1276_REG_LR_MODEMCONFIG2);
    tmp &= ~SX1276_RF_LORA_MODEMCONFIG2_RXPAYLOADCRC_MASK;
    tmp |= crc ? SX1276_RF_LORA_MODEMCONFIG2_RXPAYLOADCRC_MASK : 0;
    sx1276_reg_write(dev, SX1276_REG_LR_MODEMCONFIG2, tmp);
}

void sx1276_set_hop_period(sx1276_t *dev, uint8_t hop_period)
{
    uint8_t tmp = sx1276_reg_read(dev, SX1276_REG_LR_PLLHOP);
    dev->settings.lora.hop_period = hop_period;
    if (dev->settings.lora.freq_hop_on) {
        tmp |= ~SX1276_RF_LORA_PLLHOP_FASTHOP_ON;
        sx1276_reg_write(dev, SX1276_REG_LR_PLLHOP, tmp);
        sx1276_reg_write(dev, SX1276_REG_LR_HOPPERIOD, hop_period);
    }
}

uint8_t sx1276_get_hop_period(sx1276_t *dev)
{
    return sx1276_reg_read(dev, SX1276_REG_LR_HOPPERIOD);
}

void sx1276_set_implicit_mode(sx1276_t *dev, bool implicit)
{
}

bool  sx12376_get_implicit_mode(sx1276_t *dev)
{
    return true;
}

void sx1276_set_low_datarate_optimize(sx1276_t *dev, bool ldo)
{
}

bool sx1276_get_low_datarate_optimize(sx1276_t *dev)
{
    return true;
}

void sx1276_set_payload_length(sx1276_t *dev, uint8_t len)
{
}

uint8_t sx1276_get_payload_length(sx1276_t *dev)
{
    return 0;
}

void sx1276_set_power(sx1276_t *dev, uint8_t power)
{
}

uint8_t sx1276_get_power(sx1276_t *dev)
{
    return 0;
}

void sx1276_set_preamble_length(sx1276_t *dev, uint8_t preamble)
{
}

uint8_t sx1276_get_preamble_length(sx1276_t *dev)
{
    return 0;
}
