/*
 * Copyright (C) 2016 Fundación Inria Chile
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_sx1276
 * @{
 * @file
 * @brief       Netdev2 adoption for the sx1276 driver
 *
 * @author      José Ignacio Alamos <jose.alamos@inria.cl>
 * @}
 */

#include "net/netdev2.h"
#include "include/sx1276_registers.h"
#include "include/sx1276_internal.h"
#include "sx1276_netdev.h"
#include "sx1276.h"
#include <stddef.h>
#include <string.h>
#include "errno.h"

#define RSSI_OFFSET_LF                              -164
#define RSSI_OFFSET_HF                              -157

static int _send(netdev2_t *netdev, const struct iovec *vector, int count);
static int _recv(netdev2_t *netdev, char *buf, int len, void *info);
static int _init(netdev2_t *netdev);
static void _isr(netdev2_t *netdev);
static int _get(netdev2_t *netdev, netopt_t opt, void *val, size_t max_len);
static int _set(netdev2_t *netdev, netopt_t opt, void *val, size_t len);

static int _init(netdev2_t *netdev)
{
    sx1276_t *sx1276 = (sx1276_t*) netdev;

    sx1276->irq = 0;
    sx1276_settings_t settings;
    settings.channel = RF_FREQUENCY;
    settings.modem = SX1276_MODEM_LORA;
    settings.state = SX1276_RF_IDLE;

    sx1276->settings = settings;

    /* Launch initialization of driver and device */
    puts("init_radio: initializing driver...");
    sx1276_init(sx1276);

    init_configs(sx1276);
    /* Put chip into sleep */
    sx1276_set_sleep(sx1276);

    puts("init_radio: sx1276 initialization done");

    return 0;
}

static void _isr(netdev2_t *netdev)
{
    sx1276_t *dev = (sx1276_t *) netdev;

    uint8_t irq = dev->irq;
    dev->irq = 0;

    if(irq & SX1276_IRQ_DIO0)
    {
        sx1276_on_dio0(dev);
    }
    if(irq & SX1276_IRQ_DIO1)
    {
        sx1276_on_dio1(dev);
    }
    if(irq & SX1276_IRQ_DIO2)
    {
        sx1276_on_dio2(dev);
    }
    if(irq & SX1276_IRQ_DIO3)
    {
        sx1276_on_dio3(dev);
    }
    if(irq & SX1276_IRQ_DIO4)
    {
        sx1276_on_dio4(dev);
    }
    if(irq & SX1276_IRQ_DIO5)
    {
        sx1276_on_dio5(dev);
    }
}

static uint8_t get_tx_len(const struct iovec *vector, unsigned count)
{
    uint8_t len = 0;

    for(int i=0;i<count;i++)
    {
        len += vector[i].iov_len;
    }

    return len;
}

static int _send(netdev2_t *netdev, const struct iovec *vector, int count)
{
    sx1276_t *dev = (sx1276_t*) netdev;
    uint8_t size;
    size = get_tx_len(vector, count);
    switch (dev->settings.modem) {
        case SX1276_MODEM_FSK:
            sx1276_write_fifo(dev, &size, 1);
            for(int i=0;i<count;i++)
            {
                sx1276_write_fifo(dev, vector[i].iov_base, vector[i].iov_len);
            }
            break;

        case SX1276_MODEM_LORA:
        {

            if (dev->settings.lora.iq_inverted) {
                sx1276_reg_write(dev,
                                 SX1276_REG_LR_INVERTIQ,
                                 ((sx1276_reg_read(dev, SX1276_REG_LR_INVERTIQ)
                                   & SX1276_RF_LORA_INVERTIQ_TX_MASK & SX1276_RF_LORA_INVERTIQ_RX_MASK)
                                  | SX1276_RF_LORA_INVERTIQ_RX_OFF | SX1276_RF_LORA_INVERTIQ_TX_ON));
                sx1276_reg_write(dev, SX1276_REG_LR_INVERTIQ2, SX1276_RF_LORA_INVERTIQ2_ON);
            }
            else {
                sx1276_reg_write(dev,
                                 SX1276_REG_LR_INVERTIQ,
                                 ((sx1276_reg_read(dev, SX1276_REG_LR_INVERTIQ)
                                   & SX1276_RF_LORA_INVERTIQ_TX_MASK & SX1276_RF_LORA_INVERTIQ_RX_MASK)
                                  | SX1276_RF_LORA_INVERTIQ_RX_OFF | SX1276_RF_LORA_INVERTIQ_TX_OFF));
                sx1276_reg_write(dev, SX1276_REG_LR_INVERTIQ2, SX1276_RF_LORA_INVERTIQ2_OFF);
            }

            /* Initializes the payload size */
            sx1276_reg_write(dev, SX1276_REG_LR_PAYLOADLENGTH, size);

            /* Full buffer used for Tx */
            sx1276_reg_write(dev, SX1276_REG_LR_FIFOTXBASEADDR, 0x00);
            sx1276_reg_write(dev, SX1276_REG_LR_FIFOADDRPTR, 0x00);

            /* FIFO operations can not take place in Sleep mode
             * So wake up the chip */
            if ((sx1276_reg_read(dev, SX1276_REG_OPMODE) & ~SX1276_RF_OPMODE_MASK)
                == SX1276_RF_OPMODE_SLEEP) {
                sx1276_set_standby(dev);
                xtimer_usleep(SX1276_RADIO_WAKEUP_TIME); /* wait for chip wake up */
            }

            /* Write payload buffer */
            for(int i=0;i<count;i++)
            {
                sx1276_write_fifo(dev, vector[i].iov_base, vector[i].iov_len);
            }
            break;
        }
    }

    /* Enable TXDONE interrupt */
    sx1276_reg_write(dev, SX1276_REG_LR_IRQFLAGSMASK,
                     SX1276_RF_LORA_IRQFLAGS_RXTIMEOUT |
                     SX1276_RF_LORA_IRQFLAGS_RXDONE |
                     SX1276_RF_LORA_IRQFLAGS_PAYLOADCRCERROR |
                     SX1276_RF_LORA_IRQFLAGS_VALIDHEADER |
                     /* SX1276_RF_LORA_IRQFLAGS_TXDONE | */
                     SX1276_RF_LORA_IRQFLAGS_CADDONE |
                     SX1276_RF_LORA_IRQFLAGS_FHSSCHANGEDCHANNEL |
                     SX1276_RF_LORA_IRQFLAGS_CADDETECTED);

    /* Set TXDONE interrupt to the DIO0 line */
    sx1276_reg_write(dev,
                     SX1276_REG_DIOMAPPING1,
                     (sx1276_reg_read(dev, SX1276_REG_DIOMAPPING1)
                      & SX1276_RF_LORA_DIOMAPPING1_DIO0_MASK)
                     | SX1276_RF_LORA_DIOMAPPING1_DIO0_01);

    /* Put chip into transfer mode */
    sx1276_set_status(dev, SX1276_RF_TX_RUNNING);
    sx1276_set_op_mode(dev, SX1276_RF_OPMODE_TRANSMITTER);

    return 0;
}

static int _recv(netdev2_t *netdev, char *buf, int len, void *info)
{
    /* Clear IRQ */
    volatile uint8_t irq_flags = 0;

    sx1276_t *dev = (sx1276_t*) netdev;
    sx1276_reg_write(dev,  SX1276_REG_LR_IRQFLAGS, SX1276_RF_LORA_IRQFLAGS_RXDONE);

    irq_flags = sx1276_reg_read(dev,  SX1276_REG_LR_IRQFLAGS);
    if ((irq_flags & SX1276_RF_LORA_IRQFLAGS_PAYLOADCRCERROR_MASK) == SX1276_RF_LORA_IRQFLAGS_PAYLOADCRCERROR) {
        sx1276_reg_write(dev,  SX1276_REG_LR_IRQFLAGS, SX1276_RF_LORA_IRQFLAGS_PAYLOADCRCERROR); /* Clear IRQ */

        if (!dev->settings.lora.rx_continuous) {
            sx1276_set_status(dev,  SX1276_RF_IDLE);
        }

        return -EBADMSG;
    }

    //TODO: Add snr info to rx_info
    if(info)
    {
        int8_t snr = 0;
        uint8_t snr_value = sx1276_reg_read(dev,  SX1276_REG_LR_PKTSNRVALUE);
        if (snr_value & 0x80) { /* The SNR is negative */
            /* Invert and divide by 4 */
            snr = ((~snr_value + 1) & 0xFF) >> 2;
            snr = -snr;
        }
        else {
            /* Divide by 4 */
            snr = (snr_value & 0xFF) >> 2;
        }

        int16_t rssi = sx1276_reg_read(dev, SX1276_REG_LR_PKTRSSIVALUE);
        netdev2_sx1276_rx_info_t *rx_info = info;

        if (snr < 0) {
            if (dev->settings.channel > SX1276_RF_MID_BAND_THRESH) {
                rx_info->rssi = RSSI_OFFSET_HF + rssi + (rssi >> 4) + snr;
            }
            else {
                rx_info->rssi = RSSI_OFFSET_LF + rssi + (rssi >> 4) + snr;
            }
        }
        else {
            if (dev->settings.channel > SX1276_RF_MID_BAND_THRESH) {
                rx_info->rssi = RSSI_OFFSET_HF + rssi + (rssi >> 4);
            }
            else {
                rx_info->rssi = RSSI_OFFSET_LF + rssi + (rssi >> 4);
            }
        }
    }

    uint8_t size = sx1276_reg_read(dev, SX1276_REG_LR_RXNBBYTES);
    if (buf == NULL)
        return size;

    if (size > len)
        return -ENOBUFS;

    if (!dev->settings.lora.rx_continuous) {
        sx1276_set_status(dev,  SX1276_RF_IDLE);
    }

    /* Read the last packet from FIFO */
    uint8_t last_rx_addr = sx1276_reg_read(dev, SX1276_REG_LR_FIFORXCURRENTADDR);
    sx1276_reg_write(dev, SX1276_REG_LR_FIFOADDRPTR, last_rx_addr);
    sx1276_read_fifo(dev, (uint8_t *) buf, size);

    return size;
}

static int _set_state(sx1276_t *dev, netopt_state_t state)
{
    switch (state) {
        case NETOPT_STATE_SLEEP:
            sx1276_set_sleep(dev);
            break;

        case NETOPT_STATE_STANDBY:
            sx1276_set_standby(dev); 
            break;

        case NETOPT_STATE_IDLE:
            sx1276_set_rx(dev);
            break;

        case NETOPT_STATE_TX:
            //TODO: Implement preloading
            break;

        case NETOPT_STATE_RESET:
            sx1276_reset(dev);
            break;

        default:
            return -ENOTSUP;
    }
    return sizeof(netopt_state_t);
}

static int _get_state(sx1276_t *dev, void *val)
{
    uint8_t op_mode;
    op_mode = sx1276_get_op_mode(dev);
    netopt_state_t state;
    switch(op_mode)
    {
        case SX1276_RF_OPMODE_SLEEP:
            state = NETOPT_STATE_SLEEP;
            break;

        case SX1276_RF_OPMODE_STANDBY:
            state = NETOPT_STATE_STANDBY;
            break;

        case SX1276_RF_OPMODE_TRANSMITTER:
            state = NETOPT_STATE_TX;
            break;

        case SX1276_RF_OPMODE_RECEIVER:
        case SX1276_RF_LORA_OPMODE_RECEIVER_SINGLE:
            state = NETOPT_STATE_IDLE;
            break;

        default:
            break;
    }
    memcpy(val, &state, sizeof(netopt_state_t));
    return sizeof(netopt_state_t);
}


static int _get(netdev2_t *netdev, netopt_t opt, void *val, size_t max_len)
{
    switch(opt)
    {
        case NETOPT_STATE:
            return _get_state((sx1276_t*) netdev, val);

        case NETOPT_LORA_BANDWIDTH:
            *((sx1276_lora_bandwidth_t*) val) = sx1276_get_bandwidth((sx1276_t*) netdev);
            return sizeof(sx1276_lora_bandwidth_t);

        case NETOPT_LORA_SPREADING_FACTOR:
            *((sx1276_lora_spreading_factor_t*) val) = sx1276_get_spreading_factor((sx1276_t*) netdev);
            return sizeof(sx1276_lora_spreading_factor_t);
        case NETOPT_LORA_CODING_RATE:
            *((sx1276_lora_coding_rate_t*) val) = sx1276_get_coding_rate((sx1276_t*) netdev);
            return sizeof(sx1276_lora_coding_rate_t);

        case NETOPT_LORA_SINGLE_RECEIVE:
            *((uint8_t*) val) = sx1276_get_rx_single((sx1276_t*) netdev);
            return sizeof(uint8_t);

        default:
            break;
    }
    return 0;
}
static int _set(netdev2_t *netdev, netopt_t opt, void *val, size_t len)
{
    switch(opt)
    {
        case NETOPT_STATE:
            return _set_state((sx1276_t*) netdev, *((netopt_state_t*) val));

        case NETOPT_LORA_BANDWIDTH:
            sx1276_configure_lora_bw((sx1276_t*) netdev, *((sx1276_lora_bandwidth_t*) val));
            return sizeof(sx1276_lora_bandwidth_t);

        case NETOPT_LORA_SPREADING_FACTOR:
            sx1276_configure_lora_sf((sx1276_t*) netdev, *((sx1276_lora_spreading_factor_t*) val));
            return sizeof(sx1276_lora_spreading_factor_t);
        case NETOPT_LORA_CODING_RATE:
            sx1276_configure_lora_cr((sx1276_t*) netdev, *((sx1276_lora_coding_rate_t*) val));
            return sizeof(sx1276_lora_coding_rate_t);

        case NETOPT_LORA_SINGLE_RECEIVE:
            sx1276_set_rx_single((sx1276_t*) netdev, *((uint8_t*) val));
            return sizeof(uint8_t);

        case NETOPT_CHANNEL:
            sx1276_set_channel((sx1276_t*) netdev, *((uint32_t*) val));
            return sizeof(uint32_t);

        default:
            break;
    }
    return 0;
}

const netdev2_driver_t sx1276_driver = {
    .send = _send,
    .recv = _recv,
    .init = _init,
    .isr = _isr,
    .get = _get,
    .set = _set,
};
