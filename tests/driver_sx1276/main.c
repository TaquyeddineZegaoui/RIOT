/*
 * Copyright (c) 2014-2016 IBM Corporation.
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of the <organization> nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include "lmic.h"
#include <stdio.h>
#include "hal.h"
#include "lmic/hal.h"
#include "periph/gpio.h"

const unsigned TX_INTERVAL = 1;

// sensor functions
void initsensor(void)
{
}
u2_t readsensor(void)
{
    return 100;
}

static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;

// Pin mapping
void LMIC_enableChannel (u1_t channel) {
    if( channel < 72+MAX_XCHANNELS )
        LMIC.channelMap[channel>>4] |= (1<<(channel&0xF));
}

void  LMIC_enableSubBand (u1_t band) {
  ASSERT(band < 8);
  u1_t start = band * 8;
  u1_t end = start + 8;
  for (int channel=start; channel < end; ++channel )
      LMIC_enableChannel(channel);
}
void  LMIC_disableSubBand (u1_t band) {
  ASSERT(band < 8);
  u1_t start = band * 8;
  u1_t end = start + 8;
  for (int channel=start; channel < end; ++channel )
      LMIC_disableChannel(channel);
}
void  LMIC_selectSubBand (u1_t band) {
  ASSERT(band < 8);
  for (int b=0; b<8; ++b) {
    if (band==b)
      LMIC_enableSubBand(b);
    else
      LMIC_disableSubBand(b);
  }
}
#if 0
const lmic_pinmap lmic_pins = {
    .nss = GPIO_PIN(PA, 19),
    .rxtx = GPIO_UNDEF,
    .rst = GPIO_PIN(PA, 28),
    .dio = {GPIO_PIN(PA, 13), GPIO_PIN(PA, 7), GPIO_PIN(PA, 6)},
};
#else
const lmic_pinmap lmic_pins = {
    .nss = GPIO_PIN(PORT_C, 8),
    .rxtx = GPIO_UNDEF,
    .rst = GPIO_PIN(PORT_A, 9),
    .dio = {GPIO_PIN(PORT_B, 0), GPIO_PIN(PORT_B, 1), GPIO_PIN(PORT_C, 6)},
};
#endif


//////////////////////////////////////////////////
// CONFIGURATION (FOR APPLICATION CALLBACKS BELOW)
//////////////////////////////////////////////////

// application router ID (LSBF)
static u1_t APPEUI[8]  = {0xdb, 0xd3, 0x3d, 0xcf, 0x5e, 0x2e, 0x5e, 0x76};

// unique device ID (LSBF)
static u1_t DEVEUI[8]  = {0x2e, 0xe9, 0x9e, 0x79, 0xe8, 0xdd, 0xb5, 0x5e};

// device-specific AES key (derived from device EUI)
static u1_t DEVKEY[16] = {0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3a};

static u4_t DEVADDR = 0x17ebd35a;


// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static u1_t NWKSKEY[16] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3A };

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static u1_t APPSKEY[16] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3A };



//////////////////////////////////////////////////
// APPLICATION CALLBACKS
//////////////////////////////////////////////////

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        puts("OP_TXRXPEND, not sending");
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        puts("Packet queued");
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

// provide application router ID (8 bytes, LSBF)
void os_getArtEui (u1_t* buf) {
    memcpy(buf, APPEUI, 8);
}

// provide device ID (8 bytes, LSBF)
void os_getDevEui (u1_t* buf) {
    memcpy(buf, DEVEUI, 8);
}

// provide device key (16 bytes)
void os_getDevKey (u1_t* buf) {
    memcpy(buf, DEVKEY, 16);
}


// application entry point
int main (void) {
 // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    
    LMIC.channelMap[0] = 4;
    LMIC.channelMap[1] = 0;
    LMIC.channelMap[2] = 0;
    LMIC.channelMap[3] = 0;
    LMIC_setDrTxpow(DR_SF7,2);
    LMIC_setLinkCheckMode(0);
    printf("%i\n", LMIC.dn2Dr);
    //LMIC.dn2Dr = DR_SF12CR;

    // Start job
    do_send(&sendjob);
    os_runloop();
    return 0;
}


//////////////////////////////////////////////////
// UTILITY JOB
//////////////////////////////////////////////////


//////////////////////////////////////////////////
// LMIC EVENT CALLBACK
//////////////////////////////////////////////////

void onEvent (ev_t ev) {
    //debug_event(ev);

      printf("EV: %i\n", ev);
    switch(ev) {

        case EV_TXCOMPLETE:
            puts("EV_TXCOMPLETE (includes waiting for RX windows)");
            if (LMIC.txrxFlags & TXRX_ACK)
                puts("Received ack");
            if (LMIC.dataLen) {
                puts("Received ");
                printf("%i ", (int) LMIC.dataLen);
                puts(" bytes of payload");
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
      default:
          break;
    }
}
