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

#include "debug.h"
#include "lmic.h"
#include "board.h"

void debug_init (void) {
    printf("\r\n============== DEBUG STARTED ==============\r\n");
}

void debug_led (u1_t val) {
	if(val)
    	LED0_ON;
    else
    	LED0_OFF;
}

void debug_char (u1_t c) {
	printf((const char*)&c);
}

void debug_hex (u1_t b) {
    printf("%02x", b);
}

void debug_buf (const u1_t* buf, u2_t len) {
    while(len--) {
        debug_hex(*buf++);
        printf(" ");
    }
    printf("\r\n");
}

void debug_uint (u4_t v) {
    printf("%08x", v);
}

void debug_int (s4_t v) {
    printf("%d", v);
}

void debug_str (const u1_t* str) {
    printf((const char *)str);
}

void debug_val (const u1_t* label, u4_t val) {
    printf("%s %08x \r\n", label, val);
}

void debug_valdec (const u1_t* label, s4_t val) {
    printf("%s %d \r\n", label, val);
}

int debug_fmt (char* buf, int max, s4_t val, int base, int width, char pad) {
    char num[33], *p = num, *b = buf;
    u4_t m, v;
    // special handling of negative decimals
    v = (base == 10 && val < 0) ? -val : val;
    // generate digits backwards
    do {
        *p++ = ((m=v%base) <= 9) ? m+'0' : m+'A'-10;
    } while( v /= base );
    // prefix negative decimals with '-'
    if(base == 10 && val < 0) {
        *p++ = '-';
    }
    // add leading zeroes or spaces
    while( b-buf < max-1 && b-buf < width-(p-num) ) {
        *b++ = pad;
    }
    // copy digits and sign forwards
    do *b++ = *--p;
    while( b-buf < max && p > num );
    // return number of characters written
    return b - buf;
}

void debug_event (int ev) {
    static const char* evnames[] = {
        [EV_SCAN_TIMEOUT]   = "SCAN_TIMEOUT",
        [EV_BEACON_FOUND]   = "BEACON_FOUND",
        [EV_BEACON_MISSED]  = "BEACON_MISSED",
        [EV_BEACON_TRACKED] = "BEACON_TRACKED",
        [EV_JOINING]        = "JOINING",
        [EV_JOINED]         = "JOINED",
        [EV_RFU1]           = "RFU1",
        [EV_JOIN_FAILED]    = "JOIN_FAILED",
        [EV_REJOIN_FAILED]  = "REJOIN_FAILED",
        [EV_TXCOMPLETE]     = "TXCOMPLETE",
        [EV_LOST_TSYNC]     = "LOST_TSYNC",
        [EV_RESET]          = "RESET",
        [EV_RXCOMPLETE]     = "RXCOMPLETE",
        [EV_LINK_DEAD]      = "LINK_DEAD",
        [EV_LINK_ALIVE]     = "LINK_ALIVE",
        [EV_SCAN_FOUND]     = "SCAN_FOUND",
        [EV_TXSTART]        = "EV_TXSTART",
    };
    debug_str((ev < sizeof(evnames)/sizeof(evnames[0])) ? (const u1_t*) evnames[ev] : (const u1_t*)"EV_UNKNOWN" );
    debug_char('\r');
    debug_char('\n');
}
