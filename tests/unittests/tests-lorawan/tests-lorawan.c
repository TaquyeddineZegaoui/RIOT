/*
 * Copyright (C) 2016 Fundación Inria Chile
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @{
 *
 * @file
 */
#include <errno.h>
#include <stdlib.h>

#include "embUnit.h"
#include "net/lorawan/crypto.h"

#include "unittests-constants.h"

#define OTHER_BYTE  (TEST_UINT16 >> 8)
#define DEFAULT_TEST_SRC    { { \
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, \
            0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f \
        } \
    }
#define DEFAULT_TEST_DST    { { \
            0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, \
            0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f \
        } \
    }

#define _DEV_ADDR 0x166fb58e
#define _DIR 0
#define _SEQ_COUNTER 2
#define _SIZE 20

#define EXP_MIC 0xb8e926d6
static const uint8_t _KEY[] = {0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3A };

static const uint8_t BUF[] = {0x40, 0x8e, 0xb5, 0x6f, 0x16, 0x80, 0x02, 0x00, 0x01, 0x80, 0x4c, 0x2f, 0x1e, 0x5b, 0x49, 0x49, 0x8a, 0x56, 0x7c, 0x3a};


static void test_lorawan_mic_calc(void)
{
    /* source: https://www.cloudshark.org/captures/ea72fbab241b (No. 56) */

    uint32_t mic;
    lorawan_calc_mic(BUF, sizeof(BUF), _KEY, _DEV_ADDR, _DIR, _SEQ_COUNTER, &mic);
    TEST_ASSERT_EQUAL_INT(EXP_MIC, mic);
}

Test *tests_lorawan_tests(void)
{
    EMB_UNIT_TESTFIXTURES(fixtures) {
        new_TestFixture(test_lorawan_mic_calc),
    };

    EMB_UNIT_TESTCALLER(lorawan_tests, NULL, NULL, fixtures);

    return (Test *)&lorawan_tests;
}

void tests_lorawan(void)
{
    TESTS_RUN(tests_lorawan_tests());
}
/** @} */
