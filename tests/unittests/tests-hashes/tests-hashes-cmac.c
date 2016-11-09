/*
 * Copyright (C) 2016 Fundación Inria Chile
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     unittests
 * @{
 *
 * @file
 * @brief       Test cases for the AES-CMAC hash implementation
 *
 * @author      José Ignacio Alamos <jose.alamos@inria.cl>
 *
 * @}
 */

#include "tests-hashes.h"
#include "embUnit/embUnit.h"
static void test_hashes_cmac(void)
{
}

Test *tests_hashes_cmac_tests(void)
{
    EMB_UNIT_TESTFIXTURES(fixtures) {
        new_TestFixture(test_hashes_cmac),
    };

    EMB_UNIT_TESTCALLER(test_hashes_cmac, NULL, NULL, fixtures);

    return (Test *)&test_hashes_cmac;
}
