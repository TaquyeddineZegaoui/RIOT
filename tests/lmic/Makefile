APPLICATION = lmic_app
include ../Makefile.tests_common

FEATURES_REQUIRED = periph_spi periph_gpio

USEPKG += lmic

CFLAGS += -DCFG_us915
CFLAGS += -DCFG_lmic_clib
CFLAGS += -DCFG_sx1276_radio

USEMODULE += lmic
USEMODULE += lmic_contrib

include $(RIOTBASE)/Makefile.include
