export APPLICATION = driver_sx1276
include ../Makefile.tests_common

FEATURES_REQUIRED = periph_spi periph_gpio

USEMODULE += xtimer
USEPKG += lmic
USEMODULE += lmic
USEMODULE += lmic_contrib

CFLAGS += -DCFG_us915
CFLAGS += -DCFG_sx1276_radio

include $(RIOTBASE)/Makefile.include

