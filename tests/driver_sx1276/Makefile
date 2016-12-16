export APPLICATION = driver_sx1276
include ../Makefile.tests_common

FEATURES_REQUIRED = periph_spi periph_gpio

USEMODULE += xtimer
USEPKG += lmic
USEMODULE += lmic
USEMODULE += lmic_contrib

CFLAGS += -DCFG_us915
CFLAGS += -DCFG_sx1276_radio
CFLAGS += -DLMIC_DEBUG_LEVEL=0
CFLAGS += -DUSE_IDEETRON_AES
CFLAGS += -DOSTICKS_PER_SEC=62500

include $(RIOTBASE)/Makefile.include

