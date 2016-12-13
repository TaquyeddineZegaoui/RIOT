APPLICATION = driver_sx1276
include ../Makefile.tests_common

BOARD := samr21-xpro

CFLAGS += -DDEVELHELP
USEMODULE += sx1276
USEMODULE += xtimer


FEATURES_REQUIRED = periph_spi periph_gpio

include $(RIOTBASE)/Makefile.include
