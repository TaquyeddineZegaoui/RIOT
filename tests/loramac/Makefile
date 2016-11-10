APPLICATION = loramac

BOARD ?= samr21-xpro

CFLAGS += -DUSE_BAND_915
RIOTBASE ?= $(CURDIR)/../..

USEPKG += semtech-loramac

DRIVER ?= sx1276

# include the selected driver
USEMODULE += $(DRIVER)

include $(RIOTBASE)/Makefile.include
