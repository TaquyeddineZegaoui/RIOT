APPLICATION = loramac

FEATURES_REQUIRED = periph_spi periph_gpio periph_adc

BOARD ?= samr21-xpro

CFLAGS += -DUSE_BAND_915_HYBRID
CFLAGS += -DCRYPTO_AES
CFLAGS += -DDEVELHELP
CFLAGS += -DDEBUG_ASSERT_VERBOSE

RIOTBASE ?= $(CURDIR)/../..

DRIVER ?= sx1276

# include the selected driver
USEMODULE += $(DRIVER)
USEMODULE += hashes
USEMODULE += crypto
USEMODULE += od
USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += ps
USEMODULE += xtimer
USEMODULE += lorawan
USEMODULE += random



include $(RIOTBASE)/Makefile.include
