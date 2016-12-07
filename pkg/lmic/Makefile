PKG_NAME=lmic
PKG_URL=https://github.com/mirakonta/lmic.git
	PKG_VERSION=4bc3478b49229598f8462a9dc630d546da5ce0b4
	PKG_BUILDDIR ?= $(BINDIRBASE)/pkg/$(BOARD)/$(PKG_NAME)

.PHONY: all

all: git-download
		$(MAKE) -C $(PKG_BUILDDIR)

include $(RIOTBASE)/pkg/pkg.mk
