PKG_NAME=lmic
PKG_URL=https://github.com/jia200x/LMIC.git
	PKG_VERSION=ac67aca2bb51454f1eb250659afd57a16862ba87
	PKG_BUILDDIR ?= $(BINDIRBASE)/pkg/$(BOARD)/$(PKG_NAME)

.PHONY: all

all: git-download
		$(MAKE) -C $(PKG_BUILDDIR)

include $(RIOTBASE)/pkg/pkg.mk
