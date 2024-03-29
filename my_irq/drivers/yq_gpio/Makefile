#
# Copyright (C) 2008-2010 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=yq_gpio
PKG_RELEASE:=3
PKG_LICENSE:=GPL-2.0

include $(INCLUDE_DIR)/package.mk

define KernelPackage/yq_gpio
  CATEGORY:=YQ Properties
  SUBMENU:=Drivers
  MENU:=1
  TITLE:=yq gpio driver
  DEPENDS:=+kmod-input-core
  FILES:=$(PKG_BUILD_DIR)/yq_gpio.ko
  AUTOLOAD:=$(call AutoLoad,99,yq_gpio,1)
  KCONFIG:=
endef

define KernelPackage/yq_gpio/description
  Kernel module to generate button uevent-s from input subsystem events.
  If your device uses GPIO buttons, see gpio-yq_gpio.
endef

EXTRA_KCONFIG:= \
	CONFIG_BUTTON_HOTPLUG=m

EXTRA_CFLAGS:= \
	$(patsubst CONFIG_%, -DCONFIG_%=1, $(patsubst %=m,%,$(filter %=m,$(EXTRA_KCONFIG)))) \
	$(patsubst CONFIG_%, -DCONFIG_%=1, $(patsubst %=y,%,$(filter %=y,$(EXTRA_KCONFIG)))) \

MAKE_OPTS:= \
	$(KERNEL_MAKE_FLAGS) \
	SUBDIRS="$(PKG_BUILD_DIR)" \
	EXTRA_CFLAGS="$(EXTRA_CFLAGS)" \
	$(EXTRA_KCONFIG)

define Build/Compile
	$(MAKE) -C "$(LINUX_DIR)" \
		$(MAKE_OPTS) \
		modules
endef

$(eval $(call KernelPackage,yq_gpio))
