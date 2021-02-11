# Hey Emacs, this is a -*- makefile -*-
#
# tello.makefile
#
# Ryzen / DJI tello
#

BOARD=tello
BOARD_CFG=\"boards/$(BOARD).h\"

ARCH=linux
$(TARGET).ARCHDIR = $(ARCH)
# include conf/Makefile.swing (with specific upload rules) instead of only Makefile.linux:
ap.MAKEFILE = tello

# -----------------------------------------------------------------------
HOST?=192.168.10.1
# -----------------------------------------------------------------------

# The datalink default uses UDP
MODEM_HOST         ?= 127.0.0.1

# handle linux signals by hand
$(TARGET).CFLAGS += -DUSE_LINUX_SIGNAL -D_GNU_SOURCE

# board specific init function
$(TARGET).srcs += $(SRC_BOARD)/board.c

# Link static (Done for GLIBC)
$(TARGET).CFLAGS += -DLINUX_LINK_STATIC
$(TARGET).LDFLAGS += -static

# -----------------------------------------------------------------------

# default LED configuration
RADIO_CONTROL_LED  ?= none
BARO_LED           ?= none
AHRS_ALIGNER_LED   ?= none
GPS_LED            ?= none
SYS_TIME_LED       ?= none
