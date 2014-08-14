#
#             LUFA Library
#     Copyright (C) Dean Camera, 2014.
#
#  dean [at] fourwalledcubicle [dot] com
#           www.lufa-lib.org
#
# --------------------------------------
#         LUFA Project Makefile.
# --------------------------------------

# Run "make help" for target help.

# This works for the 8u2(no ISP, use Lite Version) and 16u2
MCU = atmega16u2
MCU_AVRDUDE = $(MCU)
MCU_DFU = $(MCU)

F_CPU = 16000000
BOARD  = USER

#Vendor ID from lufa         0x03EB
#Product ID created my own
#HOODLOADER                  0x6E68
#HOODLOADER-Lite             0x4E48

#You can also use the native Arduino VID and PID
#VendorID  Arduino           0x2341
#ProductID Arduino Uno       0x0001, 0x0043 R3
#ProductID Arduino Mega2560  0x0010, 0x0042 R3
#ProductID Arduino Mega ADK  0x003F, 0x0044 R3

HOODLOADER_OPTS  = -DVENDORID=LUFA_VID
HOODLOADER_OPTS += -DPRODUCTID=HOODLOADER_PID

#Version in BCD:
HOODLOADER_OPTS += -DHOODLOADER_V1=1
HOODLOADER_OPTS += -DHOODLOADER_V2=7
HOODLOADER_OPTS += -DHOODLOADER_V3=3


ARCH         = AVR8
F_USB        = $(F_CPU)
OPTIMIZATION = s
TARGET       = Hoodloader
SRC          = $(TARGET).c Descriptors.c Lib/HID.c Lib/Ram.c Lib/ISP.c Lib/NHP.c Lib/CDC.c $(LUFA_SRC_SERIAL) $(LUFA_SRC_USB) $(LUFA_SRC_USBCLASS)
LUFA_PATH    = ./lufa-LUFA-140302/LUFA
CC_FLAGS     = -DUSE_LUFA_CONFIG_HEADER -IConfig/ $(HOODLOADER_OPTS)
LD_FLAGS     =

# Default target
all:

# Include LUFA build script makefiles
include $(LUFA_PATH)/Build/lufa_core.mk
include $(LUFA_PATH)/Build/lufa_sources.mk
include $(LUFA_PATH)/Build/lufa_build.mk
include $(LUFA_PATH)/Build/lufa_cppcheck.mk
include $(LUFA_PATH)/Build/lufa_doxygen.mk
include $(LUFA_PATH)/Build/lufa_dfu.mk
include $(LUFA_PATH)/Build/lufa_hid.mk
include $(LUFA_PATH)/Build/lufa_avrdude.mk
include $(LUFA_PATH)/Build/lufa_atprogram.mk

