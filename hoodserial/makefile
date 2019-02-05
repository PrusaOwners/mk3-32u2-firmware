#
#             LUFA Library
#     Copyright (C) Dean Camera, 2015.
#
#  dean [at] fourwalledcubicle [dot] com
#           www.lufa-lib.org
#
# --------------------------------------
#         LUFA Project Makefile.
# --------------------------------------

# Run "make help" for target help.

MCU          = atmega32u2
ARCH         = AVR8
BOARD        = USER
F_CPU        = 16000000
F_USB        = $(F_CPU)
OPTIMIZATION = s
TARGET       = hoodserial
SRC          = $(TARGET).c Descriptors.c $(LUFA_SRC_USB) $(LUFA_SRC_USBCLASS)
LUFA_PATH    = ../lufa-LUFA-181228/LUFA
CC_FLAGS     = -DUSE_LUFA_CONFIG_HEADER -IConfig/ $(HOODSERIAL_OPTS) $(REGS) -flto -fuse-linker-plugin
LD_FLAGS     = -Wl,--section-start,.data=$(RAM_OFFSET) $(REGS) -flto -fuse-linker-plugin


# Specify the Arduino model using the assigned PID.  This is used by Descriptors.c
#   to set PID and product descriptor string

#define ARDUINO_VID				0x2341
#define PRUSA_VID				0x2c99
#define PRUSA_MK3_PID			0x0002
#define ARDUINO_UNO_PID			0x0001
#define ARDUINO_MEGA2560_PID	0x0010
#define ARDUINO_USBSERIAL_PID	0x003B
#define ARDUINO_MEGAADK_PID		0x003F
#define ARDUINO_MEGA2560R3_PID	0x0042
#define ARDUINO_UNOR3_PID		0x0043
#define ARDUINO_MEGAADKR3_PID	0x0044

HOODSERIAL_OPTS = -DVENDORID=PRUSA_VID
HOODSERIAL_OPTS += -DPRODUCTID=PRUSA_MK3_PID

# Read Prusa MK3 serial from EEPROM
HOODSERIAL_OPTS += -DPRUSA_MK3_SERIAL

# Custom serial string. Uncomment and Replace with your serial number
# if you have one.
#SERIAL_NUMBER=123456
#HOODSERIAL_OPTS += -DSERIALID=L\"$(SERIAL_NUMBER)\"


# Add an offset of 256 + 128 bytes to RAMSTART for the ringbuffers
# u2 and u4 Series (RAMSTART = 0x100), plus 2 bytes padding
RAM_OFFSET            = 0x800282

# Reserved registers for faster USB-Serial convertion
# The lower, the better (except 0 and 1)
# r2-5 used for faster USART ISRs
# r6 used for 32u4 optimization
REGS  = -ffixed-r2 -ffixed-r3 -ffixed-r4 -ffixed-r5


# Default target
all:

# Include LUFA-specific DMBS extension modules
DMBS_LUFA_PATH ?= $(LUFA_PATH)/Build/LUFA
include $(DMBS_LUFA_PATH)/lufa-sources.mk
include $(DMBS_LUFA_PATH)/lufa-gcc.mk

# Include common DMBS build system modules
DMBS_PATH      ?= $(LUFA_PATH)/Build/DMBS/DMBS
include $(DMBS_PATH)/core.mk
include $(DMBS_PATH)/cppcheck.mk
include $(DMBS_PATH)/doxygen.mk
include $(DMBS_PATH)/dfu.mk
include $(DMBS_PATH)/gcc.mk
include $(DMBS_PATH)/hid.mk
include $(DMBS_PATH)/avrdude.mk
include $(DMBS_PATH)/atprogram.mk
