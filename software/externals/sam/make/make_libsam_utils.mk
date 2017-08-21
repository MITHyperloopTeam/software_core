#-------------------------------------------------------------------------------
# @author gizatt, following stfwi
#
# INCLUDE THIS FILE IN YOUR MAKEFILE USING THE ...
#
#   include <path/to/>make_libsam_utils.mk
#
# ... MAKEFILE DIRECTIVE.
#
# It then includes general utility targets, which assume a few things have been
# defined:
#  BINARY_TO_INSTALL=*.sam.bin, which will be written to arduino
#  BUILD_DIR=folder to output the intermediates and binary to
#  UPLOAD_PORT=/dev/ttyACM0 and friends, usually
#
#   install: Uploads BINARY_TO_INSTALL to UPLOAD_PORT.
#   <various build rules for intermediates also provided)
#
# This file also does some generically useful things:
#  - Appends to INCLUDES libsam, CMSIS, etc includes
#  - Creates CFLAGS_SAM, CPPFLAGS_SAM, ARFLAGS_SAM, which are the right
#    CFLAGS for building for the Arduino platform when building baremetal.
#  - Appends to LIBRARIES libsam libraries needed for linking final executables.
#    
#-------------------------------------------------------------------------------

USB_DEFINITIONS=-DUSB_VID=0x2341 -DUSB_PID=0x003e -DUSBCON '-DUSB_MANUFACTURER="Unknown"' '-DUSB_PRODUCT="Arduino Due"'

SAM_ROOT := $(realpath $(shell dirname '$(dir $(lastword $(MAKEFILE_LIST)))'))
TOUCH_PROGRAMMING_PORT=python $(SAM_ROOT)/../../tools/touch_programming_port.py $(UPLOAD_PORT)

SAM_LIB_DIR=$(SAM_ROOT)/../../build/lib/sam/
CHIP=__SAM3X8E__
F_CPU=84000000L

#-------------------------------------------------------------------------------
# Related directories and files
#-------------------------------------------------------------------------------

INCLUDES += -I$(SAM_ROOT)/include
INCLUDES += -I$(SAM_ROOT)/libsam
INCLUDES += -I$(SAM_ROOT)/CMSIS/CMSIS/Include
INCLUDES += -I$(SAM_ROOT)/CMSIS/Device/ATMEL

#-------------------------------------------------------------------------------
ifdef DEBUG
include $(SAM_ROOT)/libsam/build_gcc/debug.mk
else
include $(SAM_ROOT)/libsam/build_gcc/release.mk
endif

#-------------------------------------------------------------------------------
#  Toolchain
#-------------------------------------------------------------------------------
include $(SAM_ROOT)/libsam/build_gcc/gcc.mk


LKELF = $(CROSS_COMPILE)g++
OBJCP = $(CROSS_COMPILE)objcopy
MKDIR=mkdir -p
UPLOAD_BOSSA=bossac


#-------------------------------------------------------------------------------
#  Flags
#-------------------------------------------------------------------------------
LNK_SCRIPT=$(SAM_ROOT)/linker_scripts/gcc/flash.ld
LIBSAM_ARCHIVE=$(SAM_LIB_DIR)/libsam_sam3x8e_gcc_rel.a
UPLOAD_PORT_BASENAME=$(patsubst /dev/%,%,$(UPLOAD_PORT))

LIBRARIES += $(LIBSAM_ARCHIVE)

#-------------------------------------------------------------------------------
# .bin ------> UPLOAD TO CONTROLLER
.PHONY: install
install:
	-@echo "Touch programming port ..."
	-@$(TOUCH_PROGRAMMING_PORT) "$(UPLOAD_PORT_BASENAME)"
	-@echo "Waiting before uploading ..."
	-@sleep 1
	-@echo "Uploading ..."
	$(UPLOAD_BOSSA) $(UPLOAD_VERBOSE_FLAGS) --port=$(UPLOAD_PORT_BASENAME) -U false -e -w -v -b $(BUILD_DIR)/$(BINARY_TO_INSTALL) -R
	@echo "Done."

#-------------------------------------------------------------------------------
# .c -> .sam.o
$(BUILD_DIR)/%.sam.o: $(BUILD_DIR)/%.c
	"$(CC)" -c $(CFLAGS_SAM) $< -o $@

# .c -> .sam.o
$(BUILD_DIR)/%.sam.o: %.c
	"$(CC)" -c $(CFLAGS_SAM) $< -o $@

# .cpp -> .sam.o
$(BUILD_DIR)/%.sam.o: %.cpp
	"$(CC)" -xc++ -c $(CPPFLAGS_SAM) $< -o $@

# .sam.s -> .sam.o
%.sam.o: %.sam.s
	"$(AS)" -c $(ASFLAGS_SAM) $< -o $@

# .sam.o -> .sam.a
%.sam.a: %.sam.o
	"$(AR)" $(ARFLAGS) $@ $^
	"$(NM)" $@ > $@.txt

# .sam.a -> .elf
%.sam.elf: %.sam.a
	"$(LKELF)" -Os -Wl,--gc-sections -mcpu=cortex-m3 \
	  "-T$(LNK_SCRIPT)" "-Wl,-Map,$@.map" \
	  -o $@ \
	  "-L$(BUILD_DIR)" \
	  -lm -lgcc -mthumb -Wl,--cref -Wl,--check-sections -Wl,--gc-sections \
	  -Wl,--entry=Reset_Handler -Wl,--unresolved-symbols=report-all -Wl,--warn-common \
	  -Wl,--warn-section-align -Wl,--warn-unresolved-symbols \
	  -Wl,--start-group \
	  $^ $(LIBRARIES) \
	  -Wl,--end-group

# .elf -> .bin
%.sam.bin: %.sam.elf
	"$(OBJCP)" -O binary $< $@
