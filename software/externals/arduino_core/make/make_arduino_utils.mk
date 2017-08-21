#-------------------------------------------------------------------------------
# @author gizatt, following stfwi
#
# INCLUDE THIS FILE IN YOUR MAKEFILE USING THE ...
#
#   include <path/to/>make_arduino_utils.mk
#
# ... MAKEFILE DIRECTIVE.
#
# It includes make_libsam_utils.mk from the sam external, which defines
# some useful stuff described in that file.
#
# This file also does some other generically useful things:
#  - Appends to INCLUDES Arduino includes
#  - Appends to LIBRARIES Arduino libraries
#    
#-------------------------------------------------------------------------------


ARDUINO_CORE_ROOT := $(realpath $(shell dirname '$(dir $(lastword $(MAKEFILE_LIST)))'))
ARDUINO_CORE_LIB_DIR=$(ARDUINO_CORE_ROOT)/../../build/lib/arduino_core

#-------------------------------------------------------------------------------
# Related directories and files
#-------------------------------------------------------------------------------

INCLUDES += -I$(ARDUINO_CORE_ROOT)
INCLUDES += -I$(ARDUINO_CORE_ROOT)/variants/arduino_due_x

#-------------------------------------------------------------------------------
#  Flags
#-------------------------------------------------------------------------------
ARDUINO_CORE_ARCHIVE=$(ARDUINO_CORE_LIB_DIR)/libvariant_arduino_due_x_gcc_dbg.a
# not sure why this one is needed:
# see http://forum.arduino.cc/index.php?topic=79595.240;wap2
ARDUINO_SYSCALLS_ARCHIVE=$(ARDUINO_CORE_LIB_DIR)/debug_arduino_due_x/syscalls_sam3.o
LIBRARIES += $(ARDUINO_CORE_ARCHIVE) $(ARDUINO_SYSCALLS_ARCHIVE)

include $(ARDUINO_CORE_ROOT)/../sam/make/make_libsam_utils.mk

# necessary defines
CFLAGS_SAM +=  -D$(CHIP) -DF_CPU=$(F_CPU)
CPPFLAGS_SAM +=  -D$(CHIP) -DF_CPU=$(F_CPU)
