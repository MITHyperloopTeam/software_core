#-------------------------------------------------------------------------------
# @author gizatt, following stfwi
#
# INCLUDE THIS FILE IN YOUR MAKEFILE USING THE ...
#
#   include <path/to/>make_arduinowrapper_utils.mk
#
# ... MAKEFILE DIRECTIVE.
#
# It includes make_libsam_utils.mk from the sam external, which defines
# some useful stuff described in that file.
# It includes make_arduino_utils.mk from the arduino core external.
#
# This file also does some other generically useful things:
#  - Appends to INCLUDES the Arduino Lib Wrapper headers.
#  - Appends to LIBRARIES the Arduino lib wrapper libraries.
#    
#-------------------------------------------------------------------------------

ARDUINO_WRAPPER_ROOT := $(realpath $(shell dirname '$(dir $(lastword $(MAKEFILE_LIST)))'))
ARDUINO_WRAPPER_LIB_DIR=$(ARDUINO_WRAPPER_ROOT)/../build/lib/arduino_wrapper/

#-------------------------------------------------------------------------------
# Related directories and files
#-------------------------------------------------------------------------------

INCLUDES += -I$(ARDUINO_WRAPPER_ROOT)

#-------------------------------------------------------------------------------
#  Flags
#-------------------------------------------------------------------------------
#LIBRARIES += $(ARDUINO_WRAPPER_LIB_DIR)/ArduinoLibWrapperArduino.sam.a
LIBRARIES_SIM += $(ARDUINO_WRAPPER_LIB_DIR)/ArduinoEmulator.sim.o
LIBRARIES_SIM += "-llcm"
LIBRARIES_SIM += "-lpthread"
LIBRARIES_SIM += "$(ARDUINO_WRAPPER_ROOT)/../communication/lcmtypes/built_types/lcmgen_c/mithl_pin_sim_t.o"

include $(ARDUINO_WRAPPER_ROOT)/../externals/arduino_core/make/make_arduino_utils.mk
include $(ARDUINO_WRAPPER_ROOT)/make/gcc.mk

CPPFLAGS_SAM += -DCOMPILE_FOR_ARDUINO $(EXTRA_DEFINE_1)
CFLAGS_SAM += -DCOMPILE_FOR_ARDUINO $(EXTRA_DEFINE_1)

CPPFLAGS_SIM += -DCOMPILE_FOR_SIMUL $(EXTRA_DEFINE_1)
CFLAGS_SIM += -DCOMPILE_FOR_SIMUL $(EXTRA_DEFINE_1)
#-------------------------------------------------------------------------------
# Sim Build Rules
#-------------------------------------------------------------------------------
# .c -> .sim.o
$(BUILD_DIR)/%.sim.o: %.c
	"$(CC_SIM)" -g -c $(CFLAGS_SIM) $< -o $@

# .cpp -> .sim.o
$(BUILD_DIR)/%.sim.o: %.cpp
	"$(CC_SIM)" -g -xc++ -c $(CPPFLAGS_SIM) $< -o $@

# .sim.s -> .sim.o
%.sim.o: %.sim.s
	"$(AS_SIM)" -g -c $(ASFLAGS_SIM) $< -o $@

# .sim.o -> .sim.a
%.sim.a: %.sim.o
	"$(AR_SIM)" $(ARFLAGS) $@ $^
	"$(NM_SIM)" $@ > $@.txt

# .sim.a -> .elf
%.sim.bin: %.sim.a
	"$(CXX_SIM)" \
	  -o $@ \
	  -L$(BUILD_DIR) \
	  -Wall -lc \
	  $^ $(LIBRARIES_SIM)
