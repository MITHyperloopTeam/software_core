BUILD_DIR = ../build/lib/arduino_wrapper

INCLUDES += -Iinclude
INCLUDES += -I../build/include
INCLUDES += -I../communication/lcmtypes/built_types/
INCLUDES += -I../common/

all: prepare lib

prepare:
	@mkdir -p $(BUILD_DIR)

lib: $(BUILD_DIR)/ArduinoEmulator.sim.o
	
clean:
	$(RM) -rf $(BUILD_DIR)
	
include make/make_arduinowrapper_utils.mk