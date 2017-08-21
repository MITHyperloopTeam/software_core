
BINARY_TO_INSTALL=tempreport.sam.bin
BUILD_DIR=build
UPLOAD_PORT=/dev/ttyACM0

LIBRARIES += ../../build/lib/arduino_libraries/LiquidCrystal.sam.o
all: prepare $(BUILD_DIR)/$(BINARY_TO_INSTALL) 

simul: prepare $(BUILD_DIR)/tempreport.sim.bin

prepare:
	@mkdir -p $(BUILD_DIR)

clean:
	@rm -rf $(BUILD_DIR)

include ../../arduino_lib_wrapper/make/make_arduinowrapper_utils.mk
