
BINARY_TO_INSTALL=blinky.sam.bin
BUILD_DIR=build
UPLOAD_PORT=/dev/ttyACM0

all: prepare $(BUILD_DIR)/$(BINARY_TO_INSTALL) simul

simul: prepare $(BUILD_DIR)/blinky.sim.bin

prepare:
	@mkdir -p $(BUILD_DIR)

clean:
	@rm -rf $(BUILD_DIR)

include ../../arduino_lib_wrapper/make/make_arduinowrapper_utils.mk
