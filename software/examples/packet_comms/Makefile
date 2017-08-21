
BINARY_TO_INSTALL=packet_comms.sam.bin
BUILD_DIR=build
UPLOAD_PORT=/dev/ttyACM0

INCLUDES += -I$(MIT_HL_ROOT)/build/include

all: prepare $(BUILD_DIR)/$(BINARY_TO_INSTALL) serial_monitor

serial_monitor: 
	gcc serial_monitor.cpp $(CPPFLAGS_SIM) -lserial -lpthread -lstdc++ -o $(BUILD_DIR)/$@

simul: prepare $(BUILD_DIR)/packet_comms.sim.bin

prepare:
	@mkdir -p $(BUILD_DIR)

clean:
	@rm -rf $(BUILD_DIR)

include ../../arduino_lib_wrapper/make/make_arduinowrapper_utils.mk
