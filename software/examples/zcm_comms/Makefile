
BINARY_TO_INSTALL=zcm_comms.sam.bin
BUILD_DIR=build
UPLOAD_PORT=/dev/ttyACM0

INCLUDES += "-I../../communication/zcm_transport/"
INCLUDES += "-I../../communication/lcmtypes/built_types"

INCLUDES_SAM += "-I../../externals/zcm_embed/"

INCLUDES_SIM += "-I../../communication/zcm_transport/"
INCLUDES_SIM += "-I../../build/include"


LIBRARIES += "../../externals/zcm_embed/build/zcm.sam.a"
LIBRARIES += "../../externals/zcm_embed/build/nonblocking.sam.a"
LIBRARIES += "../../build/lib/zcm_transport/arduino_cobs_serial_transport.sam.o"
LIBRARIES += "../../communication/lcmtypes/built_types/zcmgen_c/mithl_vectorXf_t.sam.a"
LIBRARIES += "../../communication/lcmtypes/built_types/zcmgen_c/mithl_string_t.sam.a"

LIBRARIES_SIM += -lzcm
LIBRARIES_SIM += "../../build/lib/zcm_transport/linux_cobs_serial_transport.sim.o"
LIBRARIES_SIM += "-lserial"
LIBRARIES_SIM += "../../communication/lcmtypes/built_types/zcmgen_c/mithl_vectorXf_t.o"
LIBRARIES_SIM += "../../communication/lcmtypes/built_types/zcmgen_c/mithl_string_t.o"

LIBRARIES_BRIDGE += -lzcm
LIBRARIES_BRIDGE += "../../build/lib/zcm_transport/linux_cobs_serial_transport.sim.o"
LIBRARIES_BRIDGE += "-lserial"
LIBRARIES_BRIDGE += "../../communication/lcmtypes/built_types/zcmgen_c/mithl_vectorXf_t.o"
LIBRARIES_BRIDGE += "../../communication/lcmtypes/built_types/zcmgen_c/mithl_string_t.o"

all: prepare zcm_serial_monitor $(BUILD_DIR)/$(BINARY_TO_INSTALL)

zcm_serial_monitor: 
	g++ zcm_serial_monitor.cpp $(CFLAGS) $(INCLUDES) $(INCLUDES_SIM) -lserial -lpthread -lstdc++ \
	    $(LIBRARIES_BRIDGE) -o $(BUILD_DIR)/$@

simul: prepare $(BUILD_DIR)/zcm_comms.sim.bin

prepare:
	@mkdir -p $(BUILD_DIR)

clean:
	@rm -rf $(BUILD_DIR)

include ../../arduino_lib_wrapper/make/make_arduinowrapper_utils.mk

