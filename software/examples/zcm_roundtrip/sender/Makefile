
BINARY_TO_INSTALL=sender.sam.bin
BUILD_DIR=../build
UPLOAD_PORT=/dev/ttyACM0

INCLUDES += "-I../../../communication/zcm_transport/"
INCLUDES += "-I../../../communication/lcmtypes/built_types"

INCLUDES_SAM += "-I../../../externals/zcm_embed/"

INCLUDES_SIM += "-I../../../communication/zcm_transport/"
INCLUDES_SIM += "-I../../../communication/lcmtypes/built_types/"
INCLUDES_SIM += "-I../../../externals/zcm"


LIBRARIES += "../../../externals/zcm_embed/build/zcm.sam.a"
LIBRARIES += "../../../externals/zcm_embed/build/nonblocking.sam.a"
LIBRARIES += "../../../communication/lcmtypes/built_types/zcmgen_c/mithl_string_t.sam.a"

LIBRARIES_SIM += -lzcm
LIBRARIES_SIM += "../../../communication/lcmtypes/built_types/zcmgen_c/mithl_string_t.o"

all: prepare $(BUILD_DIR)/$(BINARY_TO_INSTALL) simul

simul: prepare $(BUILD_DIR)/sender.sim.bin

prepare:
	@mkdir -p $(BUILD_DIR)

clean:
	@rm -rf $(BUILD_DIR)

include ../../../arduino_lib_wrapper/make/make_arduinowrapper_utils.mk

