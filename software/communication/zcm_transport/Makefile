BUILD_DIR = ../../build/lib/zcm_transport

# Both simulated and normal need to know about the zcm transport
# and LCM types
INCLUDES += "-I../../communication/lcmtypes/built_types/"
INCLUDES += "-I../../build/include"
INCLUDES += "-I../../common/"

# SAM needs to know how to do embedded transport
INCLUDES_SAM += "-I../../externals/zcm_embed/"
# Whereas SIM needs full ZCM stack
INCLUDES_SIM += "-I../../externals/zcm/"

# Corresponding SAM libraries
LIBRARIES += "../../externals/zcm_embed/build/zcm.sam.a"
LIBRARIES += "../../externals/zcm_embed/build/nonblocking.sam.a"

# And SIM libraries -- full LCM stack and non-SAM msg objects
LIBRARIES_SIM += "-lzcm"

all: prepare $(BUILD_DIR)/linux_cobs_serial_transport.sim.o $(BUILD_DIR)/arduino_cobs_serial_transport.sam.o

prepare:
	@mkdir -p $(BUILD_DIR)

clean:
	$(RM) -rf $(BUILD_DIR)
	
include ../../arduino_lib_wrapper/make/make_arduinowrapper_utils.mk