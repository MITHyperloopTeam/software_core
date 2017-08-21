
BINARY_TO_INSTALL=imu_publish.sam.bin
BUILD_DIR=build
UPLOAD_PORT=/dev/ttyACM0

# Both simulated and normal need to know about the zcm transport
# and LCM types
INCLUDES += "-I../../communication/zcm_transport/"
INCLUDES += "-I../../communication/lcmtypes/built_types/"
INCLUDES += "-I../../build/include"
INCLUDES += "-I../../common/"

# SAM needs to know how to do embedded transport
INCLUDES_SAM += "-I../../externals/zcm_embed/"
# Whereas SIM needs full ZCM stack
INCLUDES_SIM += "-I../../externals/zcm/"

# Corresponding SAM libraries
LIBRARIES += "../../build/lib/zcm_transport/arduino_cobs_serial_transport.sam.o"
LIBRARIES += "../../build/lib/common/arduino_time_manager.sam.o"
LIBRARIES += "../../build/lib/common/IMUParse.sam.o"
LIBRARIES += "../../communication/lcmtypes/built_types/zcmgen_c/mithl_vectorXf_t.sam.a"
LIBRARIES += "../../communication/lcmtypes/built_types/zcmgen_c/mithl_string_t.sam.a"
LIBRARIES += "../../communication/lcmtypes/built_types/zcmgen_c/mithl_trigger_t.sam.a"
LIBRARIES += "../../externals/zcm_embed/build/zcm.sam.a"
LIBRARIES += "../../externals/zcm_embed/build/nonblocking.sam.a"

# And SIM libraries -- full LCM stack and non-SAM msg objects
LIBRARIES_SIM += "-lzcm"
LIBRARIES_SIM += "../../build/lib/common/arduino_time_manager.sim.o"
LIBRARIES_SIM += "../../build/lib/common/IMUParse.sim.o"
LIBRARIES_SIM += "../../build/lib/zcm_transport/linux_cobs_serial_transport.sim.o"
LIBRARIES_SIM += "-lserial"
LIBRARIES_SIM += "../../communication/lcmtypes/built_types/zcmgen_c/mithl_vectorXf_t.o"
LIBRARIES_SIM += "../../communication/lcmtypes/built_types/zcmgen_c/mithl_trigger_t.o"
LIBRARIES_SIM += "../../communication/lcmtypes/built_types/zcmgen_c/mithl_string_t.o"
LIBRARIES_SIM += "-lserial"

LIBRARIES_BRIDGE += "-lzcm"
LIBRARIES_BRIDGE += "../../build/lib/zcm_transport/linux_cobs_serial_transport.sim.o"
LIBRARIES_BRIDGE += "-lserial"
LIBRARIES_BRIDGE += "../../communication/lcmtypes/built_types/zcmgen_c/mithl_vectorXf_t.o"
LIBRARIES_BRIDGE += "../../communication/lcmtypes/built_types/zcmgen_c/mithl_trigger_t.o"

all: prepare $(BUILD_DIR)/$(BINARY_TO_INSTALL) simul serial_lcm_bridge_imu_publish

serial_lcm_bridge_imu_publish:
	g++ serial_lcm_bridge_imu_publish.cpp $(CFLAGS) $(INCLUDES) $(INCLUDES_SIM) -lpthread -lstdc++ \
	    $(LIBRARIES_BRIDGE) -o $(BUILD_DIR)/$@

simul: prepare $(BUILD_DIR)/imu_publish.sim.bin

prepare:
	@mkdir -p $(BUILD_DIR)

clean:
	@rm -rf $(BUILD_DIR)

include ../../arduino_lib_wrapper/make/make_arduinowrapper_utils.mk
