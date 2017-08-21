
BINARY_TO_INSTALL=low_speed_module.sam.bin
BUILD_DIR=build
UPLOAD_PORT=/dev/mithl_low_speed

# Both simulated and normal need to know about the zcm transport
# and LCM types
INCLUDES += "-I../../communication/zcm_transport/"
INCLUDES += "-I../../communication/lcmtypes/built_types/"
INCLUDES += "-I../../communication/net_diag/"
INCLUDES += "-I../../build/include"
INCLUDES += "-I../../common/"

# SAM needs to know how to do embedded transport
INCLUDES_SAM += "-I../../externals/zcm_embed/"
# Whereas SIM needs full ZCM stack
INCLUDES_SIM += "-I../../externals/zcm/"

# Corresponding SAM libraries
LIBRARIES += "../../build/lib/zcm_transport/arduino_cobs_serial_transport.sam.o"
LIBRARIES += "../../communication/lcmtypes/built_types/zcmgen_c/mithl_vectorXf_t.sam.a"
LIBRARIES += "../../communication/lcmtypes/built_types/zcmgen_c/mithl_string_t.sam.a"
LIBRARIES += "../../communication/lcmtypes/built_types/zcmgen_c/mithl_zcmtypes.sam.a"

LIBRARIES += "../../communication/lcmtypes/built_types/zcmgen_c/mithl_auto_braking_t.sam.a"

LIBRARIES += "../../externals/zcm_embed/build/zcm.sam.a"
LIBRARIES += "../../externals/zcm_embed/build/nonblocking.sam.a"
LIBRARIES += "../../build/lib/common/arduino_time_manager.sam.o"
LIBRARIES += $(BUILD_DIR)/RoboteqADMController.sam.o
LIBRARIES += $(BUILD_DIR)/BrakeActuatorController.sam.o

LIBRARIES += "../../build/lib/net_diag/net_monitor.sam.o"

# And SIM libraries -- full LCM stack and non-SAM msg objects
LIBRARIES_SIM += "-lzcm"
LIBRARIES_SIM += "../../build/lib/zcm_transport/linux_cobs_serial_transport.sim.o"
LIBRARIES_SIM += "-lserial"
LIBRARIES_SIM += "../../communication/lcmtypes/built_types/zcmgen_c/mithl_zcmtypes.o"
LIBRARIES_SIM += "../../build/lib/common/arduino_time_manager.sim.o"
LIBRARIES_SIM += $(BUILD_DIR)/RoboteqADMController.sim.o
LIBRARIES_SIM += $(BUILD_DIR)/BrakeActuatorController.sim.o
LIBRARIES_SIM += "../../build/lib/net_diag/net_monitor.sim.o"

# And Bridge libraries
LIBRARIES_BRIDGE += "-lzcm"
LIBRARIES_BRIDGE += "../../build/lib/zcm_transport/linux_cobs_serial_transport.sim.o"
LIBRARIES_BRIDGE += "-lserial"
LIBRARIES_BRIDGE += "../../communication/lcmtypes/built_types/zcmgen_c/mithl_zcmtypes.o"


all: prepare $(BUILD_DIR)/RoboteqADMController.sam.o $(BUILD_DIR)/RoboteqADMController.sim.o \
     $(BUILD_DIR)/BrakeActuatorController.sam.o $(BUILD_DIR)/BrakeActuatorController.sim.o \
     serial_lcm_bridge_low_speed  $(BUILD_DIR)/$(BINARY_TO_INSTALL) simul

serial_lcm_bridge_low_speed:
	g++ serial_lcm_bridge_low_speed.cpp $(CFLAGS) $(INCLUDES) $(INCLUDES_SIM) -lserial -lpthread -lstdc++ \
	    $(LIBRARIES_BRIDGE) -o $(BUILD_DIR)/$@

simul: prepare $(BUILD_DIR)/RoboteqADMController.sim.o \
	   $(BUILD_DIR)/BrakeActuatorController.sim.o \
	   $(BUILD_DIR)/low_speed_module.sim.bin

prepare:
	@mkdir -p $(BUILD_DIR)

clean:
	@rm -rf $(BUILD_DIR)

include ../../arduino_lib_wrapper/make/make_arduinowrapper_utils.mk

