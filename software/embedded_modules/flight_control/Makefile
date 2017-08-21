
BINARY_TO_INSTALL=flight_control_module.sam.bin
BUILD_DIR=build
UPLOAD_PORT=/dev/mithl_flight_control

# Both simulated and normal need to know about the zcm transport
# and LCM types
INCLUDES += "-I../../communication/zcm_transport/"
INCLUDES += "-I../../communication/lcmtypes/built_types/"
INCLUDES += "-I../../communication/net_diag/"
INCLUDES += "-I../../build/include"
INCLUDES += "-I../../common/"
INCLUDES += "-I../../simulation/include"
INCLUDES += "-I../../externals/EigenArduino"

# SAM needs to know how to do embedded transport
INCLUDES_SAM += "-I../../externals/zcm_embed/"
# Whereas SIM needs full ZCM stack
INCLUDES_SIM += "-I../../externals/zcm/"

# Corresponding SAM libraries
LIBRARIES += "../../build/lib/zcm_transport/arduino_cobs_serial_transport.sam.o"
LIBRARIES += "../../build/lib/common/arduino_random.sam.o"
LIBRARIES += "../../build/lib/common/arduino_time_manager.sam.o"
LIBRARIES += "../../build/lib/common/arduino_imu_manager.sam.o"
LIBRARIES += "../../build/lib/common/arduino_matrix_math.sam.o"
LIBRARIES += "../../communication/lcmtypes/built_types/zcmgen_c/mithl_zcmtypes.sam.a"
LIBRARIES += "../../build/lib/net_diag/net_monitor.sam.o"


LIBRARIES += "../../externals/zcm_embed/build/zcm.sam.a"
LIBRARIES += "../../externals/zcm_embed/build/nonblocking.sam.a"
LIBRARIES += $(BUILD_DIR)/StateEstimator.sam.o
#LIBRARIES += $(BUILD_DIR)/BrakeController.sam.o
#LIBRARIES += $(BUILD_DIR)/ParticleFilter.sam.o
LIBRARIES += "../../build/lib/common/IMUParse.sam.o"

# And SIM libraries -- full LCM stack and non-SAM msg objects
LIBRARIES_SIM += "-lzcm"
LIBRARIES_SIM += "../../build/lib/zcm_transport/linux_cobs_serial_transport.sim.o"
LIBRARIES_SIM += "../../build/lib/common/arduino_time_manager.sim.o"
LIBRARIES_SIM += "../../build/lib/common/arduino_random.sim.o"
LIBRARIES_SIM += "../../build/lib/common/arduino_imu_manager.sim.o"
LIBRARIES_SIM += "../../build/lib/common/arduino_matrix_math.sim.o"
LIBRARIES_SIM += "-lserial"
LIBRARIES_SIM += "../../communication/lcmtypes/built_types/zcmgen_c/mithl_zcmtypes.o"
LIBRARIES_SIM += "../../build/lib/net_diag/net_monitor.sim.o"

LIBRARIES_SIM += $(BUILD_DIR)/StateEstimator.sim.o
#LIBRARIES_SIM += $(BUILD_DIR)/BrakeController.sim.o
#LIBRARIES_SIM += $(BUILD_DIR)/ParticleFilter.sim.o
LIBRARIES_SIM += "../../build/lib/common/IMUParse.sim.o"

# And bridge libraries
LIBRARIES_BRIDGE += "-lzcm"
LIBRARIES_BRIDGE += "../../build/lib/zcm_transport/linux_cobs_serial_transport.sim.o"
LIBRARIES_BRIDGE += "-lserial"
LIBRARIES_BRIDGE += "../../communication/lcmtypes/built_types/zcmgen_c/mithl_zcmtypes.o"


all: prepare serial_lcm_bridge_flight_control state_estimator $(BUILD_DIR)/$(BINARY_TO_INSTALL) simul

#particle_filter: $(BUILD_DIR)/ParticleFilter.sam.o $(BUILD_DIR)/ParticleFilter.sim.o ParticleFilter.cpp ParticleFilter.h

state_estimator: $(BUILD_DIR)/StateEstimator.sam.o $(BUILD_DIR)/StateEstimator.sim.o StateEstimator.cpp StateEstimator.h

#brake_controller: $(BUILD_DIR)/BrakeController.sam.o $(BUILD_DIR)/BrakeController.sim.o BrakeController.cpp BrakeController.h

serial_lcm_bridge_flight_control: 
	g++ serial_lcm_bridge_flight_control.cpp $(CFLAGS) $(INCLUDES) $(INCLUDES_SIM) -lserial -lpthread -lstdc++ \
	    $(LIBRARIES_BRIDGE) -o $(BUILD_DIR)/$@

simul: prepare state_estimator $(BUILD_DIR)/flight_control_module.sim.bin

prepare:
	@mkdir -p $(BUILD_DIR)

clean:
	@rm -rf $(BUILD_DIR)

include ../../arduino_lib_wrapper/make/make_arduinowrapper_utils.mk

