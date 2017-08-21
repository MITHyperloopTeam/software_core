
BINARY_TO_INSTALL=battery_reader_module.sam.bin
BUILD_DIR=build
UPLOAD_PORT=/dev/mithl_battery_reader

# Both simulated and normal need to know about the zcm transport
# and LCM types
INCLUDES += "-I../../communication/zcm_transport/"
INCLUDES += "-I../../communication/lcmtypes/built_types/"
INCLUDES += "-I../../communication/net_diag/"
INCLUDES += "-I../../common/"
INCLUDES += "-I../../build/include"

# SAM needs to know how to do embedded transport
INCLUDES_SAM += "-I../../externals/zcm_embed/"
# Whereas SIM needs full ZCM stack
INCLUDES_SIM += "-I../../externals/zcm/"

# Corresponding SAM libraries
LIBRARIES += "../../build/lib/zcm_transport/arduino_cobs_serial_transport.sam.o"
LIBRARIES += "../../build/lib/common/arduino_time_manager.sam.o"
LIBRARIES += "../../build/lib/arduino_libraries/Wire.sam.o"
LIBRARIES += "../../build/lib/common/BatteryI2C.sam.o"
LIBRARIES += "../../build/lib/common/IMUParse.sam.o"
LIBRARIES += "../../build/lib/net_diag/net_monitor.sam.o"
LIBRARIES += "../../communication/lcmtypes/built_types/zcmgen_c/mithl_zcmtypes.sam.a"
LIBRARIES += "../../externals/zcm_embed/build/zcm.sam.a"
LIBRARIES += "../../externals/zcm_embed/build/nonblocking.sam.a"

# And SIM libraries -- full LCM stack and non-SAM msg objects
LIBRARIES_SIM += "-lzcm"
LIBRARIES_SIM += "../../build/lib/zcm_transport/linux_cobs_serial_transport.sim.o"
LIBRARIES_SIM += "../../build/lib/common/arduino_time_manager.sim.o"
LIBRARIES_SIM += "../../build/lib/common/BatteryI2C.sim.o"
LIBRARIES_SIM += "../../build/lib/common/IMUParse.sim.o"
LIBRARIES_SIM += "../../build/lib/net_diag/net_monitor.sim.o"
LIBRARIES_SIM += "-lserial"
LIBRARIES_SIM += "../../communication/lcmtypes/built_types/zcmgen_c/mithl_zcmtypes.o"

LIBRARIES_BRIDGE += "-lzcm"
LIBRARIES_BRIDGE += "../../build/lib/zcm_transport/linux_cobs_serial_transport.sim.o"
LIBRARIES_BRIDGE += "-lserial"
LIBRARIES_BRIDGE += "../../communication/lcmtypes/built_types/zcmgen_c/mithl_zcmtypes.o"

all: prepare serial_lcm_bridge_battery_reader $(BUILD_DIR)/$(BINARY_TO_INSTALL) simul

serial_lcm_bridge_battery_reader: 
	g++ serial_lcm_bridge_battery_reader.cpp $(CFLAGS) $(INCLUDES) $(INCLUDES_SIM) -lserial -lpthread -lstdc++ \
	    $(LIBRARIES_BRIDGE) -o $(BUILD_DIR)/$@

simul: prepare $(BUILD_DIR)/battery_reader_module.sim.bin

prepare:
	@mkdir -p $(BUILD_DIR)

clean:
	@rm -rf $(BUILD_DIR)

include ../../arduino_lib_wrapper/make/make_arduinowrapper_utils.mk

