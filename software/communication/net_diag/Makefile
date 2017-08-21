BUILD_DIR = ../../build/lib/net_diag

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
LIBRARIES += "../../communication/lcmtypes/built_types/zcmgen_c/mithl_net_diag_t.sam.a"
LIBRARIES += "../../communication/lcmtypes/built_types/zcmgen_c/mithl_net_health_t.sam.a"
LIBRARIES += "../../communication/lcmtypes/built_types/zcmgen_c/mithl_trigger_t.sam.a"
LIBRARIES += "../../externals/zcm_embed/build/zcm.sam.a"
LIBRARIES += "../../externals/zcm_embed/build/nonblocking.sam.a"
LIBRARIES += "../../build/lib/common/arduino_time_manager.sam.o"

# And SIM libraries -- full LCM stack and non-SAM msg objects
LIBRARIES_SIM += "-lzcm"
LIBRARIES_SIM += "../../communication/lcmtypes/built_types/zcmgen_c/mithl_net_diag_t.o"
LIBRARIES_SIM += "../../communication/lcmtypes/built_types/zcmgen_c/mithl_net_health_t.o"
LIBRARIES_SIM += "../../communication/lcmtypes/built_types/zcmgen_c/mithl_trigger_t.o"
LIBRARIES_SIM += "../../build/lib/common/arduino_time_manager.sim.o"


all: prepare $(BUILD_DIR)/net_monitor.sam.o $(BUILD_DIR)/net_monitor.sim.o run_net_monitor

prepare:
	@mkdir -p $(BUILD_DIR)
	
run_net_monitor: run_net_monitor.cpp $(BUILD_DIR)/net_monitor.sim.o
	g++ run_net_monitor.cpp $(CFLAGS) $(INCLUDES) $(INCLUDES_SIM) -lserial -lpthread -lstdc++ \
		$(BUILD_DIR)/net_monitor.sim.o \
	    $(LIBRARIES_SIM) -o $@

clean:
	$(RM) -rf $(BUILD_DIR)
	$(RM) run_net_monitor
	
include ../../arduino_lib_wrapper/make/make_arduinowrapper_utils.mk