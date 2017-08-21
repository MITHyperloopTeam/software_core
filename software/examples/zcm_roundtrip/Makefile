
BUILD_DIR=build

# Both simulated and normal need to know about the zcm transport
# and LCM types
INCLUDES += "-I../../communication/zcm_transport/"
INCLUDES += "-I../../communication/lcmtypes/built_types/"
INCLUDES += "-I../../build/include"

# Whereas SIM needs full ZCM stack
INCLUDES_SIM += "-I../../externals/zcm/"


LIBRARIES += "../../build/lib/zcm_transport/arduino_cobs_serial_transport.sam.o"

# And SIM libraries -- full LCM stack and non-SAM msg objects
LIBRARIES_SIM += -lzcm
LIBRARIES_SIM += "../../build/lib/zcm_transport/linux_cobs_serial_transport.sim.o"
LIBRARIES_SIM += "-lserial"
LIBRARIES_SIM += "../../communication/lcmtypes/built_types/zcmgen_c/mithl_string_t.o"

all: prepare serial_lcm_bridge_roundtrip

serial_lcm_bridge_roundtrip: 
	g++ serial_lcm_bridge_roundtrip.cpp $(CFLAGS) $(INCLUDES) $(INCLUDES_SIM) -lserial -lpthread -lstdc++ \
	    $(LIBRARIES_SIM) -o $(BUILD_DIR)/$@

prepare:
	@mkdir -p $(BUILD_DIR)

clean:
	@rm -rf $(BUILD_DIR)
