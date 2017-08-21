BUILD_DIR = ../build/lib/common
TEST_DIR = ../build/bin

# Both simulated and normal need to know about the zcm transport
# and LCM types
INCLUDES += "-I../communication/lcmtypes/built_types/"
INCLUDES += "-I../build/include"
INCLUDES += "-I../common/"
INCLUDES += "-I../communication/zcm_transport"

# SAM needs to know how to do embedded transport
INCLUDES_SAM += "-I../externals/zcm_embed/"
# Whereas SIM needs full ZCM stack
INCLUDES_SIM += "-I../externals/zcm/"

# Corresponding SAM libraries
LIBRARIES += "../externals/zcm_embed/build/zcm.sam.a"
LIBRARIES += "../externals/zcm_embed/build/nonblocking.sam.a"
LIBRARIES += "../communication/lcmtypes/built_types/zcmgen_c/mithl_trigger_t.sam.a"
LIBRARIES_SIM += "../communication/lcmtypes/built_types/zcmgen_c/mithl_trigger_t.o"


all: prepare test_buffer_reading $(BUILD_DIR)/arduino_matrix_math.sim.o $(BUILD_DIR)/arduino_matrix_math.sam.o test_matrix_math $(BUILD_DIR)/arduino_imu_manager.sim.o $(BUILD_DIR)/arduino_imu_manager.sam.o $(BUILD_DIR)/arduino_random.sim.o $(BUILD_DIR)/arduino_random.sam.o $(BUILD_DIR)/arduino_time_manager.sim.o $(BUILD_DIR)/arduino_time_manager.sam.o $(BUILD_DIR)/IMUParse.sam.o $(BUILD_DIR)/IMUParse.sim.o $(BUILD_DIR)/BatteryI2C.sam.o $(BUILD_DIR)/BatteryI2C.sim.o 

test_matrix_math: $(BUILD_DIR)/arduino_matrix_math.sim.o
	g++ arduino_matrix_math_test.cpp $(CFLAGS) $(INCLUDES) $(INCLUDES_SIM)  -O0 -g -lpthread -lstdc++ \
	    $(BUILD_DIR)/arduino_matrix_math.sim.o -o $(TEST_DIR)/$@


test_buffer_reading: test_buffer_reading.cpp
	g++ test_buffer_reading.cpp $(CFLAGS) -std=c++11 $(INCLUDES) $(INCLUDES_SIM) -lpthread -lstdc++ -g -O0 -o $(TEST_DIR)/$@


prepare:
	@mkdir -p $(BUILD_DIR)

clean:
	$(RM) -rf $(BUILD_DIR)

include ../arduino_lib_wrapper/make/make_arduinowrapper_utils.mk