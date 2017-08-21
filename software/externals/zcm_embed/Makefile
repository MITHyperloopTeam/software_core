BUILD_DIR=build

VPATH=zcm
INCLUDES += "-I./"
CFLAGS_SAM += -DZCM_EMBEDDED

all: prepare $(BUILD_DIR)/zcm.sam.a $(BUILD_DIR)/nonblocking.sam.a

prepare:
	@mkdir -p $(BUILD_DIR)

clean:
	@rm -rf $(BUILD_DIR)

include ../../arduino_lib_wrapper/make/make_arduinowrapper_utils.mk