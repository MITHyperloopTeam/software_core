
BINARY_TO_INSTALL=blinky.sam.bin
BUILD_DIR=build
UPLOAD_PORT=/dev/ttyACM0

all: prepare $(BUILD_DIR)/$(BINARY_TO_INSTALL)

prepare:
	@mkdir -p $(BUILD_DIR)

clean:
	@rm -rf $(BUILD_DIR)

include ../../externals/sam/make/make_libsam_utils.mk

