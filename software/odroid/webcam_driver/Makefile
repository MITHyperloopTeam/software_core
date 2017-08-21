BUILD_DIR = build

INCLUDES += -I../../communication/lcmtypes/built_types/
INCLUDES += -I../../common/
INCLUDES += -I../../build/include

CFLAGS += `pkg-config --cflags opencv`
LIBS += `pkg-config --libs opencv`
LIBS += -lyaml-cpp 
LIBS += -lpthread
LIBS += ../../build/lib/liblcmtypes_bot2-core.a
LIBS += -llcm

all: prepare webcam_driver

webcam_driver:
	g++ -O2 $(CFLAGS) $(INCLUDES) webcam_driver.cpp $(LIBS) -o $(BUILD_DIR)/$@

%.o: $(SRC_DIR)/%.cpp
	g++ -c -std=c++11 $(INCLUDES) $< -o $(BUILD_DIR)/$@
	g++ -MM -std=c++11 $(INCLUDES) $< > $(BUILD_DIR)/$*.d

prepare:
	@mkdir -p build

.PHONY: clean
clean:
	@rm -rf $($(BUILD_DIR)
