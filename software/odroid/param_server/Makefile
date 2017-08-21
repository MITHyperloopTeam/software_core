BUILD_DIR = build
SRC_DIR = src
VPATH = build

OBJS = $(patsubst %.cpp, %.o, $(notdir $(wildcard $(SRC_DIR)/*.cpp)))

INCLUDES += -Iinclude
INCLUDES += -I../../communication/lcmtypes/built_types/
INCLUDES += -I../../common/
INCLUDES += -I../../build/include

LIBS += -lyaml-cpp 
LIBS += -llcm
LIBS += -lpthread
LIBS += ../../communication/lcmtypes/built_types/lcmgen_c/mithl_config_t.o
LIBS += ../../communication/lcmtypes/built_types/lcmgen_c/mithl_trigger_t.o
LIBS += ../../communication/lcmtypes/built_types/lcmgen_c/mithl_pod_t.o
LIBS += ../../communication/lcmtypes/built_types/lcmgen_c/mithl_tube_t.o


MAIN = param_server

all: prepare $(MAIN)

$(MAIN): $(OBJS)
	g++ $(addprefix $(BUILD_DIR)/, $(OBJS)) $(LIBS) -o $@

-include $(addprefix $(BUILD_DIR)/, $(OBJS:.o=.d))

%.o: $(SRC_DIR)/%.cpp
	g++ -c -std=c++11 $(INCLUDES) $< -o $(BUILD_DIR)/$@
	g++ -MM -std=c++11 $(INCLUDES) $< > $(BUILD_DIR)/$*.d

prepare:
	@mkdir -p build

.PHONY: clean
clean:
	@rm -rf $(MAIN) $(BUILD_DIR)

.PHONY: sandbox
sandbox:
	@cd sandbox; make

.PHONY: clean sandbox
cleanSandbox:
	@cd sandbox; make clean
