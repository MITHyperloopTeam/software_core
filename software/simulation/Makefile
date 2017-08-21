BUILD_DIR = build
SRC_DIR = src
VPATH = build

#OBJS = $(patsubst %.cpp, %.o, $(notdir $(wildcard $(SRC_DIR)/*.cpp)))
SRCS = simulate.cpp
SRCS += PodSim.cpp
SRCS += StateUtils.cpp
SRCS += Vec3d.cpp
SRCS += PodComponent.cpp
SRCS += PodComponent_ConstantForcer.cpp
SRCS += PodComponent_LevForcer.cpp
SRCS += PodComponent_BrakeSystem.cpp
SRCS += PodComponent_Pusher.cpp
SRCS += PodComponent_IMU.cpp
SRCS += PodComponent_FiducialSensor.cpp
OBJS = $(patsubst %.cpp, %.o, $(SRCS))


INCLUDES += -Iinclude
INCLUDES += -I../build/include
INCLUDES += -I../communication/lcmtypes/built_types/
INCLUDES += -I../common/

LIBS += -lyaml-cpp 
LIBS += -llcm
LIBS += -lpthread
LIBS += ../communication/lcmtypes/built_types/lcmgen_c/mithl_lcmtypes.o

MAIN = simulate

all: prepare $(MAIN)

$(MAIN): $(OBJS)
	g++ $(addprefix $(BUILD_DIR)/, $(OBJS)) $(LIBS) -g -o $@

-include $(addprefix $(BUILD_DIR)/, $(OBJS:.o=.d))

%.o: $(SRC_DIR)/%.cpp
	g++ -c -std=c++11 $(INCLUDES) -g $< -o $(BUILD_DIR)/$@

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