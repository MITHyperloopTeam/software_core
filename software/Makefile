default: all

all: lcmtypes arduino_lib_wrapper common communication examples embedded_modules simulation odroid

.PHONY: common
common:
	cd common && make

.PHONY: lcmtypes
lcmtypes:
	cd communication/lcmtypes && make

.PHONY: communication
communication:
	cd communication && make

.PHONY: arduino_lib_wrapper
arduino_lib_wrapper:
	cd arduino_lib_wrapper; make

.PHONY: examples
examples: arduino_lib_wrapper 
	cd examples; make

.PHONY: embedded_modules
embedded_modules: arduino_lib_wrapper 
	cd embedded_modules; make

.PHONY: simulation
simulation:
	cd simulation && make

.PHONY: externals
externals:
	cd externals; make;

.PHONY: odroid
odroid:
	cd odroid; make;

.PHONY: common
common:
	cd common; make;
