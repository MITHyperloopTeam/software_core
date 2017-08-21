all: recieveLCM

recieveLCM: recieveLCM.cpp
	g++ recieveLCM.cpp -I../../build/include -llcm \
	../../communication/lcmtypes/built_types/lcmgen_c/mithl_vectorXf_t.o \
	-o $@

clean:
	@rm recieveLCM