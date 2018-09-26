all: sam arduino_core arduino_libraries lcm libbot libserial zcm zcm_embed

assimp: ../build/bin/assimp
../build/bin/assimp:
	mkdir -p assimp/build
	cd assimp/build &&  cmake .. -DASSIMP_LIB_INSTALL_DIR=../../../build/lib -DASSIMP_INCLUDE_INSTALL_DIR=../../../build/include -DASSIMP_BIN_INSTALL_DIR=../../../build/bin -G "Unix Makefiles"
	cd assimp/build && make -j7 && make install 
	cd assimp/port/PyAssimp && python setup.py build && python setup.py install --prefix=../../../../build/
sam: ../build/lib/libsam_sam3x8e_gcc_rel.a
../build/lib/libsam_sam3x8e_gcc_rel.a:
	cd sam/libsam/build_gcc; 

arduino_core: sam ../build/lib/libvariant_arduino_due_x_gcc_dbg.a
../build/lib/libvariant_arduino_due_x_gcc_dbg.a:
	cd arduino_core/variants/arduino_due_x/build_gcc; make -j4

.PHONY: arduino_libraries
arduino_libraries: arduino_core
	cd arduino_libraries; make

libbot: libbot/pod-build
libbot/pod-build:
	cd libbot && make -j4

lcm: lcm-1.2.1
	cd lcm-1.2.1; ./configure --prefix=$(MIT_HL_ROOT)/build; make -j4; make install
lcm-1.2.1: lcm-1.2.1.zip
	unzip lcm-1.2.1.zip
lcm-1.2.1.zip:
	wget https://github.com/lcm-proj/lcm/releases/download/v1.2.1/lcm-1.2.1.zip

.PHONY: libserial
libserial: libserial/Makefile
	cd libserial; make; make install
libserial/Makefile:
	cd libserial; make -f Makefile.dist; ./configure --prefix=$(MIT_HL_ROOT)/build
	
.PHONY: zcm
zcm:
	cd zcm; ./waf configure --use-serial --use-udpm --prefix=$(MIT_HL_ROOT)/build; ./waf; ./waf install

zcm_embed: zcm_embed/build
zcm_embed/build:
	cd zcm_embed; make
	
director:

.PHONY: clean

clean:
	-cd sam/libsam/build_gcc; make clean
	-cd arduino_core/variants/arduino_due_x/build_gcc; make clean
	-cd arduino_libraries && make clean	
	-cd zcm && ./waf uninstall && ./waf clean
	-cd lcm && make uninstall && make clean
	-cd libbot && make clean
	-cd zcm_embed && make clean
	-cd assimp/build && make uninstall && make clean
	-cd rm -rf ../build/lib/python2.7/site-packages/pyassimp
