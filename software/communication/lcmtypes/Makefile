LCMTYPES =	mithl_floating_base_t.lcm \
			mithl_pod_t.lcm \
			mithl_string_t.lcm \
			mithl_trigger_t.lcm \
			mithl_tube_t.lcm \
			mithl_vectorXf_t.lcm \
			mithl_velocity_t.lcm \
			mithl_config_t.lcm \
			mithl_state_t.lcm \
			mithl_net_diag_t.lcm \
			mithl_net_health_t.lcm \
			mithl_particle_filter_t.lcm \
			mithl_auto_braking_t.lcm \
			mithl_adm_config_t.lcm \
			mithl_adm_status_t.lcm \
			mithl_battery_reader_low_rate_t.lcm \
			mithl_braking_test_command_t.lcm \
			mithl_bac_auto_cmd_t.lcm \
			mithl_bac_teleop_cmd_t.lcm \
			mithl_bac_config_t.lcm \
			mithl_bac_mode_t.lcm \
			mithl_bac_state_high_rate_t.lcm \
			mithl_bac_state_low_rate_t.lcm \
			mithl_battery_status_t.lcm \
			mithl_analog_front_high_rate_t.lcm \
			mithl_analog_front_medium_rate_t.lcm \
			mithl_analog_front_low_rate_t.lcm \
			mithl_analog_rear_high_rate_t.lcm \
			mithl_analog_rear_medium_rate_t.lcm \
			mithl_analog_rear_low_rate_t.lcm \
			mithl_fiducial_config_t.lcm \
			mithl_fiducial_color_dataval_t.lcm \
			mithl_fiducial_color_row_t.lcm \
			mithl_fiducial_teach_table_t.lcm \
			mithl_fiducial_t.lcm \
			mithl_fiducial_low_rate_t.lcm \
			mithl_flight_control_high_rate_t.lcm \
			mithl_flight_control_low_rate_t.lcm \
			mithl_state_estimator_particle.lcm \
			mithl_state_estimator_particle_set.lcm \
			mithl_state_estimator_state_t.lcm \
			mithl_state_estimator_config_t.lcm \
			mithl_fsm_state_high_rate_t.lcm \
			mithl_low_speed_low_rate_t.lcm \
			mithl_gamepad_t.lcm \
			mithl_sim_pusher_cmd_t.lcm \
			mithl_spacex_telemetry_state_t.lcm \
			mithl_brake_trajectory_simple_t.lcm

BUILD_SUBDIR=built_types
LCM_CSUBDIR=$(BUILD_SUBDIR)/lcmgen_c
LCM_PSUBDIR=$(BUILD_SUBDIR)/lcmgen_p
LCM_JSUBDIR=$(BUILD_SUBDIR)/lcmgen_j
ZCM_CSUBDIR=$(BUILD_SUBDIR)/zcmgen_c

LCM_J_OUTPUTDIR=$(MIT_HL_ROOT)/build/share/java

BUILD_DIR=$(ZCM_CSUBDIR)

LCM_OFILES=$(patsubst %.lcm, $(LCM_CSUBDIR)/%.o, $(notdir $(LCMTYPES) mithl_pin_sim_t.lcm))
LCM_JFILES=$(patsubst %.lcm, $(LCM_CSUBDIR)/%.java, $(notdir $(LCMTYPES)))
ZCM_SAM_OFILES=$(patsubst %.lcm, $(ZCM_CSUBDIR)/%.sam.a, $(notdir $(LCMTYPES)))
ZCM_OFILES=$(patsubst %.lcm, $(ZCM_CSUBDIR)/%.o, $(notdir $(LCMTYPES)))
CFLAGS = -Wall
CFLAGS += "-I../../build/include/"
INCLUDES += "-I../../externals/zcm_embed/"
CFLAGS_SAM += -DZCM_EMBEDDED

VPATH+=$(ZCM_CSUBDIR)

.PRECIOUS: $(ZCM_CSUBDIR)/%.sam.cc $(ZCM_CSUBDIR)/%.c

all: lcm_c lcm_j lcm_p zcm_c

lcm_c: $(LCM_OFILES)
	ld -r $(LCM_OFILES) -o $(LCM_CSUBDIR)/mithl_lcmtypes.o

lcm_j:
	@mkdir -p $(LCM_JSUBDIR)
	lcm-gen -j $(LCMTYPES) --jpath=$(LCM_JSUBDIR)
	javac -cp $(MIT_HL_ROOT)/build/share/java/lcm.jar $(LCM_JSUBDIR)/mithl/*.java
	jar cf $(LCM_J_OUTPUTDIR)/lcmtypes_mithl.jar $(LCM_JSUBDIR)/mithl/*.class

lcm_p: 
	@mkdir -p $(LCM_PSUBDIR) 
	lcm-gen -p $(LCMTYPES) --ppath=$(LCM_PSUBDIR)

zcm_c: $(ZCM_OFILES) $(ZCM_SAM_OFILES)
	ld -r $(ZCM_OFILES) -o $(ZCM_CSUBDIR)/mithl_zcmtypes.o 
	ar -rcT $(ZCM_CSUBDIR)/mithl_zcmtypes.sam.a $(ZCM_SAM_OFILES)

$(LCM_CSUBDIR)/%.c : %.lcm
	@mkdir -p $(LCM_CSUBDIR) 
	lcm-gen -c --c-cpath $(LCM_CSUBDIR) --c-hpath $(LCM_CSUBDIR) $<

$(LCM_CSUBDIR)/%.o : $(LCM_CSUBDIR)/%.c
	gcc -c $(CFLAGS) $< -o $@

$(ZCM_CSUBDIR)/%.c : %.lcm
	@mkdir -p $(ZCM_CSUBDIR)
	zcm-gen -c --c-cpath $(ZCM_CSUBDIR) --c-hpath $(ZCM_CSUBDIR) $<

$(ZCM_CSUBDIR)/%.o : $(ZCM_CSUBDIR)/%.c
	gcc -c $(CFLAGS) $< -o $@

clean:
	rm -rf built_types

include ../../arduino_lib_wrapper/make/make_arduinowrapper_utils.mk
