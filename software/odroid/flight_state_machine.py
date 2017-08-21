#!/usr/bin/env python

import lcm
import lcm_utils
import time
from mithl import string_t, trigger_t, floating_base_t, state_t
from mithl import bac_state_high_rate_t, bac_mode_t, bac_auto_cmd_t
from mithl import flight_control_high_rate_t, state_estimator_particle_set
from mithl import fsm_state_high_rate_t
from mithl import brake_trajectory_simple_t
from threading import Lock
import numpy as np

# transition ARM to LAUNCH on >=1second of >0.4m/s/s accel, after 0.01 alpha low-pass filter
# transition LAUNCH to FLIGHT on >0.25second of <0.4m/s/s accel, after 0.01 alpha low-pass filter
class FlightStateMachine():
    ''' Does flight control logic. '''    
    IMU_WATCHDOG_PERIOD = 1.0
    BAC_HIGH_RATE_WATCHDOG_PERIOD = 1.0
    BAC_MODE_WATCHDOG_PERIOD = 1.0
    SE_WATCHDOG_PERIOD = 1.0
    ACCEL_BIAS_N_SAMPLES = 100

    ESTOP_PUBLISH_PERIOD = 0.2
    SETPOINT_COMMAND_PERIOD = 0.1
    MODE_SWITCH_COMMAND_PERIOD = 0.2
    CONFIG_PUBLISH_PERIOD = 0.33

    ARM_SETPOINT = 6.5
    ARM_SETPOINT_LOWER = 5.25
    ARM_MIN_PRESSURE = 1300

    SHUTDOWN_PRESSURE = 100
    SHUTDOWN_SETPOINT = 0.0
    # After last heard an estop transition command,
    # can't leave for this long
    ESTOP_STICKINESS = 1.0

    def __init__(self, lc):
        self.lc = lc

        self.brake_config = brake_trajectory_simple_t()
        self.brake_config.accel_alpha = 0.1
        self.brake_config.accel_bias_alpha = 0.0001
        self.brake_config.launch_start_accel_duration = 1.0
        self.brake_config.launch_end_accel_duration = 1.0
        self.brake_config.launch_start_accel_threshold = 0.4
        self.brake_config.launch_end_accel_threshold = 0.4
        self.brake_config.end_of_launch_timeout = 10.0
        self.brake_config.cruise_setpoint = 5.3
        self.brake_config.flight_softstop_timeout = 20.0
        self.brake_config.soft_stop_setpoint = 1.0
        self.brake_config.brake_trajectory_start_close_time = 0
        self.brake_config.brake_trajectory_start_braking_speed = 1000
        self.brake_config.brake_trajectory_close_rate_inch_per_second = 100
        self.config_pub = lc.subscribe("FSM_CONFIG_SET", self.handle_config_set)

        temp_state = state_t()

        self.states = {
            "ESTOP" : temp_state.ESTOP,
            "SHUTDOWN" : temp_state.SHUTDOWN,
            "SOFT_STOP" : temp_state.SOFT_STOP,
            "ARMING" : temp_state.ARMING,
            "ARM" : temp_state.ARM,
            "LAUNCH" : temp_state.LAUNCH,
            "FLIGHT" : temp_state.FLIGHT,
            "PRE_DRIVE" : temp_state.PRE_DRIVE,
            "DRIVE" : temp_state.DRIVE,
            "TELEOP" : temp_state.TELEOP
        }
        
        self.state_name_map = {}
        for key in self.states.keys():
            self.state_name_map[self.states[key]] = key

        self.state = self.states["ESTOP"]
        self.requested_state_sub = lc.subscribe("FSM_REQUESTED_STATE", self.handle_requested_state) 

        self.estop_sub = lc.subscribe("_ESTOP", self.handle_estop)

        self.pod_imu_sub = lc.subscribe("_FC_OH", self.flight_control_high_rate_handler)
        self.last_fc_high_rate_recv_time = None
        # quick and dirty mode switch estimator
        self.acc_estimator_lock = Lock()
        self.acceleration_est = 0.
        self.acceleration_bias_est = 0.
        self.acceleration_bias_est_sum = 0.
        self.taring_counter = 0

        self.bac_state_high_rate_sub = lc.subscribe("_BAC_STATE_H", self.handle_bac_state_high_rate)
        self.last_bac_high_rate_recv_time = None
        self.brake_width = 0.
        self.brake_pressure = 0.
        
        # some brake traj state
        self.doing_braking = False
        self.doing_braking_setpoint = 0.0
        self.last_brake_trajectory_update_time = time.time() - 1000


        self.bac_mode_sub = lc.subscribe("_BAC_MODE_STATE", self.handle_bac_mode)
        self.last_bac_mode_recv_time = None
        self.bac_mode = None

        self.se_sub = lc.subscribe("_FC_SE", self.handle_state_estimate)
        self.last_se_recv_time = None
        self.particles = None
        self.stopped = None 
        self.se_lock = Lock()

        ##self.tare_sub = lc.subscribe("TARE", self.handle_brake_tare)

        self.last_print_time = time.time()
        self.last_publish_time = time.time()
        self.last_estop_publish_time = time.time()
        self.last_commanded_setpoint = time.time()
        self.last_commanded_mode_switch = time.time()
        self.last_publish_fsm_state = time.time()
        self.last_requested_to_go_to_estop = time.time()
        self.last_published_config = time.time()

        self.last_accel_below_threshold = time.time()
        self.last_accel_above_threshold = time.time()

    def update(self):
        # first, sanity check age of our data. If we're not getting all the data we want
        # given the states we're in, panic
        imu_watchdog_triggered = False
        bac_high_rate_watchdog_triggered = False
        bac_mode_watchdog_triggered = False

        now = time.time()
        # WATCHDOG LOGIC
        if (not self.last_fc_high_rate_recv_time or now - self.last_fc_high_rate_recv_time > self.IMU_WATCHDOG_PERIOD):
            imu_watchdog_triggered = True
        if (not self.last_bac_high_rate_recv_time or now - self.last_bac_high_rate_recv_time > self.BAC_HIGH_RATE_WATCHDOG_PERIOD):
            bac_high_rate_watchdog_triggered = True
        if (not self.last_bac_mode_recv_time or now - self.last_bac_mode_recv_time > self.BAC_MODE_WATCHDOG_PERIOD):
            bac_mode_watchdog_triggered = True
        if (imu_watchdog_triggered or bac_high_rate_watchdog_triggered or bac_mode_watchdog_triggered):
            self.state = self.states["ESTOP"]

        # if the BAC estop'd itself, we should ESTOP unless we're trying to arm it
        if (self.state != self.states["ARMING"] and self.state != self.states["TELEOP"]) and self.bac_mode == "MODE_ESTOP":
            self.state = self.states["ESTOP"]


        if self.state == self.states["ESTOP"]:
            # make sure everyone knows
            self.desired_mode = "MODE_ESTOP"
            self.desired_setpoint = None
            if (now - self.last_estop_publish_time > self.ESTOP_PUBLISH_PERIOD):
                self.last_estop_publish_time = now
                self.publish_estop()


        elif self.state == self.states["SHUTDOWN"]:
            if self.brake_pressure and self.brake_pressure > self.SHUTDOWN_PRESSURE:
                self.desired_mode = "MODE_AUTO"
                self.desired_setpoint = self.SHUTDOWN_SETPOINT
            else:
                print "SHUTDOWN COMPLETE, BRAKE PRESSURE %f BELOW %F" % (self.brake_pressure, self.SHUTDOWN_PRESSURE)
                self.desired_mode = "MODE_ESTOP"
                self.desired_setpoint = None
                self.state = self.states["ESTOP"]



        elif self.state == self.states["SOFT_STOP"]:
            if (self.stopped):
                print "END OF FLIGHT DETECTED BY STOP DETECTOR, GOING TO SHUTDOWN"
                self.state = self.states["SHUTDOWN"]
                self.desired_mode = "MODE_AUTO"
                self.desired_setpoint = 0
            else:
                self.desired_mode   = "MODE_AUTO"
                self.desired_setpoint = self.brake_config.soft_stop_setpoint


        elif self.state == self.states["ARMING"]:
            self.desired_mode = "MODE_AUTO"
            self.desired_setpoint = self.ARM_SETPOINT

            if (self.bac_mode == "MODE_AUTO" and self.brake_width > self.ARM_SETPOINT_LOWER and self.brake_pressure > self.ARM_MIN_PRESSURE):
                print "DONE ARMING"
                self.state = self.states["ARM"]



        elif self.state == self.states["ARM"]:
            self.desired_mode = "MODE_AUTO"
            self.desired_setpoint = self.ARM_SETPOINT
            self.doing_braking = False
            self.doing_braking_setpoint = 0
            if (now - self.last_accel_below_threshold > self.brake_config.launch_start_accel_duration):
                self.start_of_launch = time.time()
                print "LAUNCH DETECTED, GOING TO LAUNCH"
                self.state = self.states["LAUNCH"]



        elif self.state == self.states["LAUNCH"]:
            self.desired_mode = "MODE_AUTO"
            self.desired_setpoint = self.ARM_SETPOINT

            if (now - self.start_of_launch > self.brake_config.end_of_launch_timeout):
                print "LAUNCH TIMEOUT REACHED, GOING TO FLIGHT"
                self.start_of_flight = time.time()
                self.state = self.states["FLIGHT"]

            if (now - self.last_accel_above_threshold > self.brake_config.launch_end_accel_duration):
                self.start_of_flight = time.time()
                print "END OF LAUNCH DETECTED BY IMU, GOING TO FLIGHT"
                self.state = self.states["FLIGHT"]



        elif self.state == self.states["FLIGHT"]:
            self.acc_estimator_lock.acquire()
            
            # unless we verify all the appropriate messages are in, 
            # we'll want to ESTOP
            self.desired_mode = "MODE_ESTOP"
            self.desired_setpoint = None

            if (not self.last_fc_high_rate_recv_time or time.time() - self.last_fc_high_rate_recv_time > self.IMU_WATCHDOG_PERIOD):
                print "CAN'T HEAR STOP DETECTOR, GOING TO ESTOP"
                self.state = self.states["ESTOP"]
            elif (not self.last_se_recv_time or time.time() - self.last_se_recv_time > self.SE_WATCHDOG_PERIOD):
                print "CAN'T HEAR SE, GOING TO ESTOP"
                self.state = self.states["ESTOP"]
            else:
                if (self.stopped):
                    print "END OF FLIGHT DETECTED BY STOP DETECTOR, GOING TO SHUTDOWN"
                    self.state = self.states["SHUTDOWN"]
                    self.desired_mode = "MODE_AUTO"
                    self.desired_setpoint = 0
                else:
                    self.desired_mode = "MODE_AUTO"
                    if (time.time() - self.start_of_launch > self.brake_config.flight_softstop_timeout):
                        self.state = self.states["SOFT_STOP"]
                        print "SOFT STOPPING BY TIMEOUT"
                    else:
                        # if we're safely past start of cruising
                        if (time.time() - self.start_of_launch > self.brake_config.brake_trajectory_start_close_time
                           and self.get_est_state()[1] < self.brake_config.brake_trajectory_start_braking_speed):
                            if self.doing_braking == False:
                                self.doing_braking = True
                                self.doing_braking_setpoint = self.brake_config.cruise_setpoint
                                self.last_brake_trajectory_update_time = time.time()
                            else:
                                dt = time.time() - self.last_brake_trajectory_update_time
                                self.last_brake_trajectory_update_time = time.time()
                                self.doing_braking_setpoint -= dt * self.brake_config.brake_trajectory_close_rate_inch_per_second
                                if (self.doing_braking_setpoint < 0):
                                    self.doing_braking_setpoint = 0
                            self.desired_setpoint = self.doing_braking_setpoint
                        else:
                            self.desired_setpoint = self.brake_config.cruise_setpoint

                        #cur_pos = self.get_est_state()[0]
                        #dist_traversed = cur_pos
                        #if dist_traversed >= self.brake_config.brake_trajectory_hard_brake_dist:
                        #    print "BRAKING BY PIECEWISE TRAJECTORY: HARD BRAKE"
                        #    self.desired_setpoint = self.brake_config.brake_trajectory_hard_brake_setpoint
                        #elif dist_traversed >= self.brake_config.brake_trajectory_light_brake_dist:
                        #    print "BRAKING BY PIECEWISE TRAJECTORY: LIGHT BRAKE"
                        #    self.desired_setpoint = self.brake_config.brake_trajectory_light_brake_setpoint
                        #elif dist_traversed < self.brake_config.brake_trajectory_light_brake_dist:
                        #    print "BRAKING BY PIECEWISE TRAJECTORY: CRUISE"
                        #    self.desired_setpoint = self.brake_config.cruise_setpoint
#                    elif (time.time() - self.start_of_launch > self.brake_config.brake_trajectory_brake_start_time):
#                        self.desired_setpoint = self.brake_config.brake_trajectory_setpoint
#                        print "BRAKING BY TRAJECTORY"
#                    else:
#                        self.desired_setpoint =  self.brake_config.cruise_setpoint

            self.acc_estimator_lock.release()



        elif self.state == self.states["PRE_DRIVE"]:
            print "I DON'T KNOW HOW TO DO THIS"
            self.state = self.states["ESTOP"]            



        elif self.state == self.states["DRIVE"]:
            print "HOW DID I EVEN GET HERE"
            self.state = self.states["ESTOP"]



        elif self.state == self.states["TELEOP"]:
            # chill
            self.desired_mode = None
            self.desired_setpoint = None




        if (self.bac_mode is not None and self.desired_mode is not None and self.bac_mode != self.desired_mode and now - self.last_commanded_mode_switch > self.MODE_SWITCH_COMMAND_PERIOD):
            self.last_commanded_mode_switch = now
            self.publish_bac_mode(self.desired_mode)
        
        if (self.desired_mode == "MODE_AUTO" and 
            self.desired_setpoint is not None and 
            now - self.last_commanded_setpoint > self.SETPOINT_COMMAND_PERIOD):
            self.last_commanded_setpoint = now
            self.publish_bac_setpoint(self.desired_setpoint)


        if (time.time() - self.last_print_time > 0.25):
            self.last_print_time = time.time()
            if (imu_watchdog_triggered):
                print "IMU Watchdog triggered!"
            if (bac_mode_watchdog_triggered):
                print "BAC Mode watchdog triggered!"
            if (bac_high_rate_watchdog_triggered):
                print "BAC High Rate watchdog triggered!"
            print "FSM CURRENT STATE: ", self.state_name_map[self.state]

        if (time.time() - self.last_publish_time > 0.05):
            self.last_publish_time = time.time()
            self.publish_state()

        if (time.time() - self.last_publish_fsm_state > 0.05):
            self.last_publish_fsm_state = time.time()
            self.publish_fsm_internal_state()

        if (time.time() - self.last_published_config > self.CONFIG_PUBLISH_PERIOD):
            self.last_published_config = time.time()
            self.lc.publish("FSM_CONFIG", self.brake_config.encode())

    def publish_bac_mode(self, mode_str):
        msg = bac_mode_t()
        msg.utime = time.time()*1000*1000
        if (mode_str == "MODE_ESTOP"):
            msg.mode = msg.MODE_ESTOP
        elif (mode_str == "MODE_TELEOP"):
            msg.mode = msg.MODE_TELEOP
        elif (mode_str == "MODE_AUTO"):
            msg.mode = msg.MODE_AUTO
        else:
            print "Trying to send invalid BAC mode %s" % mode_str
            return
        self.lc.publish("_BAC_MODE_SET", msg.encode()) 

    def publish_bac_setpoint(self, setpoint):
        msg = bac_auto_cmd_t()
        msg.utime = time.time()*1000*1000
        msg.setpoint = setpoint
        self.lc.publish("_BAC_AUTO_CMD_SET", msg.encode())

    def publish_estop(self):
        msg = trigger_t()
        msg.utime = time.time()*1000*1000
        self.lc.publish("_ESTOP", msg.encode())

    def publish_fsm_internal_state(self):
        msg = fsm_state_high_rate_t()
        msg.utime = int(time.time() * 1000 * 1000)
        msg.acceleration_est = self.acceleration_est
        self.lc.publish("FSM_OH", msg.encode())

    def publish_state(self):
        msg = state_t()
        msg.utime = int(time.time() * 1000 * 1000)
        msg.currentState = self.state
        self.lc.publish("FSM_STATE", msg.encode())

    def try_state_transition(self, newState):
        if (newState not in self.state_name_map.keys()):
            print "Requested unknown state %d, ignoring" % newState
            return

        print "Attempting transition %s to %s..." % (self.state_name_map[self.state], self.state_name_map[newState]),
        
        # Transition logic time!
        success = False

        if newState is self.states["ESTOP"]:
            success = True

        # all other transition subject to ESTOP stickiness
        else:
            if (time.time() - self.last_requested_to_go_to_estop) > self.ESTOP_STICKINESS:
            
                if self.state == self.states["ESTOP"] \
                    and newState in [self.states["ESTOP"], 
                                         self.states["ARMING"],
                                         self.states["TELEOP"],
                                         self.states["PRE_DRIVE"]]:
                    success = True

                elif self.state == self.states["SHUTDOWN"] \
                    and newState in [self.states["ESTOP"], 
                                             self.states["SHUTDOWN"]]:
                    success = True

                elif self.state == self.states["ARMING"] \
                    and newState in [self.states["ESTOP"],
                                             self.states["ARMING"],
                                             self.states["SHUTDOWN"]]:
                    success = True

                elif self.state == self.states["ARM"] \
                    and newState in [self.states["ESTOP"], 
                                             self.states["SHUTDOWN"]]:
                    success = True

                elif self.state == self.states["SOFT_STOP"] \
                    and newState in [self.states["ESTOP"],
                                             self.states["SHUTDOWN"]]:
                    success = True

                elif self.state == self.states["LAUNCH"] \
                    and newState in [self.states["ESTOP"], 
                                             self.states["SOFT_STOP"],
                                             self.states["SHUTDOWN"]]:
                    success = True

                elif self.state == self.states["FLIGHT"] \
                    and newState in [self.states["ESTOP"], 
                                             self.states["SOFT_STOP"],
                                             self.states["SHUTDOWN"]]:
                    success = True

                elif self.state == self.states["PRE_DRIVE"] \
                    and newState in [self.states["ESTOP"],
                                             self.states["SHUTDOWN"]]:
                    success = True

                elif self.state == self.states["DRIVE"] \
                    and newState in [self.states["ESTOP"],
                                             self.states["SHUTDOWN"]]:
                    success = True

                elif self.state == self.states["TELEOP"] \
                    and newState in [self.states["ESTOP"]]:
                    success = True
            else:
                success = False
                print "Rejecting transition due to ESTOP stickiness."

        if (success):
            print " success."
            self.state = newState
        else:
            print " illegal."


    def get_est_state(self):
        # presently, rely on self.particles to provice a good guess at location
        if self.particles is None or len(self.particles)==0:
            print "WARNING: CALL TO GET_EST_POS WHEN self.particles HAD VALUE " + str(self.particles)
            return np.array([0.0, 0.0, 0.0])

        # take weighted mean of particles' mu s
        mu_avg = 0
        weight_sum = 0
        for i in range(len(self.particles)):
            mu = np.array(self.particles[i][0]) # particle = [mu, Sigma, weight]
            weight = self.particles[i][2]
            mu_avg += mu * weight
            weight_sum += weight
        if weight_sum != 0:
            mu_avg = mu_avg / weight_sum
        else:
            print "WARNING: CALL TO GET_EST_POS RESULTED IN weight_sum == 0"
            return np.array([0.0, 0.0, 0.0])
        return mu_avg

    def handle_requested_state(self, channel, data):
        msg = state_t.decode(data)
        self.try_state_transition(msg.currentState)

    def flight_control_high_rate_handler(self, channel, data):
        self.acc_estimator_lock.acquire()

        try:
            msg = flight_control_high_rate_t.decode(data)
            self.stopped = msg.stopped
            self.last_fc_high_rate_recv_time = time.time()

        except Exception as e:
            print "Exception in flight control high rate handler: ", e
        self.acc_estimator_lock.release()

    def handle_bac_state_high_rate(self, channel, data):
        msg = bac_state_high_rate_t.decode(data)

        self.last_bac_high_rate_recv_time = time.time()
        self.brake_width = msg.estimated_distance
        self.brake_pressure = msg.brake_pressure

    def handle_brake_tare(self, channel, data):
        self.taring_counter = self.ACCEL_BIAS_N_SAMPLES
        self.acceleration_bias_est_sum = 0.0

    def handle_state_estimate(self, channel, data):
        self.se_lock.acquire()
        try:
            msg = state_estimator_particle_set.decode(data)

            if (self.last_se_recv_time):
                elapsed = time.time() - self.last_se_recv_time
                # low pass filter for acceleration averaged with RC constant of ALPHA second
                alpha = elapsed / (elapsed + self.brake_config.accel_alpha)    
            else:
                elapsed = 0
                alpha = 1.0

            particles = []
            for i in range(msg.n_particles):
                if msg.particles[i].id >= 0 and msg.particles[i].weight > 0.:
                    particles.append([msg.particles[i].mu,
                                      msg.particles[i].Sigma,
                                      msg.particles[i].weight])
            self.particles = particles
            self.last_se_recv_time = time.time()

            state = self.get_est_state()
            accel = state[2]
            # and average the accelerations further
            if (self.taring_counter > 0):
                self.acceleration_bias_est_sum = self.acceleration_bias_est_sum + state[2]
                self.taring_counter -= 1
                if (self.taring_counter == 0):
                    self.acceleration_bias_est = self.acceleration_bias_est_sum / float(self.ACCEL_BIAS_N_SAMPLES)

            self.acceleration_est = (1. - alpha)*self.acceleration_est + alpha*(accel - self.acceleration_bias_est)

            if (self.acceleration_est >= self.brake_config.launch_end_accel_threshold):
                self.last_accel_above_threshold = time.time()
            if (self.acceleration_est <= self.brake_config.launch_start_accel_threshold):
                self.last_accel_below_threshold = time.time()


        except Exception as e:
            print "Exception in state estimate handler: ", e

        self.se_lock.release()

    def handle_bac_mode(self, channel, data):
        msg = bac_mode_t.decode(data)
        if (msg.mode == msg.MODE_ESTOP):
            self.bac_mode = "MODE_ESTOP"
        elif (msg.mode == msg.MODE_TELEOP):
            self.bac_mode = "MODE_TELEOP"
        elif (msg.mode == msg.MODE_AUTO):
            self.bac_mode = "MODE_AUTO"
        else:
            print "Got BAC mode of weird mode ", msg.mode
            return
        self.last_bac_mode_recv_time = time.time()

    def handle_estop(self, channel, data):
        #self.last_requested_to_go_to_estop = time.time()
        self.state = self.states["ESTOP"]

    def handle_config_set(self, channel, data):
        msg = brake_trajectory_simple_t.decode(data)
        self.brake_config = msg

if __name__ == '__main__':
    lc = lcm_utils.create_lcm()
    fsm = FlightStateMachine(lc = lc)

    lcm_utils.start_lcm(lc)
    going = True
    while (going):
        try:
            fsm.update()
        except KeyboardInterrupt:
            going = False
        time.sleep(0.01)
    print "Bye o7"
        
