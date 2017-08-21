#!/usr/bin/env python

import lcm
import lcm_utils
import time
from mithl import trigger_t, floating_base_t, state_t

class HealthMonitor():
    ''' Does flight control logic. '''

    def __init__(self, lc):
        self.lc = lc

        self.fsm_state_sub = lc.subscribe("STATE", self.handle_fsm_state) 
        self.pod_se_fb_sub = lc.subscribe("_SE_FB", self.handle_pod_se_fb)

        self.tempState = state_t()
        self.last_fsm_state = self.tempState.SAFE

        # error states
        self.do_estop = False

        self.last_state_msg_timestamp = -1.
        # in case this is useful:
        self.last_state_msg_localtime = -1.

        # quick and dirty mode switch estimator
        self.acceleration_est = 0.
        self.velocity_last = 0.

        self.last_print_time = time.time()
        self.last_publish_timestamp = self.last_state_msg_timestamp
        self.last_publish_time = time.time()

    def update(self):
        if (time.time() - self.last_print_time > 1.0):
            self.last_print_time = time.time()
            print "HM>> ", "I see FSM in state ", (self.last_fsm_state or "None"), " and estop is ", self.do_estop

        # health check:
        if self.velocity_last > 1 and self.last_fsm_state not in ["STATE_LAUNCH", "STATE_FLIGHT", "SOFT_STOP", "ESTOP"]:
             self.do_estop = True

        # if sim is much faster than reality, keep up with it.
        if (self.do_estop and 
            (self.last_state_msg_timestamp - self.last_publish_timestamp > 0.01 or
            time.time() - self.last_publish_time > 0.01)):
            self.publish_estop()
            self.last_publish_timestamp = self.last_state_msg_timestamp
            self.last_publish_time = time.time()


    def publish_estop(self):
        msg = state_t()
        msg.utime = int(self.last_state_msg_timestamp * 1000 * 1000)
        msg.currentState = self.tempState.ESTOP
        self.lc.publish("REQUESTED_STATE", msg.encode())

    def handle_fsm_state(self, channel, data):
        msg = state_t.decode(data)
        self.last_fsm_state = msg.currentState

    def handle_pod_se_fb(self, channel, data):
        msg = floating_base_t.decode(data)

        elapsed = msg.utime/1000./1000. - self.last_state_msg_timestamp
        instantaneous_accel = msg.v[0] - self.velocity_last
        self.velocity_last = msg.v[0]

        # low pass filter for acceleration averaged with RC constant of 0.1 second
        alpha = elapsed / (elapsed + 0.1)
        self.acceleration_est = (1 - alpha)*self.acceleration_est + alpha*instantaneous_accel

        self.last_state_msg_timestamp = msg.utime/1000./1000.
        self.last_state_msg_localtime = time.time()

        

if __name__ == '__main__':
    lc = lcm_utils.create_lcm()
    hm = HealthMonitor(lc = lc)

    lcm_utils.start_lcm(lc)
    going = True
    while (going):
        try:
            hm.update()
        except KeyboardInterrupt:
            going = False

    print "Bye o7"
        
