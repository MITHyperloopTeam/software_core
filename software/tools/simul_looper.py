#!/usr/bin/env python

import lcm
import lcm_utils
import time
from mithl import string_t, trigger_t, floating_base_t, state_t, vectorXf_t

class Looper():

    def __init__(self, lc):
        self.lc = lc
        podchannel_sub = self.lc.subscribe("SIM_FB", self.handle_vectorXf_t)

    def update(self):
        self.reset_sim()
        time.sleep(2.0)
        self.arm_pod()
        time.sleep(2.0)
        self.sim_not_done = True
        self.start_sim()

        startWait = time.time()
        while (self.sim_not_done and time.time() - startWait < 10.0):
            time.sleep(0.1)

        time.sleep(4.0)

    def arm_pod(self):
        sendTime = time.time()
        state_msg = state_t()
        state_msg.utime = 0
        state_msg.currentState = state_msg.ARM
        self.lc.publish("REQUESTED_STATE",state_msg.encode())

    def start_sim(self):
        sendTime = time.time()
        start_msg = trigger_t()
        start_msg.utime = 0
        self.lc.publish("START",start_msg.encode())

    def reset_sim(self):
        sendTime = time.time()
        reset_msg = trigger_t()
        reset_msg.utime = 0
        self.lc.publish("RESET",reset_msg.encode())


    def handle_vectorXf_t(self, channel, data):
        msg = vectorXf_t.decode(data)
        if (msg.rows > 0):
            if (msg.data[0] > 900 and abs(msg.data[1]) < 0.1) or (msg.data[0] > 995):
                self.sim_not_done = False
        

if __name__ == '__main__':
    lc = lcm_utils.create_lcm()
    looper = Looper(lc = lc)

    lcm_utils.start_lcm(lc)
    going = True
    while (going):
        try:
            looper.update()
        except KeyboardInterrupt:
            going = False

    print "Bye o7"
        
