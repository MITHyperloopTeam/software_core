#!/usr/bin/env python

import sys
import math
import signal
import time 
import os

import math, random
import numpy as np
from scipy.signal import chirp
from numpy import linalg

#comms stuff
import lcm
from mithl import bac_auto_cmd_t
from mithl import bac_state_t
from lcm_utils import *


if __name__ == '__main__':
    lc = create_lcm()

    if (len(sys.argv) != 6):
        print "Usage: <duration, secs> <f0, hz> <f1, hz> <low> <high>"
        sys.exit(0)

    t1 = float(sys.argv[1])
    f0 = float(sys.argv[2])
    f1 = float(sys.argv[3])
    low = float(sys.argv[4])
    high = float(sys.argv[5])

    print "Chirping from t=0 to t=%f, freq %f to %f, %f to %f amplitude" % (t1, f0, f1, low, high)

    timestep = 0.01
    t = np.arange(0, t1, timestep)

    fun = chirp(t, f0, t1, f1, method='linear')

    # remap fun into [0, 1]
    fun = (-fun)/2.0 + 0.5

    # and then into supplied range
    fun = (fun * (high-low)) + low

    start_lcm(lc)

    t_start = time.time()
    tlast = 0.0

    while (time.time() - t_start < t1):
        tnow = time.time() - t_start

        if (tnow - tlast > timestep):
            tlast = tnow
            val = np.interp(tnow, t, fun)
            cmd_msg = bac_auto_cmd_t()
            cmd_msg.utime = tnow * 1000 * 1000
            cmd_msg.setpoint = val
            

            lc.publish("_BAC_AUTO_CMD_SET",cmd_msg.encode())
        else:
            time.sleep(timestep)
