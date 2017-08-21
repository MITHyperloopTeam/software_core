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
from mithl import bac_teleop_cmd_t
from mithl import bac_state_t
from lcm_utils import *


if __name__ == '__main__':
    lc = create_lcm()

    if (len(sys.argv) != 4):
        print "Usage: <duration, secs> <f0, hz> <f1, hz>"
        sys.exit(0)

    t1 = float(sys.argv[1])
    f0 = float(sys.argv[2])
    f1 = float(sys.argv[3])

    timestep = 0.01
    t = np.arange(0, t1, timestep)

    fun = chirp(t, f0, t1, f1, method='linear')

    # remap fun into valve range, starting closed
    fun = (-fun)/2.0 + 0.5

    start_lcm(lc)

    t_start = time.time()
    tlast = 0.0

    while (time.time() - t_start < t1):
        tnow = time.time() - t_start

        if (tnow - tlast > timestep):
            tlast = tnow
            val = np.interp(tnow, t, fun)
            teleop_msg = bac_teleop_cmd_t()
            teleop_msg.utime = tnow * 1000 * 1000
            teleop_msg.PWREN = True
            teleop_msg.PumpMotorPWM = 0.0
            teleop_msg.PumpMotorEnable = False
            teleop_msg.OnOffValve = False
            teleop_msg.ValveEnable = True
            teleop_msg.LSMotorEnable = False
            teleop_msg.LSMotorDisable = True
            teleop_msg.LSActuatorEnable = False
            teleop_msg.LSActuatorDisable = True

            teleop_msg.ValvePWM = val

            lc.publish("_BAC_TELEOP_CMD_SET",teleop_msg.encode())
        else:
            time.sleep(timestep)