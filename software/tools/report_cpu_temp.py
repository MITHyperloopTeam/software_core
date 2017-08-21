#!/usr/bin/python

"""
Computer Temperature Reporter

MIT Hyperloop Team, 2016
"""

import sys
import math
import signal
import os
import time

# the messaging stuff
import lcm
from mithl import vectorXf_t
from lcm_utils import *

lc = create_lcm()
start_lcm(lc)

if len(sys.argv) != 2:
    print "Usage: report_cpu_temp.py CHANNEL_NAME"
    exit(0)

print "Starting temp reporting."

while (1):
    try:
        msg = vectorXf_t()

        msg.utime = time.time()*1000*1000
        msg.rows = 1
        cat = lambda file: open(file, 'r').read().strip()
        temp = float(cat("/sys/devices/virtual/thermal/thermal_zone0/temp"))/1000.0 # *C
        print "Temp %f" % temp
        msg.data = [temp]

        lc.publish(sys.argv[1], msg.encode())

        time.sleep(0.5)

    except Exception as e:
        print "Exception ", e
