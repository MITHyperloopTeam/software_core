#!/usr/bin/python

"""
Reads FC_OH IMU data (between two timestamps) in a file, and republishes
as SIM_IMU.
"""

import sys
import math
import signal
import os
import time

# the messaging stuff
import lcm
from mithl import flight_control_high_rate_t, vectorXf_t
from lcm_utils import *

import numpy as np


lc = create_lcm()
start_lcm(lc)

if len(sys.argv) < 2:
    print "Usage: publish_sim_imu_from_log.py LOGFILE <START_T> <END_T>"
    exit(0)

fc_oh = np.recfromcsv(sys.argv[1])
ts = fc_oh["recv_timestamp"] / 1000. / 1000
ts -= ts[0]
f_xdd = fc_oh["front_nav_imu_xdd"]
r_xdd = fc_oh["rear_nav_imu_xdd"]

start_t = ts[0]
end_t = ts[-1]
if len(sys.argv) >= 3:
	start_t = float(sys.argv[2])
if len(sys.argv) >= 4:
	end_t = float(sys.argv[3])

msg_f = vectorXf_t()
msg_f.rows = 6
msg_f.data = [0.0]*6
msg_r = vectorXf_t()
msg_r.rows = 6
msg_r.data = [0.0]*6

starttime = time.time()
for i in range(fc_oh.shape[0]):
	if (ts[i] < start_t):
		starttime = time.time()
	else:
		thistime = ts[i] - start_t
		if ((time.time() - starttime) < thistime):
			time.sleep((thistime - (time.time() - starttime)))
		
		msg_f.utime = ts[i]*1000*1000
		msg_f.data[0] = f_xdd[i]
		lc.publish("SIM_FC_IMU_F", msg_f.encode())

		msg_r.utime = ts[i]*1000*1000
		msg_r.data[0] = r_xdd[i]
		lc.publish("SIM_FC_IMU_R", msg_r.encode())