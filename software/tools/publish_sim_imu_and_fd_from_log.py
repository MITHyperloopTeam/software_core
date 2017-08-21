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
from mithl import flight_control_high_rate_t, vectorXf_t, fiducial_t
from lcm_utils import *

import numpy as np


lc = create_lcm()
start_lcm(lc)

if len(sys.argv) < 2:
    print "Usage: publish_sim_imu_from_log.py LOGFILE_FC LOGFILE_FD <START_T> <END_T>"
    exit(0)

fc_oh = np.recfromcsv(sys.argv[1])
ts = fc_oh["recv_timestamp"] / 1000. / 1000
t0 = ts[0]
ts -= t0
f_xdd = fc_oh["front_nav_imu_xdd"]
r_xdd = fc_oh["rear_nav_imu_xdd"]

fd_o = np.recfromcsv(sys.argv[2])
ts_fd = fd_o["recv_timestamp"] / 1000. / 1000
ts_fd -= t0

start_t = ts[0]
end_t = ts[-1]
if len(sys.argv) >= 4:
	start_t = float(sys.argv[3])
if len(sys.argv) >= 5:
	end_t = float(sys.argv[4])

msg_f = vectorXf_t()
msg_f.rows = 6
msg_f.data = [0.0]*6
msg_r = vectorXf_t()
msg_r.rows = 6
msg_r.data = [0.0]*6

msg_fd = fiducial_t()

starttime = time.time()

tnow = start_t
fc_oh_i = -1;
fd_o_i = -1;
while (tnow < end_t):
	# find soonest message in future and publish that
	soonest_fd = None
	for i in range(fd_o_i+1, fd_o.shape[0]):
		if (ts_fd[i] > tnow):
			soonest_fd = ts_fd[i]
			#fd_o_i = i
			break
	soonest_fc = None
	for i in range(fc_oh_i+1, fc_oh.shape[0]):
		if (ts[i] > tnow):
			soonest_fc = ts[i]
			#fc_oh_i = i
			break

	publish_fc = False
	publish_fd = False
	if (soonest_fc is not None and soonest_fd is not None):
		print soonest_fc, soonest_fd
		if (soonest_fd < soonest_fc):
			publish_fd = True
			thistime = soonest_fd - start_t
		else:
			publish_fc = True
			thistime = soonest_fc - start_t
	elif soonest_fc:
		publish_fc = True
		thistime = soonest_fc - start_t
	elif soonest_fd:
		publish_fd = True
		thistime = soonest_fd - start_t
	else:
		break

	if ((time.time() - starttime) < thistime):
		print thistime, time.time() - starttime
		try:
			time.sleep((thistime - (time.time() - starttime)))
		except:
			pass

	if (publish_fc):
		msg_f.utime = ts[fc_oh_i]*1000*1000
		msg_f.data[0] = f_xdd[fc_oh_i]
		lc.publish("SIM_FC_IMU_F", msg_f.encode())

		msg_r.utime = ts[fc_oh_i]*1000*1000
		msg_r.data[0] = r_xdd[fc_oh_i]
		lc.publish("SIM_FC_IMU_R", msg_r.encode())
		fc_oh_i += 1

	if (publish_fd):
		msg_fd.utime = ts_fd[fd_o_i]*1000*1000
		msg_fd.time_since_last = fd_o["time_since_last"][fd_o_i]
		msg_fd.average_time_between = fd_o["average_time_between"][fd_o_i]
		msg_fd.average_time_strip = fd_o["average_time_strip"][fd_o_i]
		msg_fd.total_count = fd_o["total_count"][fd_o_i]
		lc.publish("_FD_O", msg_fd.encode())
		fd_o_i += 1

	tnow = thistime