#!/usr/bin/env python

'''
    Step response tester for brake actuator controller

    Repeatedly sends random goals in the actuator range.
    Required parameters:
        - Min command
        - Max command
        - Tolerance (symmetric -- above or below)
        - Settling time required for demonstration

    Picks a random value in the actuator range, and commands it as
    a setpoint. Measures:
        - Response time to first enter the tolerance band
        - Max overshoot from tolerance band after that point on the
          other side of the setpoint
        - Settling time (earliest time such that all later samples
          are within tolerance zone)
'''
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
from mithl import bac_state_high_rate_t
from lcm_utils import *

from threading import Lock

import numpy as np
import matplotlib.pyplot as plt

min_command = 0
max_command = 5.25
tolerance = 0.1 #both above and below
settling_time_required = 2.0
max_test_duration = 8.0

state_lock = Lock()
state_known = False
cmd_known = False

# -1 if no command set or need to update values
setpoint = -1
cmd_confirmed_time = -1
est_response_time = -1
approach_direction = 1 # 1 from bottom, -1 from top
max_overshoot = -1
max_overshoot_time = -1
est_settling_time = -1

last_known_position = 0
last_state_time = 0

def handle_cmd_state(channel, data):
    global min_command, max_command, tolerance, settling_time_required, max_test_duration
    global state_lock, state_known, cmd_known, setpoint, cmd_confirmed_time

    msg = bac_auto_cmd_t.decode(data)
    state_lock.acquire()
    now = time.time()
    # has a new command been confirmed?
    if (setpoint > 0 and cmd_confirmed_time < 0 and abs(msg.setpoint - setpoint) < 0.01):
        cmd_confirmed_time = now
        print "\tCmd to %f confirmed at %f" % (msg.setpoint, cmd_confirmed_time)

    cmd_known = True
    state_lock.release()


def handle_bac_state(channel, data):
    global min_command, max_command, tolerance, settling_time_required, max_test_duration
    global state_lock, state_known, cmd_known, setpoint, cmd_confirmed_time
    global est_response_time, max_overshoot, max_overshoot_time, est_settling_time
    global last_known_position, last_state_time

    msg = bac_state_high_rate_t.decode(data)
    state_lock.acquire()

    now = time.time()
    # if we have a command and it has been confirmed...
    if (state_known and setpoint > 0):
        # response time is first time we're in tolerance
        if est_response_time < 0 and abs(msg.estimated_distance - setpoint) < tolerance:
            est_response_time = time.time()
            print "\tResponse time found"

        # overshoot cases
        if (est_response_time > 0 and msg.estimated_distance > setpoint and approach_direction > 0):
            if (msg.estimated_distance - setpoint > max_overshoot):
                max_overshoot = msg.estimated_distance - setpoint
                max_overshoot_time = now
                print "\tOvershoot updated"
        if (est_response_time > 0 and msg.estimated_distance < setpoint and approach_direction < 0):
            if (setpoint - msg.estimated_distance > max_overshoot):
                max_overshoot = setpoint - msg.estimated_distance
                max_overshoot_time = now
                print "\tOvershoot updated"
        
        # settling time calculation
        if (est_response_time > 0):
            if est_settling_time < 0 or abs(msg.estimated_distance - setpoint) > tolerance:
                est_settling_time = now
                print "\tSettling time updated"

    state_known = True
    last_known_position = msg.estimated_distance
    last_state_time = now
    state_lock.release()


def send_command(val):
    global cmd_confirmed_time
    cmd_msg = bac_auto_cmd_t()
    cmd_msg.utime = time.time() * 1000 * 1000
    cmd_msg.setpoint = val

    resending = True
    cmd_confirmed_time = -1

    while (resending):
        lc.publish("_BAC_AUTO_CMD_SET",cmd_msg.encode())
        time.sleep(0.2)
        state_lock.acquire()
        if (cmd_confirmed_time > 0):
            resending = False
        state_lock.release()

def update_plots(x1, y1, y2, y3):
    plt.subplot(3, 1, 1)
    plt.plot(x1, y1, 'ro')
    plt.title("Step Responses")
    plt.ylabel("Response Times")

    plt.subplot(3, 1, 2)
    plt.plot(x1, y2, 'go')
    plt.ylabel("Overshoots")

    plt.subplot(3, 1, 3)
    plt.plot(x1, y3, 'bo')
    plt.ylabel("Settling Times")

    plt.xlabel("Step size and dir")

    plt.draw()

    print "Current table:"
    print "Magnitudes: ", x1
    print "Response times: ", y1
    print "Overshoots: ", y2
    print "Settling times: ", y3
    plt.pause(0.0001)

if __name__ == '__main__':

    try:
        import signal
        signal.signal(signal.SIGTSTP, signal.SIG_IGN)

        def signal_handler(signal, frame):
                print('You pressed Ctrl+C!')
                plt.savefig("step_responses.png")
                plt.close('all')
                exit(0)
        signal.signal(signal.SIGINT, signal_handler)

        lc = create_lcm()

        if (len(sys.argv) != 2):
            print "Usage: <password>"
            sys.exit(0)

        step_magnitudes = []
        response_times = []
        overshoots = []
        settling_times = []

        plt.ion()

        update_plots(step_magnitudes, response_times, overshoots, settling_times)

        start_lcm(lc)

        lc.subscribe("_BAC_STATE_H", handle_bac_state)
        lc.subscribe("_BAC_AUTO_CMD_STATE", handle_cmd_state)

        waiting_for_state = True
        while (waiting_for_state):
            state_lock.acquire()
            if (state_known):
                waiting_for_state = False
            state_lock.release()
            time.sleep(0.25)

        while (1):
            new_dest = random.random()*(max_command - min_command) + min_command

            # reset everything
            setpoint = new_dest
            cmd_confirmed_time = -1
            est_response_time = -1
            max_overshoot = -1
            max_overshoot_time = -1
            est_settling_time = -1

            state_lock.acquire()
            step = new_dest - last_known_position
            if (new_dest > last_known_position):
                approach_direction = 1 # 1 from bottom, -1 from top
            else:
                approach_direction = -1
            state_lock.release()

            send_command(new_dest)
            sent_command = time.time()
            waiting = True
            while (waiting):
                print "here"
                now = time.time()
                state_lock.acquire()
                print "\t Step of %f rt %f, overshoot %f, settled in %f." % (step, est_response_time - cmd_confirmed_time, max_overshoot, est_settling_time - cmd_confirmed_time)
                if (now - sent_command > max_test_duration):
                    print "\t Timed out!"
                    waiting = False
                elif cmd_confirmed_time > 0 and est_response_time > 0 and max_overshoot_time > 0 and est_settling_time > 0:
                    print "\t Step of %f rt %f, overshoot %f, settled in %f." % (step, est_response_time - cmd_confirmed_time, max_overshoot, est_settling_time - cmd_confirmed_time)
                    step_magnitudes.append(step)
                    response_times.append(est_response_time - cmd_confirmed_time)
                    overshoots.append(max_overshoot)
                    settling_times.append(est_settling_time - cmd_confirmed_time)
                    waiting = False
                state_lock.release()
                time.sleep(0.05)

            update_plots(step_magnitudes, response_times, overshoots, settling_times)
            time.sleep(1)

    except Exception as e:
        print "Caught exception ", e

    plt.show()
    plt.savefig("step_responses.png")
    plt.close('all')