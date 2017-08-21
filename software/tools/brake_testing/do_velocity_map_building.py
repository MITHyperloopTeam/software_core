#!/usr/bin/env python

'''
    Velocity lookup table generator for brake actuator

    Opens and closes brakes repeatedly to build
    an actuator-state-to-velocity lookup table

    To open brakes, puts on/off on, and prop to 100%,
    and runs motor at some rate, looking at velocity from motor.

    To close brakes, puts on/off and motor off, and prop to some
    value, and looks at closing velocity.

    Needs a couple of values defined to work:

    For opening: 
        - motor_min_rate, motor_max_rate, and motor_stepsize

    For closing:
        - prop_min_rate, prop_max_rate, prop_stepsize

    For both:
        - brake_open_outer and brake_closed_outer, which are the staging positions
        the system reaches before beginning a test
        - brake_open_inner and brake_closed_inner, which are the two points between
        which the movement is timed to figure out the brake caliper speed at this setting

    So during a given open or close test, the system starts at the *_outer position,
    submits a teleop command in the right configuration, starts and stops a timer as the
    brakes cross the inner positions. When the system reaches the other *_outer position,
    it freezes everything by turning on on/off valve.
'''
import sys
import math
import signal
import time
import datetime
import os

import math, random
import numpy as np
from scipy.signal import chirp
from numpy import linalg

#comms stuff
import lcm
from mithl import bac_teleop_cmd_t
from mithl import bac_state_high_rate_t
from mithl import bac_mode_t
from lcm_utils import *

from threading import Lock

import numpy as np
import matplotlib.pyplot as plt

brake_open_outer = 5.1
brake_closed_outer = 0.35
brake_open_inner = 4.9
brake_closed_inner = 0.5

motor_num_steps = 8
motor_min_rate = 0.1
motor_max_rate = 1.0
motor_default_rate = 1.0 # used to open system

prop_num_steps = 8
prop_min_rate = 0.68
prop_max_rate = 0.83
prop_default_rate = 0.78 # used to close system

max_test_duration = 30.0

state_lock = Lock()
state_known = False
last_state_time = 0
last_known_position = 0
last_crossed_open_inner = 0
last_crossed_closed_inner = 0
last_on_off_state = False
last_motor_rate = 0
last_prop_rate = 0

def handle_bac_state(channel, data):
    global state_lock, state_known, last_crossed_open_inner, last_crossed_closed_inner
    global last_known_position, last_state_time, last_motor_rate, last_prop_rate, last_on_off_state

    msg = bac_state_high_rate_t.decode(data)
    state_lock.acquire()

    now = time.time()
    if (state_known):
        if (msg.estimated_distance > brake_open_inner and last_known_position <= brake_open_inner):
            frac_after = msg.estimated_distance - brake_open_inner
            frac_before = brake_open_inner - last_known_position
            last_crossed_open_inner = (now * frac_after + last_state_time * frac_before) / (frac_after + frac_before)
            print "Crossed open inner outwards %f" % last_crossed_open_inner
        elif (msg.estimated_distance < brake_open_inner and last_known_position >= brake_open_inner):
            frac_after = brake_open_inner - msg.estimated_distance
            frac_before = last_known_position - brake_open_inner
            last_crossed_open_inner = (now * frac_after + last_state_time * frac_before) / (frac_after + frac_before)
            print "Crossed open inner inwards at %f" % last_crossed_open_inner

        if (msg.estimated_distance > brake_closed_inner and last_known_position <= brake_closed_inner):
            frac_after = msg.estimated_distance - brake_closed_inner
            frac_before = brake_closed_inner - last_known_position
            last_crossed_closed_inner = (now * frac_after + last_state_time * frac_before) / (frac_after + frac_before)
            print "Crossed closed inner outwards at %f" % last_crossed_closed_inner

        elif (msg.estimated_distance < brake_closed_inner and last_known_position >= brake_closed_inner):
            frac_after = brake_closed_inner - msg.estimated_distance
            frac_before = last_known_position - brake_closed_inner
            last_crossed_closed_inner = (now * frac_after + last_state_time * frac_before) / (frac_after + frac_before)
            print "Crossed closed inner inwards at %f" % last_crossed_closed_inner

    state_known = True
    last_known_position = msg.estimated_distance
    last_state_time = now
    last_motor_rate = msg.PumpMotorPWM
    last_prop_rate = msg.ValvePWM
    last_on_off_state = msg.OnOffValve
    state_lock.release()

def get_into_teleop_mode():
    teleop_msg = bac_teleop_cmd_t()
    teleop_msg.utime = time.time() * 1000 * 1000
    teleop_msg.PWREN = True
    teleop_msg.PumpMotorPWM = 0
    teleop_msg.PumpMotorEnable = False
    teleop_msg.OnOffValve = True
    teleop_msg.ValvePWM = 1.0
    teleop_msg.ValveEnable = True
    teleop_msg.LSMotorEnable = False
    teleop_msg.LSMotorDisable = True
    teleop_msg.LSActuatorEnable = False
    teleop_msg.LSActuatorDisable = True
    lc.publish("_BAC_TELEOP_CMD_SET", teleop_msg.encode())

    time.sleep(0.25)
    msg = bac_mode_t()
    msg.mode = msg.MODE_TELEOP
    lc.publish("_BAC_MODE_SET", msg.encode())

def send_freeze_command_blocking():
    teleop_msg = bac_teleop_cmd_t()
    teleop_msg.utime = time.time() * 1000 * 1000
    teleop_msg.PWREN = True
    teleop_msg.PumpMotorPWM = 0
    teleop_msg.PumpMotorEnable = False
    teleop_msg.OnOffValve = True
    teleop_msg.ValvePWM = 1.0
    teleop_msg.ValveEnable = True
    teleop_msg.LSMotorEnable = False
    teleop_msg.LSMotorDisable = True
    teleop_msg.LSActuatorEnable = False
    teleop_msg.LSActuatorDisable = True

    resending = True
    while (resending):
        lc.publish("_BAC_TELEOP_CMD_SET",teleop_msg.encode())
        time.sleep(0.25)
        state_lock.acquire()
        if (last_on_off_state == True):
            resending = False
        state_lock.release()

def send_close_command_at_rate(prop_rate):
    teleop_msg = bac_teleop_cmd_t()
    teleop_msg.utime = time.time() * 1000 * 1000
    teleop_msg.PWREN = True
    teleop_msg.PumpMotorPWM = 0
    teleop_msg.PumpMotorEnable = False
    teleop_msg.OnOffValve = False
    teleop_msg.ValvePWM = prop_rate
    teleop_msg.ValveEnable = True
    teleop_msg.LSMotorEnable = False
    teleop_msg.LSMotorDisable = True
    teleop_msg.LSActuatorEnable = False
    teleop_msg.LSActuatorDisable = True

    lc.publish("_BAC_TELEOP_CMD_SET",teleop_msg.encode())

def send_open_command_at_rate(motor_rate):
    teleop_msg = bac_teleop_cmd_t()
    teleop_msg.utime = time.time() * 1000 * 1000
    teleop_msg.PWREN = True
    teleop_msg.PumpMotorPWM = motor_rate
    teleop_msg.PumpMotorEnable = True
    teleop_msg.OnOffValve = True
    teleop_msg.ValvePWM = 1.0
    teleop_msg.ValveEnable = True
    teleop_msg.LSMotorEnable = False
    teleop_msg.LSMotorDisable = True
    teleop_msg.LSActuatorEnable = False
    teleop_msg.LSActuatorDisable = True

    lc.publish("_BAC_TELEOP_CMD_SET",teleop_msg.encode())
    

def do_opening_test_at_rate(motor_rate):
    send_close_command_at_rate(prop_default_rate)
    
    print "\tGoing to closed state ..."

    waiting_for_close = True
    while (waiting_for_close):
        time.sleep(0.1)
        send_close_command_at_rate(prop_default_rate)
        state_lock.acquire()
        if (state_known and last_known_position < brake_closed_outer):
            waiting_for_close = False
        state_lock.release()

    time.sleep(0.25)

    print "\tSending open cmd %f ..." % motor_rate
    send_open_command_at_rate(motor_rate)

    print "\tWaiting for conclude ..."
    running_test = True
    test_start_time = time.time()
    while (running_test):
        time.sleep(0.1)
        send_open_command_at_rate(motor_rate)
        state_lock.acquire()
        if (state_known and last_known_position > brake_open_outer):
            running_test = False
            # calculate result
            print "\tFinished test."
            elapsed = last_crossed_open_inner - last_crossed_closed_inner
            distance = brake_open_inner - brake_closed_inner
            velocity_estimate = distance / elapsed
            state_lock.release()
            # freeze system
            send_freeze_command_blocking()
        elif (time.time() - test_start_time > max_test_duration):
            print "\tTimed out on test."
            velocity_estimate = 0
            running_test = False
            state_lock.release()
            send_freeze_command_blocking()
        else:
            state_lock.release()

    print "\tDone with vel %f" % velocity_estimate
    return velocity_estimate

def do_closing_test_at_rate(prop_rate):
    send_open_command_at_rate(motor_default_rate)
    
    print "\tGoing to open state ..."

    waiting_for_open = True
    while (waiting_for_open):
        time.sleep(0.1)
        send_open_command_at_rate(motor_default_rate)
        state_lock.acquire()
        if (state_known and last_known_position > brake_open_outer):
            waiting_for_open = False
        state_lock.release()

    time.sleep(0.25)

    print "\tSending close cmd %f ..." % prop_rate
    send_close_command_at_rate(prop_rate)

    print "\tWaiting for conclude ..."
    running_test = True
    test_start_time = time.time()
    while (running_test):
        time.sleep(0.1)
        send_close_command_at_rate(prop_rate)
        state_lock.acquire()
        if (state_known and last_known_position < brake_closed_outer):
            running_test = False
            print "\tFinished test."
            # calculate result
            elapsed = last_crossed_closed_inner - last_crossed_open_inner
            distance = brake_closed_inner - brake_open_inner
            velocity_estimate = distance / elapsed
            state_lock.release()
            # freeze system
            send_freeze_command_blocking()
        elif (time.time() - test_start_time > max_test_duration):
            print "\tTimed out on test."
            velocity_estimate = 0
            running_test = False
            state_lock.release()
            send_freeze_command_blocking()
        else:
            state_lock.release()

    print "\tDone with vel %f" % velocity_estimate
    return velocity_estimate


def update_plots(x1, y1, x2, y2):
    plt.subplot(2, 1, 1)
    plt.hold(False)
    plt.plot(x1, y1, 'ro-')
    plt.title("Learned velocities")
    plt.ylabel("Motor open")

    plt.subplot(2, 1, 2)
    plt.plot(x2, y2, 'bo-')
    plt.ylabel("Valve close")
    plt.xlabel("Input")

    plt.draw()

    print "Current table:"
    print motor_rates
    print motor_vels
    print prop_rates
    print prop_vels
    plt.pause(0.0001)

if __name__ == '__main__':

    try:
        import signal
        signal.signal(signal.SIGTSTP, signal.SIG_IGN)

        def signal_handler(signal, frame):
                print('You pressed Ctrl+C!')
                plt.savefig("fig.png")
                plt.close('all')
                exit(0)
        signal.signal(signal.SIGINT, signal_handler)

        lc = create_lcm()

        if (len(sys.argv) != 2):
            print "Usage: <password>"
            sys.exit(0)

        motor_rates = np.linspace(motor_min_rate, motor_max_rate, motor_num_steps)
        motor_vels = 0 * motor_rates
        motor_rate_i = 0

        prop_rates = np.linspace(prop_min_rate, prop_max_rate, prop_num_steps)
        prop_vels = 0 * prop_rates
        prop_rate_i = 0

        plt.ion()

        update_plots(motor_rates, motor_vels, prop_rates, prop_vels)

        start_lcm(lc)

        lc.subscribe("_BAC_STATE_H", handle_bac_state)

        get_into_teleop_mode()

        while (motor_rate_i < len(motor_rates) or prop_rate_i < len(prop_rates)):
            if (motor_rate_i < len(motor_rates)):
                print "Doing motor test with rate %f" % motor_rates[motor_rate_i]
                motor_vels[motor_rate_i] = do_opening_test_at_rate(motor_rates[motor_rate_i])
                motor_rate_i+=1

                update_plots(motor_rates, motor_vels, prop_rates, prop_vels)

                time.sleep(0.1)

            if (prop_rate_i < len(prop_rates)):
                print "Doing prop test with rate %f" % prop_rates[prop_rate_i]
                prop_vels[prop_rate_i] = do_closing_test_at_rate(prop_rates[prop_rate_i])
                prop_rate_i+=1

                update_plots(motor_rates, motor_vels, prop_rates, prop_vels)

                time.sleep(0.1)

    except Exception as e:
        print "Caught exception ", e

    timestampname = datetime.datetime.fromtimestamp(time.time()).strftime("%Y%m%d_%H.%M.%S")
    outputname = 'velmap_' + timestampname

    print "Writing output..."
    the_text_file = open(outputname + ".txt","w")
    print ('motor_rates:\n%s\n' % str(motor_rates)),
    the_text_file.write('motor_rates:\n%s\n' % str(motor_rates))
    print ('motor_vels:\n%s\n' % str(motor_vels)),
    the_text_file.write('motor_vels:\n%s\n' % str(motor_vels))
    print ('prop_rates:\n%s\n' % str(prop_rates)),
    the_text_file.write('prop_rates:\n%s\n' % str(prop_rates))
    print ('prop_vels:\n%s\n' % str(prop_vels)),
    the_text_file.write('prop_vels:\n%s\n' % str(prop_vels))
    the_text_file.close()

    plt.show()
    plt.savefig(outputname + ".png")
    plt.close('all')
