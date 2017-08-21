#!/usr/bin/python

"""
Gamepad LCM Controller Driver

Referencing code from dgonz's website
http://yameb.blogspot.com/2013/01/gamepad-input-in-python.html
and the second-order references of that.

MIT Hyperloop Team, 2016
"""

import sys
import math
import signal
import os
import time

# the messaging stuff
import lcm
from mithl import gamepad_t
from lcm_utils import *

import pygame

lc = create_lcm()
start_lcm(lc)

pygame.init()
j = pygame.joystick.Joystick(0)
j.init()

stdout = sys.__stdout__
stderr = sys.__stderr__
def silence():
    sys.stdout = open(os.devnull,'w')
    sys.stderr = open(os.devnull,'w')
def unsilence():
    sys.stdout = stdout
    sys.stderr = stderr

print "Initialized Joystick: %s" % j.get_name()
silence()
n_axes = j.get_numaxes()
n_balls = j.get_numballs()
n_buttons = j.get_numbuttons()
n_hats = j.get_numhats()
unsilence()

print "It has %d axes, %d balls, %d buttons, and %d hats" % (n_axes,
                                                             n_balls,
                                                             n_buttons,
                                                             n_hats)


while (1):
    try:
        # ugh, why doesn't this silencing work? goddamn sdl
        silence()
        pygame.event.pump()

        msg = gamepad_t()
        msg.n_axes = n_axes

        msg.axis = [j.get_axis(i) for i in range(n_axes)]


        msg.n_balls = n_balls
        balls = [j.get_ball(i) for i in range(n_balls)]
        msg.ball_x = [pair[0] for pair in balls] 
        msg.ball_y = [pair[1] for pair in balls] 

        msg.n_buttons = n_buttons
        msg.button = [j.get_button(i) for i in range(n_buttons)]

        msg.n_hats = n_hats
        hats = [j.get_hat(i) for i in range(n_hats)]
        msg.hat_x = [pair[0] for pair in hats]
        msg.hat_y = [pair[1] for pair in hats]
        unsilence()

        lc.publish("GAMEPAD", msg.encode())

        time.sleep(0.05)

    except Exception as e:
        unsilence()
        print "Exception ", e
        sys.exit(0)