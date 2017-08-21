#!/usr/bin/python

"""
String publisher

MIT Hyperloop Team, 2016
"""

import sys
import math
import signal
import os
import time

# the messaging stuff
import lcm
from mithl import string_t
from lcm_utils import *

lc = create_lcm()
start_lcm(lc)

if len(sys.argv) != 3:
    print "Usage: publish_string.py CHANNEL_NAME STRING"
    exit(0)

msg = string_t()

msg.utime = time.time()*1000*1000
msg.data = sys.argv[2]

lc.publish(sys.argv[1], msg.encode())