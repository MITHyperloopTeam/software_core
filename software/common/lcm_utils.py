import sys
import math
import signal
import thread
import lcm

def update_lcm(name, lc):
    while (1):
    	try:
        	lc.handle()
        except Exception as e:
        	print "Exception ", e, " in lcm update loop."

def create_lcm():
    return lcm.LCM("udpm://239.255.76.67:62237?ttl=0")

def start_lcm(lc):
    thread.start_new_thread( update_lcm, ("LCM Updater", lc) )
