#!/usr/bin/python
'''
Author: geronm@mit.edu

This is a script to grab fields from LCM channels and write them to ascii files.

'''
import os.path
import sys
import time
import random

import lcm

import bot_core
import mithl

commstr = sys.argv[0]
USAGE = '''
USAGE: ''' + commstr + ''' input_log_file output_csv_file channel_name lcmtype_path msg_field_expr1 [msg_field_expr2  [...]]

Warning: will not automatically overwrite the output file.

Example:
  ''' + commstr + ''' logfile.log my_output.csv CAMERA_RGB bot_core.image_t msg.width
  ''' + commstr + ''' logfile.log my_output.csv _AF_BATTERY mithl.battery_status_t msg.utime msg.cell_voltage[0]
'''

args = sys.argv
if len(args) < 1+5:
    print (USAGE)
    sys.exit(1)

in_filename = sys.argv[1]
out_filename = sys.argv[2]
listen_channel = sys.argv[3]
lcmtype_str = sys.argv[4]
field_expr_str_lst = sys.argv[5:]

if os.path.exists(out_filename):
    print ('File exists: %s' % out_filename)
    sys.exit(1)

out_file = open(out_filename,'w')

print 'LCM Writer Setup Success.  Writing to file ' + out_filename + '...'

PRINT_LIMIT_SECONDS = 0.1
last_time = time.time()

try:
    def write_handler(channel, data):
        global last_time
        cur_time = time.time()

        if cur_time - last_time > PRINT_LIMIT_SECONDS:
            print('Received...')
            last_time = time.time()
        
        msg = eval(lcmtype_str + '.decode(data)')
        for i, field_expr_str in enumerate(field_expr_str_lst):
            field = eval(field_expr_str)
            out_file.write(str(field) + (',' if (i < len(field_expr_str_lst)-1) else '\n'))

    S1 = 'file://'
    S2 = in_filename
    S3 = '?speed=1000000000.0'

    fullurl = S1 + S2 + S3

    print (fullurl)
    print ('Creating lcm file reader:')
    lcfile = lcm.LCM(fullurl)
    subs = lcfile.subscribe(listen_channel, write_handler)

    while(True):
        lcfile.handle()

except IOError:
    out_file.close()
    print 'End of logfile.  Writer Terminated'    

