#!/usr/bin/python
'''
Author: geronm@mit.edu and gizatt@mit.edu

This is a script to grab fields from LCM channels and write them to ascii files.

'''
import os.path
import sys
import time
import random
import struct

import lcm

import yaml
import binascii

import bot_core
import mithl

commstr = sys.argv[0]
USAGE = '''
USAGE: ''' + commstr + ''' input_log_file output_csv_file_directory input_yaml_file [starttime] [endtime]

Warning: will not automatically overwrite the output directory.

Example:
  ''' + commstr + ''' logfile.log out_directory test_output.yaml
'''

args = sys.argv
if len(args) < 1+3:
    print (USAGE)
    sys.exit(1)

in_filename = sys.argv[1]
out_dir = sys.argv[2]
in_yaml = sys.argv[3]
starttime = -1
endtime = -1
if len(sys.argv) >= 5:
    starttime = float(sys.argv[4])
if len(sys.argv) >= 6:
    endtime = float(sys.argv[5])

with open(in_yaml, 'r') as stream:
    try:
        yaml_config = yaml.load(stream)
    except yaml.YAMLError as exc:
        print(exc)

print yaml_config

# make output directory
if os.path.exists(out_dir):
    print ('Directory exists: %s' % out_dir)
    sys.exit(1)
os.mkdir(out_dir)

# generate output files and dummy msgs for each channel we'll listen for
message_file_dict = {}
message_instance_dict = {}
for channel in yaml_config.keys():
    message_file_dict[channel] = open(os.path.join(out_dir, channel + ".csv"), "w")
    field_name_strings = ["recv_timestamp"]
    for field_name in yaml_config[channel]["fields"]:
        field_name_strings.append(field_name)
    message_file_dict[channel].write(",".join(field_name_strings) + "\n")
    message_instance_dict[channel] = eval(yaml_config[channel]["type"] + "()")


global lc

print 'LCM Writer Setup Success.  Writing to ... '
for field in message_file_dict.keys():
    print "\t *** " + out_dir + '/' + field + ".csv"

try:
    t0 = -1
    with open(in_filename, "rb") as f:
        while 1:
            # read in header
            bytestr = f.read(4)
            if len(bytestr) == 4:
                syncword = struct.unpack(">L", bytestr[0:4])[0]
                if syncword != 0xeda1da01:
                    print "Corrupted LCMlog -- bad sync word."
                else:
                    bytestr = f.read(24)
                    eventnumber = struct.unpack(">q", bytestr[0:8])[0]
                    timestamp = struct.unpack(">q", bytestr[8:16])[0]
                    channellen = struct.unpack(">L", bytestr[16:20])[0]
                    datalen = struct.unpack(">L", bytestr[20:24])[0]
                    channelname = f.read(channellen)
                    buf = f.read(datalen)

                    if t0 < 0:
                        t0 = timestamp
                    in_time_range = True
                    if (starttime > 0 and timestamp - t0 < starttime*1000*1000):
                        in_time_range = False
                    if (endtime > 0 and timestamp - t0 > endtime*1000*1000):
                        in_time_range = False

                    if in_time_range and channelname in yaml_config.keys():
                        #try:
                            msg = message_instance_dict[channelname].decode(buf)
                            vals = [eval("msg." + field) for field in yaml_config[channelname]["fields"] ]
                            vals.insert(0, timestamp)
                            message_file_dict[channelname].write(",".join([str(val) for val in vals]) + "\n")
                        #except Exception as ve:
                        #    print "Error, probably in decoding... skipping: ", ve

            else:
                print "Reached end of file"
                break



except IOError as e:
    print "End of logfile, error ", e, ".  Writer Terminated"    

for key in message_file_dict.keys():
    message_file_dict[key].close()
