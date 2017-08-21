#!/usr/bin/env python
"""
@brief Hyperloop team telemetry packet test server
@author Bob Urberger

Subject to the existing rights of third parties, SPACE EXPLORATION
TECHNOLOGIES is the owner of the copyright in this work and no portion
thereof is to be copied, reproduced or communicated to any person without
written permission.
"""
import socket
import struct

def parse_message(msg):
    params = {}
    pattern = '!BBi7I'
    (params['team_id'],
     params['status'],
     params['accel'],
     params['position'],
     params['velocity'],
     params['voltage'],
     params['current'],
     params['bat_temp'],
     params['pod_temp'],
     params['stripe_count'])= struct.unpack(pattern, msg)

    return params

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', 3000))

    while True:
        message = sock.recv(512)
        length = len(message)
        if length == 34:
            results = parse_message(message)
            print results
        else:
            print "Incorrect message length: {}".format(length)

if __name__ == "__main__":
    main()
