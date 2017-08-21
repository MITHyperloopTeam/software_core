#!/bin/bash
# Borrowed from OpenHumanoids!
# First arg is either "server" to run tunnel server,
# or the server IP.
if [[ "$1" == "server" ]]; then
command="bot-lcm-tunnel"
else
command="bot-lcm-tunnel $1 -l udpm://239.255.76.67:62237"
fi
killall -qw bot-lcm-tunnel
screen -D -m -S tunnel $command &
until pids=$(pidof bot-lcm-tunnel); do sleep 0.1; done
echo started lcm tunnel on `hostname`: $command