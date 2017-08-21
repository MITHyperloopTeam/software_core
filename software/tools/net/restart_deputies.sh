#!/bin/bash

killall -qw bot-procman-deputy
daemon --command="bot-procman-deputy -n $1 -u udpm://239.255.76.67:62237" 2>&1 | tee ~/deputy_output.txt
until pids=$(pidof bot-procman-deputy); do sleep 0.1; done
echo started deputy on $1 with LCM url: udpm://239.255.76.67:62237