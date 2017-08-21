# From http://stackoverflow.com/questions/16040128/hook-up-raspberry-pi-via-ethernet-to-laptop-without-router
# Get IP on an interface
ifconfig eth3 | grep "Bcast" | awk -F: '{print $3}' | awk '{print $1}'
# That'll return an IP like 10.42.0.255
# Then use nmap to scan for the ODroid from there
nmap -n -sP 10.42.0.255/24
# It'll look for connected devices. ODroid will show up, at somewhere like
# 10.42.0.45. SSH into that as normal and it'll be happy.
