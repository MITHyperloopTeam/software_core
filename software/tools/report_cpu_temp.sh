while true; do
temp=$(cat /sys/devices/virtual/thermal/thermal_zone0/temp)
echo Reported $((temp/1000)) degrees C
sleep 1
done

