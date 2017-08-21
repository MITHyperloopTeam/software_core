import serial
import sys

dev = sys.argv[1]
print "About to touch..."
ser = serial.Serial(dev, 1200)
print "Touching ", dev
ser.write("")
ser.close()