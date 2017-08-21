import serial

port = serial.Serial("/dev/ttyS2", baudrate=115200, timeout=0.5)

from binascii import hexlify

while True:
   read = port.read()
   if len(read) > 0:
#     read = hexlify(read)
#     print int(read, 16)
      for chr in read:
         if chr >= 32 and chr <= 127:
            print chr,
   else: print ""
