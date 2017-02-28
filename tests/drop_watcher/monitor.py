import serial
import time
import sys

ser = serial.Serial("/dev/ttyUSB0", 115200)
while True:
    line = ser.readline().replace("\n","")
    print line
f.close()
