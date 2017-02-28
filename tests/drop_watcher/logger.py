import serial
import time
import sys

raw_input("Press Enter for start measuring\n")
t = time.time()
raw_input("Go! Press Enter again to stop\n")
t = time.time()-t

a = raw_input("Please enter the amount of measured water\n")
measured = float(a)

flow = measured/t*3.6

raw_input("Press enter for start drop watcher measurement")
T=20

t_end = time.time()+T
ser = serial.Serial("/dev/ttyUSB0", 115200)
filename = str(flow)+".log"
print "Measured flow (l/h): "+str(flow)
f = open("output/"+filename,'w')
while time.time() < t_end:
    line = ser.readline().replace("\n","")
    f.write(line)
f.close()
