import serial
ser = serial.Serial("/dev/ttyUSB0", 115200)
while True:
    line = ser.readline().replace("\n","")
    print line
