import serial
ser = serial.Serial('/dev/ttys1', 115200, timeout=5)
while True:
    x= ser.read(6)
    print(x)
ser.close()
