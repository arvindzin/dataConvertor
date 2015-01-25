import serial
ser = serial.Serial('/dev/ptys1')
ser.baudrate=115200
while True:
    data="zin"
    x= ser.write(data)
ser.close()
