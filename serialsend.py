import serial
ser0 = serial.Serial('/dev/ttyS0')
ser1 = serial.Serial('/dev/ttyS1')
ser2 = serial.Serial('/dev/ttyS2')
ser3 = serial.Serial('/dev/ttyS3')
ser0.baudrate=115200
ser1.baudrate=115200
ser2.baudrate=115200
ser3.baudrate=115200
#while True
data="p0"
x= ser0.write(data)
data="p1"
x= ser1.write(data)
data="p2"
x= ser2.write(data)
data="p3"
x= ser3.write(data)

ser0.close()
ser1.close()
ser2.close()
ser3.close()