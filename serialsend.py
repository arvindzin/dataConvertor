import serial
import time

ser0 = serial.Serial('/dev/ttyS0')
ser1 = serial.Serial('/dev/ttyS1')
ser2 = serial.Serial('/dev/ttyS2')
ser3 = serial.Serial('/dev/ttyS3')
ser0.baudrate=115200
ser1.baudrate=115200
ser2.baudrate=115200
ser3.baudrate=115200
while True:
    data="From port0"
    x= ser0.write(data)
    data="From port1"
    x= ser1.write(data)
    data="From port2"
    x= ser2.write(data)
    data="From port3"
    x= ser3.write(data)
    time.sleep(3)
ser0.close()
ser1.close()
ser2.close()
ser3.close()