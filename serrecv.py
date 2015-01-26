import serial
import thread

ser0 = serial.Serial('/dev/ttys0', 115200, timeout=5)
ser1 = serial.Serial('/dev/ttys1', 115200, timeout=5)
ser2 = serial.Serial('/dev/ttys2', 115200, timeout=5)
ser3 = serial.Serial('/dev/ttys3', 115200, timeout=5)

def recvSerial(threadname, serial)
    print "%s" % ( threadName )
    while True:
        x= serial.read(6)
        print(x)
    serial.close()

try:
    thread.start_new_thread( recvSerial, ("p0", ser0, ))
    thread.start_new_thread( recvSerial, ("p1", ser1, ))
    thread.start_new_thread( recvSerial, ("p2", ser2, ))
    thread.start_new_thread( recvSerial, ("p3", ser3, ))
except:
    print "Error"
    
while 1:
    pass