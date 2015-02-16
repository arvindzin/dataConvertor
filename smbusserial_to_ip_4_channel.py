#!/usr/bin/env python

import sys
import os
import time
import thread
import threading
import socket
import codecs
import serial
import smbus

try:
    True
except NameError:
    True = 1
    False = 0

class Redirector:
    def __init__(self, serial_instance, serverip, serverport, clientip, clientport, spy=False):
        self.serial = serial_instance
        self.serverport = serverport
        self.clientport = clientport
        self.serverip   = serverip
        self.clientip   = clientip
        #self.socket = self.launchServer(serverip, serverport)
        #self.socketClient = self.launchClient()
        self.spy = spy
        self._write_lock = threading.Lock()

    def serial_ip_bridge(self):
        """connect the serial port to the TCP port by copying everything
           from one side to the other"""
        self.alive = True
        self.thread_read = threading.Thread(target=self.serial_to_ip)
        self.thread_read.setDaemon(True)
        self.thread_read.setName('serial->socket')
        self.thread_read.start()
        self.ip_to_serial()

    def serial_to_ip(self):
        """loop forever and copy serial->socket"""
        self.socket = self.launchServer()
        while self.alive:
            try:
                data = self.serial.read(1)              # read one, blocking
                n = self.serial.inWaiting()             # look if there is more
                if n:
                    data = data + self.serial.read(n)   # and get as much as possible
                if data:
                    # the spy shows what's on the serial port, so log it before converting newlines
                    if self.spy:
                        sys.stdout.write(codecs.escape_encode(data)[0])
                        sys.stdout.flush()
                   # escape outgoing data when needed (Telnet IAC (0xff) character)
                    self._write_lock.acquire()
                    try:
                        self.socket.sendall(data)           # send it over TCP
                    finally:
                        self._write_lock.release()
            except socket.error, msg:
                sys.stderr.write('ERROR: %s\n' % msg)
                # probably got disconnected
                break
        self.alive = False

    def write(self, data):
        """thread safe socket write with no data escaping. used to send telnet stuff"""
        self._write_lock.acquire()
        try:
            self.socket.sendall(data)
        finally:
            self._write_lock.release()

    def ip_to_serial(self):
        """loop forever and copy socket->serial"""
        self.socketClient = self.launchClient()
        while self.alive:
            try:
                data = self.socketClient.recv(1024)
                if not data:
                    break
                self.serial.write(data)                 # get a bunch of bytes and send them
                # the spy shows what's on the serial port, so log it after converting newlines
                if self.spy:
                    sys.stdout.write(codecs.escape_encode(data)[0])
                    sys.stdout.flush()
            except socket.error, msg:
                sys.stderr.write('ERROR: %s\n' % msg)
                # probably got disconnected
                break
        self.alive = False
        self.thread_read.join()
        
    def launchServer(self):  
        ip = self.serverip
        port = self.serverport  
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_address = (ip, port)
        print >>sys.stderr, 'starting up on %s port %s' % server_address 
        sock.bind(server_address)
        sock.listen(1)
        while True:
            print >>sys.stderr, 'waiting...'
            connection, client_address = sock.accept()
            try:
                print >>sys.stderr, 'connected', client_address
                return connection
            except:
                print "server connection failed"
            #finally:
            #    connection.close()
                
    def launchClient(self):
        ip = self.clientip
        port = self.clientport
        server_address = (ip, port)
        print >>sys.stderr, 'connecting to %s port %s' % server_address
        while True:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)        
            try:
                sock.connect(server_address)
            except socket.error, msg:
                sock.close()
            else:
                return sock
    
class smbusControl:
    gpio = smbus.SMBus(2)
    def __init__(self, serverport, serverip, clientport, clientip, address, gpiodirection):
        self.gpio_device_addr = address
        self.serverport = serverport
        self.clientport = clientport
        self.serverip   = serverip
        self.clientip   = clientip
        self.gpiodirection = gpiodirection
        self._write_lock = threading.Lock()
        
    def writeRegisterPair(self,reg,value):
        low = value & 0xff
    	high = (value >>  8) & 0xff
    	self.gpio.write_byte_data(self.gpio_device_addr,reg,low)
    	self.gpio.write_byte_data(self.gpio_device_addr,reg+1,high)
			
    # read a 16 bit value to register pair
    def readRegisterPair(self,reg):
    	low = self.gpio.read_byte_data(self.gpio_device_addr,reg)
    	high = self.gpio.read_byte_data(self.gpio_device_addr,reg+1)
    	return low | (high << 8)
			
    # set IO ports to input if the corresponding direction bit is 1.
    # otherwise set it to output
    def setGpioDirection(self,mask):
    	self.writeRegisterPair(6,mask)
			
    # set the IO port outputs
    def writeGpio(self, value):
        self.writeRegisterPair(2,value)
    # read the IO port inputs
    def readGpio(self):
        return self.readRegisterPair(0)
        
    def gpio_ip_bridge(self):
        """connect the serial port to the TCP port by copying everything
           from one side to the other"""
        self.setGpioDirection(self.gpiodirection)   
        self.alive = True
        self.thread_read = threading.Thread(target=self.gpio_to_ip)
        self.thread_read.setDaemon(True)
        self.thread_read.setName('gpio->socket')
        self.thread_read.start()
        self.ip_to_gpio()
        
    def gpio_to_ip(self):
        self.socket = self.launchServer()
        while self.alive:
            try:
                data = self.readGpio()            # read one, blocking
                print >>sys.stderr, 'reading smbus', data
                self._write_lock.acquire()
                try:
                    self.socket.sendall(data)           # send it over TCP
                finally:
                    self._write_lock.release()
            except socket.error, msg:
                sys.stderr.write('ERROR: %s\n' % msg)
                # probably got disconnected
                break
        self.alive = False
        
    def ip_to_gpio(self):
        """loop forever and copy socket->serial"""
        self.socketClient = self.launchClient()
        while self.alive:
            try:
                data = self.socketClient.recv(2)
                self.writeGpio(data)                 # get a bunch of bytes and send them
                # the spy shows what's on the serial port, so log it after converting newlines
            except socket.error, msg:
                sys.stderr.write('ERROR: %s\n' % msg)
                # probably got disconnected
                break
        self.alive = False
        self.thread_read.join()
        
    def launchServer(self):  
        ip = self.serverip
        port = self.serverport  
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_address = (ip, port)
        print >>sys.stderr, 'starting up on %s port %s' % server_address 
        sock.bind(server_address)
        sock.listen(1)
        while True:
            print >>sys.stderr, 'waiting...'
            connection, client_address = sock.accept()
            try:
                print >>sys.stderr, 'connected', client_address
                return connection
            except:
                print "server connection failed"
            #finally:
            #    connection.close()
                
    def launchClient(self):
        ip = self.clientip
        port = self.clientport
        server_address = (ip, port)
        print >>sys.stderr, 'connecting to %s port %s' % server_address
        while True:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)        
            try:
                sock.connect(server_address)
            except socket.error, msg:
                sock.close()
            else:
                return sock    

def launchGpioInterface(serverport, clientport, serverip, clientip, smbus_device_address, gpio_mask):
    try:
        gpio = smbusControl(serverport,
               serverip,
               clientport,
               clientip,
               smbus_device_address,
               gpio_mask,
               )
        gpio.gpio_ip_bridge()
    except KeyboardInterrupt:
        sys.stderr.write('ERROR:')
    except socket.error, msg:
        sys.stderr.write('ERROR: %s\n' % msg)    

def launchinstance(serverport,clientport, serverip, clientip, serialPort): 
    while True:
        try:
            # enter network <-> serial loop
            r = Redirector(
                serialPort,
                serverip,
                serverport,
                clientip,
                clientport,
                True,
                )
            r.serial_ip_bridge()
            sys.stdout.write('\n')
            sys.stderr.write('Disconnected\n')
            connection.close()
        except KeyboardInterrupt:
            break
        except socket.error, msg:
            sys.stderr.write('ERROR: %s\n' % msg)

                #sys.stderr.write('\n--- exit ---\n')
    
#if __name__ == '__main':
print "starting"
#**************************************SETUP SERIAL PORTS *********************************************
# get port and baud rate from command line arguments or the option switches
baudrate = 115200
ser0 = serial.Serial('/dev/ttyS0', baudrate, timeout=5)
ser1 = serial.Serial('/dev/ttyS1', baudrate, timeout=5)
ser2 = serial.Serial('/dev/ttyS2', baudrate, timeout=5)
ser3 = serial.Serial('/dev/ttyUSB0', baudrate, timeout=5)
#    ser.parity   = options.parity
#    ser.rtscts   = options.rtscts
#    ser.xonxoff  = options.xonxoff


sys.stderr.write("--- TCP/IP to Serial redirector --- type Ctrl-C / BREAK to quit\n")
sys.stderr.write("--- %s %s,%s,%s,%s ---\n" % (ser0.portstr, ser0.baudrate, 8, ser0.parity, 1))
sys.stderr.write("--- %s %s,%s,%s,%s ---\n" % (ser1.portstr, ser1.baudrate, 8, ser1.parity, 1))
sys.stderr.write("--- %s %s,%s,%s,%s ---\n" % (ser2.portstr, ser2.baudrate, 8, ser2.parity, 1))
sys.stderr.write("--- %s %s,%s,%s,%s ---\n" % (ser3.portstr, ser3.baudrate, 8, ser3.parity, 1))

try:
    ser0.open()
except serial.SerialException, e:
    sys.stderr.write("Could not open serial port %s: %s\n" % (ser0.portstr, e))
    sys.exit(1)
try:
    ser1.open()
except serial.SerialException, e:
    sys.stderr.write("Could not open serial port %s: %s\n" % (ser1.portstr, e))
    sys.exit(1)
try:
    ser2.open()
except serial.SerialException, e:
    sys.stderr.write("Could not open serial port %s: %s\n" % (ser2.portstr, e))
    sys.exit(1)
try:
    ser3.open()
except serial.SerialException, e:
    sys.stderr.write("Could not open serial port %s: %s\n" % (ser3.portstr, e))
    sys.exit(1)

#try:
serverport1 = 10000
clientport1 = 10001
serverport2 = 10002
clientport2 = 10003
serverport3 = 10004
clientport3 = 10005
serverport4 = 10006
clientport4 = 10007
serverport5 = 10008
clientport5 = 10009

serverip = '192.168.2.3'
clientip = '192.168.2.4'
smbus_device_address = 0x20
gpio_mask = 0xffff
thread.start_new_thread( launchinstance, ( serverport1, clientport1, serverip, clientip, ser0, ))
thread.start_new_thread( launchinstance, ( serverport2, clientport2, serverip, clientip, ser1, ))
thread.start_new_thread( launchinstance, ( serverport3, clientport3, serverip, clientip, ser2, ))
thread.start_new_thread( launchinstance, ( serverport4, clientport4, serverip, clientip, ser3, ))
thread.start_new_thread( launchGpioInterface, (serverport5, clientport5, serverip, clientip, smbus_device_address, gpio_mask))
    
    

