#!/usr/bin/env python

import sys
import os
import time
import threading
import socket
import codecs
import serial
try:
    True
except NameError:
    True = 1
    False = 0

class Redirector:
    def __init__(self, serial_instance, socket, socketClient, spy=False):
        self.serial = serial_instance
        self.socket = socket
        self.socketClient = socketClient
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
                    if self.ser_newline and self.net_newline:
                        # do the newline conversion
                        # XXX fails for CR+LF in input when it is cut in half at the begin or end of the string
                        data = net_newline.join(data.split(ser_newline))
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
        while self.alive:
            try:
                data = self.socketClient.recv(1024)
                if not data:
                    break
                if self.ser_newline and self.net_newline:
                    # do the newline conversion
                    # XXX fails for CR+LF in input when it is cut in half at the begin or end of the string
                    data = ser_newline.join(data.split(net_newline))
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

    def stop(self):
        """Stop copying"""
        if self.alive:
            self.alive = False
            self.thread_read.join()

    # get port and baud rate from command line arguments or the option switches
    port = "/dev/ttyS0"
    baudrate = 115200

    # connect to serial port
    ser = serial.Serial()
    ser.port     = port
    ser.baudrate = baudrate
#    ser.parity   = options.parity
#    ser.rtscts   = options.rtscts
#    ser.xonxoff  = options.xonxoff
    ser.timeout  = 5     # required so that the reader thread can exit

    sys.stderr.write("--- TCP/IP to Serial redirector --- type Ctrl-C / BREAK to quit\n")
    sys.stderr.write("--- %s %s,%s,%s,%s ---\n" % (ser.portstr, ser.baudrate, 8, ser.parity, 1))

    try:
        ser.open()
    except serial.SerialException, e:
        sys.stderr.write("Could not open serial port %s: %s\n" % (ser.portstr, e))
        sys.exit(1)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #srv.bind( ('localhost', options.local_port) )
    srv.bind( ('192.168.2.3', 10000) )
    srv.listen(1)
    while True:
        try:
            sys.stderr.write("Waiting for connection on %s...\n" % 10000)
            connection, addr = srv.accept()
            sys.stderr.write('Connected by %s\n' % (addr,))
            time.sleep(2)
            clientAddr = ('192.168.2.4', 10001)
            client.connect(clientAddr)
            # enter network <-> serial loop
            r = Redirector(
                ser,
                connection,
                client,
                None,
                None,
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

    sys.stderr.write('\n--- exit ---\n')
