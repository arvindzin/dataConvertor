import socket
import sys

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect the socket to the port where the server is listening
server_address = ('localhost', 10000)
print >>sys.stderr, 'connecting to %s port %s' % server_address
sock.connect(server_address)
fn = open("file1.txt", "r")
message = fn.readline()
try:
    
    # Send data
    
    while message:
        print >>sys.stderr, 'sending "%s"' % message
        sock.sendall(message)
        # Look for the response
        amount_received = 0
        amount_expected = len(message)
    
        while amount_received < amount_expected:
            data = sock.recv(16)
            amount_received += len(data)
            print >>sys.stderr, 'received "%s"' % data
        message = fn.readline()
finally:
    print >>sys.stderr, 'closing socket'
    fn.close
    sock.close()
