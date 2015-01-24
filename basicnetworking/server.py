import socket
import sys

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address = ('localhost', 10000)
print >>sys.stderr, 'starting up on %s port %s' % server_address 
sock.bind(server_address)
sock.listen(1)

while True:
    print >>sys.stderr, 'waiting...'
    connection, client_address = sock.accept()
    try:
        print >>sys.stderr, 'connected', client_address
        while True:
            data = connection.recv(16)
            print >>sys.stderr, 'recvd "%s"' % data
            if data:
                print >>sys.stderr, 'sending data back'
                connection.sendall(data)
            else:
                print >>sys.stderr, 'done', client_address
                break
    finally:
        connection.close()
