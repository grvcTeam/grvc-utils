#!/usr/bin/env python
# license removed for brevity

import socket, struct

UDP_IP = "127.0.0.1"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

while True:
    data = sock.recvfrom(1024) # buffer size is 1024 bytes
    data = struct.unpack('fffffffQ', data[0])

    print 'x= ', data[0], '\n',  'y= ', data[1], '\n', 'z= ', data[2], '\n','qx= ', data[3], '\n','qy= ', data[4], '\n','qz= ', data[5], '\n','qw= ', data[6], '\n','T= ', data[7], '\n---------\n', 