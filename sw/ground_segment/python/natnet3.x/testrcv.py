#!/usr/bin/env python3

import socket


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

#sock.bind(("127.0.0.1", 5005))
sock.bind(("192.168.1.238", 9090))

while True:

  data, addr = sock.recvfrom(1024)

  print(data)
