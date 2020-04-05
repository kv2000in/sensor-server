#!/usr/bin/python
# -*- coding: utf-8 -*-
#
import socket
import threading
from datetime import datetime

bind_ip = ''
bind_port = 9999

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind((bind_ip, bind_port))
server.listen(5)  # max backlog of connections

print 'Listening on {}:{}'.format(bind_ip, bind_port)


def handle_client_connection(client_socket):
	request = client_socket.recv(1024)
	print 'Received {}'.format(request)
	'''
	fobj = open("/home/om/leveldata", 'a')
	fobj.write(request)
	fobj.write('\n')
	fobj.close() 
	'''
	client_socket.send('ACK!')
	client_socket.close()

while True:
	try:
		client_sock, address = server.accept()
		print datetime.now()
		print 'Accepted connection from {}:{}'.format(address[0], address[1])
		client_handler = threading.Thread(
			target=handle_client_connection,
			args=(client_sock,)  # without comma you'd get a... TypeError: handle_client_connection() argument after * must be a sequence, not _socketobject
		)
		client_handler.start()
	except KeyboardInterrupt:
		server.shutdown(socket.SHUT_RDWR)
		server.close()
		print "TCP server closed"
