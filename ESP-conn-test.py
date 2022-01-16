#!/usr/bin/python
# -*- coding: utf-8 -*-
#
import socket
import struct
import threading
from datetime import datetime
import binascii

global request

bind_ip = ''
bind_port = 9999

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind((bind_ip, bind_port))
server.listen(5)  # max backlog of connections
print 'Listening on {}:{}'.format(bind_ip, bind_port)


def handle_client_connection(client_socket):
	global request
	request = client_socket.recv(512)
	#if more than 16 bytes received - meaning data from both sensors.
	#encoded_string = request.encode()
	#byte_array = bytearray(encoded_string)
	#print byte_array
	'''
	fobj = open("/home/om/leveldata", 'a')
	fobj.write(request)
	fobj.write('\n')
	fobj.close() 
	'''
	client_socket.send('ACK!')
	client_socket.close()
	print "len of request= "+ str(len(request))
	#if len >16 - do it for every 16 bytes, discard duplicates
	if (len(request)>16):
		nofchunks=len(request)/16
		i=0
		while i < nofchunks:
			if (binascii.hexlify(request[0+i*16:6+i*16])!=binascii.hexlify(request[0:6])):	
				print "Received from "+ binascii.hexlify(request[0+i*16:6+i*16]) # 1st six bytes contain Sender Mac Address
				print "Distance =" +str( struct.unpack('h',request[6+i*16:8+i*16])[0]) # next two bytes 6:8 means 6 and 7 - distance as 2 bytes integer. struct.unpack will generate a tuple. [0] at the end selects first element of the tuple
				print "Temp =" + str(struct.unpack('h',request[8+i*16:10+i*16])[0])# need the str() to print int
				print "Battery ="+ str( struct.unpack('h',request[10+i*16:12+i*16])[0])
				print i
				break
			i+=1
		print "Received from "+ binascii.hexlify(request[0:6]) # 1st six bytes contain Sender Mac Address
		print "Distance =" +str( struct.unpack('h',request[6:8])[0]) # next two bytes 6:8 means 6 and 7 - distance as 2 bytes integer. struct.unpack will generate a tuple. [0] at the end selects first element of the tuple
		print "Temp =" + str(struct.unpack('h',request[8:10])[0])# need the str() to print int
		print "Battery ="+ str( struct.unpack('h',request[10:12])[0])
	else:
		print "Received from "+ binascii.hexlify(request[0:6]) # 1st six bytes contain Sender Mac Address
		print "Distance =" +str( struct.unpack('h',request[6:8])[0]) # next two bytes 6:8 means 6 and 7 - distance as 2 bytes integer. struct.unpack will generate a tuple. [0] at the end selects first element of the tuple
		print "Temp =" + str(struct.unpack('h',request[8:10])[0])# need the str() to print int
		print "Battery ="+ str( struct.unpack('h',request[10:12])[0])

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
