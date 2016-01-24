#!/usr/bin/env python
import socket
import serial
import select
from pymavlink import mavutil

source = ("/dev/ttyUSB0", 57600)
bind = ("0.0.0.0", 14555)
dest = [ ("192.168.40.255", 14550) ]

udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_sock.bind((bind[0],bind[1]))
udp_sock.setblocking(0)
udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

mav = mavutil.mavserial(source[0], baud=source[1], use_native=False)
#with serial.Serial(port=source[0], baudrate=source[1], rtscts=True, timeout=0, writeTimeout=0) as ser:
while True:
	readable, writable, exceptional = select.select([mav.fd, udp_sock], [], [], 1)
	if udp_sock in readable:
		msg = udp_sock.recv(4096)
		bytes = mav.write(msg)
		#print("From client: %s" % msg)
		#print("%i in : %i out" % (len(msg),bytes))
	if mav.fd in readable:
		msg = mav.recv_msg()
		if msg:
			#print(msg.get_msgbuf())
			for d in dest:
				while 1:                                        
					try:                                    
						bytes = udp_sock.sendto(msg.get_msgbuf(), (d[0],d[1] ))                                                                            
						break                           
					except:                                 
						pass  
			#print("%i in : %i out" % (len(msg),bytes))
		#print("From serial: %s" % msg.encode('hex'))
