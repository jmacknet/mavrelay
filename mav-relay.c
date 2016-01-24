#include <sys/select.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <fcntl.h>

#include "mavlink/ardupilotmega/mavlink.h"

#define BUF_SIZE 1024

char serial_name[] = "/dev/ttyUSB0";
unsigned int serial_baud = 57600;

char udp_dest[] = "192.168.40.255";
unsigned short udp_send_port = 14550;
unsigned short udp_bind_port = 14555;

unsigned short tcp_port = 14550;

char log_location[] = "/tmp/relay.txt";
int log_interval = 100;

int f_exit = 0;

int open_serial() {
	struct termios tty;
	int fd;
	
	// Open serial port
	fd = open(serial_name, O_RDWR);
	
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0)
	{
		perror("error from tcgetattr");
		return -1;
	}
	
	cfsetspeed(&tty, serial_baud);		// set baud rate
	cfmakeraw(&tty);		// make raw, uses BSD extensions

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag &= ~CSTOPB;		// one stop bit (disable 2 stop bits)
	tty.c_cflag &= ~CRTSCTS;	// disable flow control

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
	{
		perror("error from tcsetattr");
		return -1;
	}
	
	return fd;
}

int open_udp() {
	int s, bcast;
	struct sockaddr_in saddr;
	
	// create a UDP socket
    if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        perror("udp socket creation");
        return -1;
    }
    
    // set up sockaddr structure
    memset(&saddr, 0, sizeof(saddr));
    saddr.sin_family = AF_INET;
    saddr.sin_port = htons(udp_bind_port);
    saddr.sin_addr.s_addr = htonl(INADDR_ANY);
    
    // bind socket to port
    if( bind(s , (struct sockaddr*)&saddr, sizeof(saddr) ) == -1)
    {
        perror("UDP socket bind");
        return -1;
    }
   
	bcast = 1; 
    // Turn on broadcast
    if( setsockopt(s, SOL_SOCKET, SO_BROADCAST, &bcast, sizeof(bcast)) ) {
		perror("UDP socket broadcast set");
        return -1;
    }
    
    return s;
}

int open_tcp() {
	int s;
	struct sockaddr_in saddr;
	
	// create a TCP socket
    if ((s=socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) == -1)
    {
        perror("tcp socket creation");
        return -1;
    }
    
    // set up sockaddr structure
    memset(&saddr, 0, sizeof(saddr));
    saddr.sin_family = AF_INET;
    saddr.sin_port = htons(tcp_port);
    saddr.sin_addr.s_addr = htonl(INADDR_ANY);
    
    // bind socket to port
    if( bind(s , (struct sockaddr*)&saddr, sizeof(saddr) ) == -1)
    {
        perror("TCP socket bind");
        return -1;
    }
    
    // listen for incoming connections
	if( listen(s , 1) == -1)
    {
        perror("TCP socket listen");
        return -1;
    }
    
    return s;
}

int get_fd_max(int ser, int udp, int tcps, int tcpc) {
	int max = -1;
	
	if (ser > max) max = ser;
	if (udp > max) max = udp;
	if (tcps > max) max = tcps;
	if (tcpc > max) max = tcpc;
	
	return max;
}

void sigint_handler(int sig) {
	printf("interrupt\n");
	f_exit = 1;
}

int main() {
	// Serial mavlink variables
	int fd_ser;
	mavlink_status_t status;
    mavlink_message_t msg;
    char ser_inbuf[BUF_SIZE];
    char ser_outbuf[BUF_SIZE];
    size_t mav_len;
	
	// UDP mavlink variables
	int fd_udp;
	struct sockaddr saddr;
	struct sockaddr_in dest;
	int saddr_len;
	char udp_inbuf[BUF_SIZE];
	socklen_t udp_len;
	
	// TCP mavlink variables
	int fd_tcp_srv;
	int fd_tcp_cli = -1;
	struct sockaddr_in tcp_dest;
	int tcp_dest_len;
	char tcp_inbuf[BUF_SIZE];
	int tcp_len;
	
	// Select variables
	fd_set fdset;
	int fd_max;
    struct timeval tv;
    int retval;
	
	// Status tracking
	uint16_t msg_count = 0;
	uint16_t drop_count = 0;
	uint16_t last_reported_count = 0;
	uint16_t last_reported_drop = 0;
	uint8_t last_seq;
	FILE *logfile;
	
	// Common variables
	size_t len;
	int i;
	
	// Setup signal handlers
	signal(SIGINT, sigint_handler);
	
	// Open targets
	if ((fd_ser = open_serial()) < 0) {
		perror("opening serial port");
		return -1;
	}
	if ((fd_udp = open_udp()) < 0) {
		perror("opening UDP socket");
		return -1;
	}
	if ((fd_tcp_srv = open_tcp()) < 0) {
		perror("opening TCP listen socket");
		return -1;
	}
	logfile = fopen(log_location, "w");
	if (logfile == NULL) {
		perror("opening log file");
		return -1;
	}
	
	// Setup destination address
	memset(&saddr, 0, sizeof(dest));
	dest.sin_family = AF_INET;
	dest.sin_port = htons(udp_send_port);
	if ( inet_aton(udp_dest, &dest.sin_addr) == 0 ) {
		perror("inet_aton failed");
		return -1;
	}
	
	// Main loop
	for (; !f_exit ;) {
		// Set up select variables
		fd_max = get_fd_max(fd_ser, fd_udp, fd_tcp_srv, fd_tcp_cli);
		FD_ZERO(&fdset);
		FD_SET(fd_ser, &fdset);
		FD_SET(fd_udp, &fdset);
		FD_SET(fd_tcp_srv, &fdset);
		if (fd_tcp_cli != -1)
			FD_SET(fd_tcp_cli, &fdset);
		tv.tv_sec = 5;
		tv.tv_usec = 0;
		
		// Select on input sources
		retval = select(fd_max + 1, &fdset, NULL, NULL, &tv);
		
		// Handle ready files
		if (retval > 0) {
			// Handle serial port
			if (FD_ISSET(fd_ser, &fdset)) {
				len = read(fd_ser, ser_inbuf, BUF_SIZE);
				for (i=0; i < len; ++i) {
					if (mavlink_parse_char(MAVLINK_COMM_0, ser_inbuf[i], &msg, &status)) {
						// Relay messages as needed
						mav_len = mavlink_msg_to_send_buffer(ser_outbuf, &msg);
						// UDP
						udp_len = sendto(fd_udp, ser_outbuf, mav_len, 0, (struct sockaddr *) &dest, sizeof(dest));
						if (udp_len < 0) {
							perror("UDP send");
						}
						// TCP
						if (fd_tcp_cli != -1) {
							//printf("trying tcp\n");
							tcp_len = send(fd_tcp_cli, ser_outbuf, mav_len, MSG_NOSIGNAL);
							if (tcp_len < mav_len) {
								perror("TCP write error");
							}
							//printf("sent tcp\n");
						}
						// Collect statistics for autopilot messages
						if (msg.sysid == 1) { 
							// Calculate statistics for status reporting
							uint8_t count = status.current_rx_seq - last_seq;
							msg_count += count;
							drop_count += count - 1;
							last_seq = status.current_rx_seq;
							if (last_reported_count + log_interval <= msg_count) {
								fprintf(logfile, "%i count / %i drop \n", msg_count - last_reported_count, drop_count - last_reported_drop);
								fflush(logfile);
								last_reported_count = msg_count;
								last_reported_drop = drop_count;
							}
						}
					}
				}
			}
			// Handle UDP socket
			if (FD_ISSET(fd_udp, &fdset)) {
				udp_len = recvfrom(fd_udp, udp_inbuf, BUF_SIZE, 0, &saddr, &saddr_len);
				write(fd_ser, udp_inbuf, udp_len);
			}
			// Handle TCP server socket
			if (FD_ISSET(fd_tcp_srv, &fdset)) {
				if (fd_tcp_cli == -1) {
					fd_tcp_cli = accept(fd_tcp_srv, (struct sockaddr *)&tcp_dest, &tcp_dest_len);
					if (fd_tcp_cli < 0) {
						perror("TCP socket connection failure");
					}
					printf("Got connection from %s\n", inet_ntoa(tcp_dest.sin_addr));
				} else {
					int tmpfd;
					tmpfd = accept(fd_tcp_srv, NULL, NULL);
					close(tmpfd);
					printf("Rejected incoming TCP connection (too many)\n");
				}
			}
			// Handle TCP client socket
			if (FD_ISSET(fd_tcp_cli, &fdset)) {
				tcp_len = read(fd_tcp_cli, tcp_inbuf, BUF_SIZE);
				if (tcp_len == 0) {
					printf("TCP client disconnect\n");
					close(fd_tcp_cli);
					fd_tcp_cli = -1;
				} else if (tcp_len < 0) {
					perror("tcp read");
				} else {
					write(fd_ser, tcp_inbuf, tcp_len);
				}					
			}
		} else {
			printf("select error: %i\n", retval);
		}
	}
	
	// Clean up
	close(fd_ser);
	close(fd_udp);
	close(fd_tcp_srv);
	if (fd_tcp_cli != -1)
		close(fd_tcp_cli);
	fclose(logfile);
	
	return 0;
}

/*
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
*/
