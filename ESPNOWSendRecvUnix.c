#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/if_arp.h>
#include <arpa/inet.h>
#include <assert.h>

#define UDS_PATH "/tmp/raw_socket_uds"

void setup_raw_socket(char *interface) {
	struct ifreq ifr;
	int sock = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
	
	if (sock < 0) {
		perror("Socket creation failed");
		exit(1);
	}
	
	memset(&ifr, 0, sizeof(struct ifreq));
	strncpy(ifr.ifr_name, interface, IFNAMSIZ - 1);
	
	if (ioctl(sock, SIOCGIFINDEX, &ifr) == -1) {
		perror("ioctl failed");
		close(sock);
		exit(1);
	}
	
	struct sockaddr_ll sa;
	memset(&sa, 0, sizeof(struct sockaddr_ll));
	sa.sll_ifindex = ifr.ifr_ifindex;
	
		// Receive data
	uint8_t buffer[2048];
	while (1) {
		int len = recvfrom(sock, buffer, sizeof(buffer), 0, (struct sockaddr *)&sa, sizeof(sa));
		if (len < 0) {
			perror("Receive failed");
		} else {
				// Send received data to Python via Unix Domain Socket
			int uds_sock = socket(AF_UNIX, SOCK_STREAM, 0);
			if (uds_sock < 0) {
				perror("Unix socket creation failed");
				continue;
			}
			
			struct sockaddr_un addr;
			memset(&addr, 0, sizeof(addr));
			addr.sun_family = AF_UNIX;
			strncpy(addr.sun_path, UDS_PATH, sizeof(addr.sun_path) - 1);
			
			if (connect(uds_sock, (struct sockaddr *)&addr, sizeof(addr)) == -1) {
				perror("Unix socket connection failed");
				close(uds_sock);
				continue;
			}
			
			if (send(uds_sock, buffer, len, 0) == -1) {
				perror("Send to Unix socket failed");
			}
			close(uds_sock);
		}
	}
	
	close(sock);
}

void send_raw_packet(char *sender_mac, char *destination_mac, int additional_bytes) {
		// Construct the raw packet and send data
	printf("Sending data: Sender MAC: %s, Destination MAC: %s, Additional Bytes: %d\n", sender_mac, destination_mac, additional_bytes);
		// Implement the raw socket sending logic based on the provided data
}

int main(int argc, char *argv[]) {
	if (argc < 2) {
		fprintf(stderr, "Usage: %s <interface>\n", argv[0]);
		exit(1);
	}
	
		// Setup Unix domain socket server
	int uds_sock = socket(AF_UNIX, SOCK_STREAM, 0);
	if (uds_sock < 0) {
		perror("Unix socket creation failed");
		exit(1);
	}
	
	struct sockaddr_un addr;
	memset(&addr, 0, sizeof(addr));
	addr.sun_family = AF_UNIX;
	strncpy(addr.sun_path, UDS_PATH, sizeof(addr.sun_path) - 1);
	
	unlink(UDS_PATH); // Unlink the socket if it exists
	if (bind(uds_sock, (struct sockaddr *)&addr, sizeof(addr)) == -1) {
		perror("Binding Unix socket failed");
		exit(1);
	}
	
	if (listen(uds_sock, 1) == -1) {
		perror("Unix socket listen failed");
		exit(1);
	}
	
	printf("Waiting for Python client...\n");
	
		// Accept incoming connection from Python client
	int client_sock = accept(uds_sock, NULL, NULL);
	if (client_sock == -1) {
		perror("Accept failed");
		exit(1);
	}
	
	char buffer[2048];
	while (1) {
		int bytes_received = recv(client_sock, buffer, sizeof(buffer), 0);
		if (bytes_received < 0) {
			perror("Unix socket receive failed");
			continue;
		}
		
		if (bytes_received == 0) {
			break; // Client disconnected
		}
		
			// Process the received data (sender_mac, destination_mac, additional_bytes)
		char sender_mac[18], destination_mac[18];
		int additional_bytes;
		sscanf(buffer, "%17s %17s %d", sender_mac, destination_mac, &additional_bytes);
		
			// Send raw packet based on received data
		send_raw_packet(sender_mac, destination_mac, additional_bytes);
	}
	
	close(client_sock);
	close(uds_sock);
	
		// Set up raw socket to receive packets
	setup_raw_socket(argv[1]);
	
	return 0;
}

