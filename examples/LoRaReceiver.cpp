#include "LoRa.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>

#define UDS_PATH "/tmp/raw_socket_uds_lora"
#define BUFFER_SIZE 2048

int uds_sock, client_sock;

	// Function to set up Unix socket server
void setup_unix_socket() {
	struct sockaddr_un addr;
	
		// Remove any existing socket file
	unlink(UDS_PATH);
	
	uds_sock = socket(AF_UNIX, SOCK_STREAM, 0);
	if (uds_sock < 0) {
		perror("Unix socket creation failed");
		exit(1);
	}
	
	memset(&addr, 0, sizeof(addr));
	addr.sun_family = AF_UNIX;
	strncpy(addr.sun_path, UDS_PATH, sizeof(addr.sun_path) - 1);
	
	if (bind(uds_sock, (struct sockaddr *)&addr, sizeof(addr)) == -1) {
		perror("Binding Unix socket failed");
		close(uds_sock);
		exit(1);
	}
	
	if (listen(uds_sock, 1) == -1) {
		perror("Listening on Unix socket failed");
		close(uds_sock);
		exit(1);
	}
	
	printf("Waiting for Python to connect...\n");
	
		// Accept connection from Python
	client_sock = accept(uds_sock, NULL, NULL);
	if (client_sock == -1) {
		perror("Accepting connection failed");
		close(uds_sock);
		exit(1);
	}
	
	printf("Python connected to Unix socket\n");
}

void setup() {
	printf("LoRa Receiver\n");
	LoRa.setPins(10, 3, 7);
	
	if (!LoRa.begin(439E6, 0)) {
		printf("Starting LoRa failed!\n");
		while (1);
	}
	
	printf("Init LoRa Done!\n");
	setup_unix_socket(); // Start Unix socket server and wait for Python
}

void loop() {
	int packetSize = LoRa.parsePacket();
	if (packetSize) {
		uint8_t buffer[BUFFER_SIZE];
		int index = 0;
		
		//printf("Received LoRa packet: ");
		while (LoRa.available() && index < BUFFER_SIZE - 1) {
			uint8_t byte = LoRa.read();
			//printf("%02X ", byte);
			buffer[index++] = byte;
		}
		 
		buffer[index] = '\0'; // Null terminate for safety
		
		//printf("\n");
		
			// Send the LoRa packet data to Python via Unix socket
		if (send(client_sock, buffer, index, 0) < 0) {
			perror("Sending LoRa packet to Unix socket failed");
		} else {
			//printf("Sent %d bytes to Python via Unix socket\n", index);
		}
	}
}

int main(void) {
	setup();
	while (1) loop();
	
	close(client_sock);
	close(uds_sock);
	return 0;
}
