/*
 To compile
pi@raspberrypi:~/Downloads/playground/sensor-server $ g++ -Wall -o LoRaDuplexWithRecvCallBack ./examples/LoRaDuplexWithRecvCallBack.cpp ./src/LoRa.cpp ./src/Print.cpp ./src/WString.cpp ./src/itoa.cpp -I ./src  -lwiringPi
 
 chmod +x LoRaDuplexWithRecvCallBack
 */

#include <signal.h>
#include "LoRa.h"
#include <stdio.h>
#include <stdint.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/select.h>

#define UDS_PATH "/tmp/raw_socket_uds_lora"
#define BUFFER_SIZE 2048

int uds_sock, client_sock;

const int csPin = 10;          // LoRa radio chip select
const int resetPin = 3;        // LoRa radio reset
const int irqPin = 7;          // change for your board; must be a hardware interrupt pin

// Function to send data over LoRa
void sendMessage(uint8_t *data, int len) {
	usleep(500000);
	LoRa.idle();  // Stop receiving before sending
	usleep(500000);
	printf("idle");
	LoRa.beginPacket();
	printf("begin");
	LoRa.write(data, len);
	printf("write");
	LoRa.endPacket(false);
	printf("END");
	LoRa.receive();  // Resume receiving after sending
	printf("END of sendMessage");
}

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

// Function to handle LoRa packet reception
void onReceive(int packetSize) {
    if (packetSize == 0) return;
    
    uint8_t buffer[BUFFER_SIZE];
    int index = 0;
    
    while (LoRa.available() && index < BUFFER_SIZE - 1) {
        buffer[index++] = LoRa.read();
    }
    
    if (index > 0) {
        buffer[index] = '\0'; // Null terminate for safety
        if (send(client_sock, buffer, index, 0) < 0) {
            perror("Sending LoRa packet to Unix socket failed");

        }

    }
}

// Function to receive data from Unix socket and send via LoRa



void receiveUnixSocket() {
	uint8_t buffer[BUFFER_SIZE];
	int bytesRead = recv(client_sock, buffer, BUFFER_SIZE, 0);

	if (bytesRead > 0) {
		printf("Received %d bytes from Python, sending via LoRa\n", bytesRead);

		// Ensure buffer does not exceed LoRa packet limit
		if (bytesRead > 255) {
			printf("Error: Packet size exceeds LoRa limit\n");
			return;
		}

		// Debug: Print received data
		printf("Sending %d bytes over LoRa: ", bytesRead);
		for (int i = 0; i < bytesRead; i++) {
			printf("%02X ", buffer[i]);
		}
		printf("\n");

		sendMessage(buffer, bytesRead);
		usleep(100000);  // Wait 100ms after sending
	} else if (bytesRead == 0) {
		printf("Python disconnected, waiting for reconnection...\n");
		close(client_sock);
		client_sock = accept(uds_sock, NULL, NULL);
		if (client_sock == -1) {
			perror("Accepting connection failed");
			close(uds_sock);
			exit(1);
		}
		printf("Python reconnected\n");
	} else {
		perror("Error receiving from Unix socket");
	}
}




void cleanup(int signum) {
    printf("Caught signal %d, cleaning up...\n", signum);
    
    // Close the sockets
    if (client_sock > 0) close(client_sock);
    if (uds_sock > 0) close(uds_sock);
    
    // Remove the Unix domain socket file
    unlink(UDS_PATH);
    
    printf("Cleanup complete. Exiting...\n");
    exit(0);
}

void setup_signal_handler() {
    struct sigaction sa;
    sa.sa_handler = cleanup;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    
    sigaction(SIGINT, &sa, NULL);  // Handle Ctrl+C
    sigaction(SIGTERM, &sa, NULL); // Handle termination signals
}

void setup() {
    printf("LoRa Duplex\n");
    
    setup_signal_handler();
    
    LoRa.setPins(csPin, resetPin, irqPin);
    
    if (!LoRa.begin(439E6, 0)) {
        printf("Starting LoRa failed!\n");
        while (1);
    }
    
    printf("Init LoRa Done!\n");
    setup_unix_socket();
    
    LoRa.onReceive(onReceive);
    LoRa.receive();
}

void loop() {
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(client_sock, &read_fds);
    int max_fd = client_sock;
    
    struct timeval timeout = {0, 100000};  // 100ms timeout
    
    int activity = select(max_fd + 1, &read_fds, NULL, NULL, &timeout);
    
    if (activity > 0) {
        if (FD_ISSET(client_sock, &read_fds)) {
            receiveUnixSocket();
        }
    }
    
    LoRa.receive(); // Continue receiving LoRa packets
}

int main(void) {
    setup();
    while (1) loop();
}
