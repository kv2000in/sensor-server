#include "LoRa.h"
#include <stdio.h>
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
#include <linux/filter.h>
#include <sys/un.h>

#define UDS_PATH "/tmp/raw_socket_uds_esp"
#define BUFFER_SIZE 2048
#define PACKET_LENGTH 400 //Approximate
#define MYDATA 18         //0x12
#define MAX_PACKET_LEN 1000
int counter = 0;

void setup() {

  printf("LoRa Receiver\n");
  LoRa.setPins(10, 3, 7);
  
  if (!LoRa.begin(439E6, 0)) {
    printf("Starting LoRa failed!\n");
    while (1);
  }
  printf("Init LoRa Done !!\n");
}

void printPacketHex(const uint8_t *packet, size_t length) {
	printf("Packet (Hex): ");
	for (size_t i = 0; i < length; i++) {
		if (i > 0) printf(" ");
		printf("%02X", packet[i]);
	}
	
}

void loop() {
// try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
     printf("Received packet '");

    // read packet
    while (LoRa.available()) {
      //printf("%c", (char)LoRa.read());
		printf("%02X", LoRa.read());
	
    }

    // print RSSI of packet
     printf("' with RSSI ");
     printf("%d\n",LoRa.packetRssi());
    
  }
}


int  main(void) {
   setup();
   while(1) loop();
}
