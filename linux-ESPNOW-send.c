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

static uint8_t gu8a_dest_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

int create_raw_socket(char *dev)
{
	struct sockaddr_ll s_dest_addr; // code from sender
	struct ifreq ifr;
	int fd, ifi, rb;
	
	bzero(&s_dest_addr, sizeof(s_dest_addr));
	bzero(&ifr, sizeof(ifr));
	
	(void)memset(&s_dest_addr, 0, sizeof(s_dest_addr));
	
	fd = socket(PF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
	assert(fd != -1); // abort if error
	
	strncpy((char *)ifr.ifr_name, dev, IFNAMSIZ);
	ifi = ioctl(fd, SIOCGIFINDEX, &ifr);
	assert(ifi != -1); // abort if error
	
	s_dest_addr.sll_family = PF_PACKET;
	s_dest_addr.sll_protocol = htons(ETH_P_ALL);
	s_dest_addr.sll_ifindex = ifr.ifr_ifindex;
	s_dest_addr.sll_hatype = ARPHRD_ETHER;
	s_dest_addr.sll_pkttype = PACKET_OTHERHOST; // PACKET_OUTGOING
	s_dest_addr.sll_halen = ETH_ALEN;
	s_dest_addr.sll_addr[0] = gu8a_dest_mac[0];
	s_dest_addr.sll_addr[1] = gu8a_dest_mac[1];
	s_dest_addr.sll_addr[2] = gu8a_dest_mac[2];
	s_dest_addr.sll_addr[3] = gu8a_dest_mac[3];
	s_dest_addr.sll_addr[4] = gu8a_dest_mac[4];
	s_dest_addr.sll_addr[5] = gu8a_dest_mac[5];
	s_dest_addr.sll_addr[6] = 0x00; // not used
	s_dest_addr.sll_addr[7] = 0x00; // not used
	
	rb = bind(fd, (struct sockaddr *)&s_dest_addr, sizeof(s_dest_addr));
	assert(rb != -1); // abort if error
	
	return fd;
}

int main(int argc, char **argv)
{
	if (argc < 4)
	{
		fprintf(stderr, "Usage: %s <interface> <senderMACAddress> <destinationMACAddress> <additional_byte>\n", argv[0]);
		return EXIT_FAILURE;
	}
	
	char *dev = argv[1];
	uint8_t senderMAC[6];
	uint8_t destinationMAC[6];
	uint8_t additional_byte;
	
		// Parse senderMACAddress (argv[2]) and destinationMACAddress (argv[3])
	sscanf(argv[2], "\\x%02hhx\\x%02hhx\\x%02hhx\\x%02hhx\\x%02hhx\\x%02hhx", 
		   &senderMAC[0], &senderMAC[1], &senderMAC[2], &senderMAC[3], &senderMAC[4], &senderMAC[5]);
	
	sscanf(argv[3], "\\x%02hhx\\x%02hhx\\x%02hhx\\x%02hhx\\x%02hhx\\x%02hhx", 
		   &destinationMAC[0], &destinationMAC[1], &destinationMAC[2], &destinationMAC[3], &destinationMAC[4], &destinationMAC[5]);
	
		// Parse additional_byte (argv[4]) in the same format
	sscanf(argv[4], "\\x%02hhx", &additional_byte);
	
	int sock_fd = -1;
	int32_t s32_res = -1;
	
	sock_fd = create_raw_socket(dev); /* Creating the raw socket */
	if (-1 == sock_fd)
	{
		perror("Could not create the socket");
		goto LABEL_CLEAN_EXIT;
	}
	
	fflush(stdout);
	sleep(1);
	
	uint8_t data[82] = {
		0x00, 0x00, 0x26, 0x00, 0x2f, 0x40, 0x00, 0xa0, 0x20, 0x08, 0x00, 0xa0, 0x20, 0x08, 0x00, 0x00,
		0xdf, 0x32, 0xfe, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x10, 0x0c, 0x6c, 0x09, 0xc0, 0x00, 0xd3, 0x00,
		0x00, 0x00, 0xd3, 0x00, 0xc7, 0x01, 0xd0, 0x00, 0x3a, 0x01, 0x58, 0xbf, 0x25, 0x82, 0x8e, 0xd8,
		0x84, 0xcc, 0xa8, 0xa9, 0xe1, 0xe8, 0x58, 0xbf, 0x25, 0x82, 0x8e, 0xd8, 0x70, 0x51, 0x7f, 0x18,
		0xfe, 0x34, 0xa2, 0x03, 0x92, 0xb0, 0xdd, 0x06, 0x18, 0xfe, 0x34, 0x04, 0x01,
		additional_byte, 0x1c, 0xd5, 0x35, 0xd3 
	};
	
		// Replace data array values dynamically
	for (int i = 0; i < 6; i++) {
		data[42 + i] = destinationMAC[i]; // Replace destinationMAC in data[42] to data[47]
		data[54 + i] = destinationMAC[i]; // Replace destinationMAC in data[54] to data[59]
		data[48 + i] = senderMAC[i];      // Replace senderMAC in data[48] to data[53]
	}
	
	s32_res = sendto(sock_fd, data, sizeof(data), 0, NULL, 0);
	
	if (-1 == s32_res)
	{
		perror("Socket send failed");
		goto LABEL_CLEAN_EXIT;
	}
	
LABEL_CLEAN_EXIT:
	if (sock_fd > 0)
	{
		close(sock_fd);
	}
	
	printf("***** Packet sent with byte: 0x%02x *****\n", additional_byte);
	
	return EXIT_SUCCESS;
}

