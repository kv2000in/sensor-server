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

static uint8_t gu8a_dest_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
/*our MAC address*/
	//{0xF8, 0x1A, 0x67, 0xB7, 0xeB, 0x0B};

/*ESP8266 host MAC address*/
	//{0x84,0xF3,0xEB,0x73,0x55,0x0D};


	//filter action frame packets
	//Equivalent for tcp dump :
	//type 0 subtype 0xd0 and wlan[24:4]=0x7f18fe34 and wlan[32]=221 and wlan[33:4]&0xffffff = 0x18fe34 and wlan[37]=0x4
	//NB : There is no filter on source or destination addresses, so this code will 'receive' the action frames sent by this computer...
#define FILTER_LENGTH 20
static struct sock_filter bpfcode[FILTER_LENGTH] = {
	{ 0x30, 0, 0, 0x00000003 },	// ldb [3]	// radiotap header length : MS byte
	{ 0x64, 0, 0, 0x00000008 },	// lsh #8	// left shift it
	{ 0x7, 0, 0, 0x00000000 },	// tax		// 'store' it in X register
	{ 0x30, 0, 0, 0x00000002 },	// ldb [2]	// radiotap header length : LS byte
	{ 0x4c, 0, 0, 0x00000000 },	// or  x	// combine A & X to get radiotap header length in A
	{ 0x7, 0, 0, 0x00000000 },	// tax		// 'store' it in X
	{ 0x50, 0, 0, 0x00000000 },	// ldb [x + 0]		// right after radiotap header is the type and subtype
	{ 0x54, 0, 0, 0x000000fc },	// and #0xfc		// mask the interesting bits, a.k.a 0b1111 1100
	{ 0x15, 0, 10, 0x000000d0 },	// jeq #0xd0 jt 9 jf 19	// compare the types (0) and subtypes (0xd)
	{ 0x40, 0, 0, 0x00000018 },	// Ld  [x + 24]			// 24 bytes after radiotap header is the end of MAC header, so it is category and OUI (for action frame layer)
	{ 0x15, 0, 8, 0x7f18fe34 },	// jeq #0x7f18fe34 jt 11 jf 19	// Compare with category = 127 (Vendor specific) and OUI 18:fe:34
	{ 0x50, 0, 0, 0x00000020 },	// ldb [x + 32]				// Begining of Vendor specific content + 4 ?random? bytes : element id
	{ 0x15, 0, 6, 0x000000dd },	// jeq #0xdd jt 13 jf 19		// element id should be 221 (according to the doc)
	{ 0x40, 0, 0, 0x00000021 },	// Ld  [x + 33]				// OUI (again!) on 3 LS bytes
	{ 0x54, 0, 0, 0x00ffffff },	// and #0xffffff			// Mask the 3 LS bytes
	{ 0x15, 0, 3, 0x0018fe34 },	// jeq #0x18fe34 jt 16 jf 19		// Compare with OUI 18:fe:34
	{ 0x50, 0, 0, 0x00000025 },	// ldb [x + 37]				// Type
	{ 0x15, 0, 1, 0x00000004 },	// jeq #0x4 jt 18 jf 19			// Compare type with type 0x4 (corresponding to ESP_NOW)
	{ 0x6, 0, 0, 0x00040000 },	// ret #262144	// return 'True'
	{ 0x6, 0, 0, 0x00000000 },	// ret #0	// return 'False'
};

void print_packet(uint8_t *data, int len)
{
	printf("----------------------------new packet-----------------------------------\n");
	int i;
	for (i = 0; i < len; i++)
	{
		if (i % 16 == 0)
		printf("\n");
		printf("0x%02x, ", data[i]);
	}
	printf("\n\n");
}
#define SEQ_BUFFER_SIZE 30  // Store last 30 sequence numbers

uint16_t seq_buffer[SEQ_BUFFER_SIZE] = {0};  // Circular buffer for sequence numbers
int seq_index = 0;  // Tracks the position in buffer

	// Function to check if sequence number exists in buffer
int is_duplicate_seq(uint16_t seq_num) {
	for (int i = 0; i < SEQ_BUFFER_SIZE; i++) {
		if (seq_buffer[i] == seq_num) {
			return 1; // Found duplicate
		}
	}
	return 0; // Unique sequence
}

	// Function to add a new sequence number to the circular buffer
void add_sequence(uint16_t seq_num) {
	seq_buffer[seq_index] = seq_num;
	seq_index = (seq_index + 1) % SEQ_BUFFER_SIZE; // Circular index
}


int create_raw_socket(char *dev, struct sock_fprog *bpf)
{
	struct sockaddr_ll sll;
	struct ifreq ifr;
	int fd, ifi, rb, attach_filter;
	
	bzero(&sll, sizeof(sll));
	bzero(&ifr, sizeof(ifr));
	
	(void)memset(&sll, 0, sizeof(sll));
	
	fd = socket(PF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
	assert(fd != -1);
	
	strncpy((char *)ifr.ifr_name, dev, IFNAMSIZ);
	ifi = ioctl(fd, SIOCGIFINDEX, &ifr);
	assert(ifi != -1);
	
	sll.sll_protocol = htons(ETH_P_ALL);
	sll.sll_family = PF_PACKET;
	sll.sll_ifindex = ifr.ifr_ifindex;
	sll.sll_pkttype = PACKET_HOST;
	sll.sll_hatype = ARPHRD_ETHER;
	sll.sll_halen = ETH_ALEN;
	sll.sll_addr[0] = gu8a_dest_mac[0];
	sll.sll_addr[1] = gu8a_dest_mac[1];
	sll.sll_addr[2] = gu8a_dest_mac[2];
	sll.sll_addr[3] = gu8a_dest_mac[3];
	sll.sll_addr[4] = gu8a_dest_mac[4];
	sll.sll_addr[5] = gu8a_dest_mac[5];
	sll.sll_addr[6] = 0x00; // not used
	sll.sll_addr[7] = 0x00; // not used
	
	rb = bind(fd, (struct sockaddr *)&sll, sizeof(sll));
	assert(rb != -1);
	
	attach_filter = setsockopt(fd, SOL_SOCKET, SO_ATTACH_FILTER, bpf, sizeof(*bpf));
	assert(attach_filter != -1);
	
	return fd;		
}

uint8_t senderMAC[6];
uint8_t destinationMAC[6];
uint8_t additional_byte;




int main(int argc, char **argv)
{
	uint8_t buffer[BUFFER_SIZE];
	fd_set readfds;
	int max_fd;
	
	assert(argc == 2);
	if (argc < 2)
	{
		fprintf(stderr, "Usage: %s <interface>\n", argv[0]);
		return EXIT_FAILURE;
	}
	
	
	
	
	
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
		// Accept a connection on the UNIX socket
	int uds_conn = accept(uds_sock, NULL, NULL);
	assert(uds_conn != -1);
	
	
	int sock_fd;
	char *dev = argv[1];
	struct sock_fprog bpf = {FILTER_LENGTH, bpfcode};
	
	sock_fd = create_raw_socket(dev, &bpf); /* Creating the raw socket */
	
	printf("\n Waiting to receive ESPNOW packets........ \n");
	
	while (1) {
		FD_ZERO(&readfds);
		FD_SET(uds_conn, &readfds);
		FD_SET(sock_fd, &readfds);
		
		max_fd = (uds_conn > sock_fd) ? uds_conn : sock_fd;
		
		int activity = select(max_fd + 1, &readfds, NULL, NULL, NULL);
		if (activity < 0) {
			perror("select");
			break;
		}
		
			// Handle data from UNIX socket
		if (FD_ISSET(uds_conn, &readfds)) {
			int bytes_read = read(uds_conn, buffer, BUFFER_SIZE);
			if (bytes_read > 0) {
				printf("Received data on UNIX socket: %d bytes\n", bytes_read);
				if (bytes_read==7){
						// Forward to raw socket
					memcpy(destinationMAC, buffer, 6);
					additional_byte = buffer[6];
						// Replace data array values dynamically
					uint8_t data[78] = {
						0x00, 0x00, 0x26, 0x00, 0x2f, 0x40, 0x00, 0xa0, 0x20, 0x08, 0x00, 0xa0, 0x20, 0x08, 0x00, 0x00,
						0xdf, 0x32, 0xfe, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x10, 0x0c, 0x6c, 0x09, 0xc0, 0x00, 0xd3, 0x00,
						0x00, 0x00, 0xd3, 0x00, 0xc7, 0x01, 0xd0, 0x00, 0x3a, 0x01, 0x58, 0xbf, 0x25, 0x82, 0x8e, 0xd8,
						0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x58, 0xbf, 0x25, 0x82, 0x8e, 0xd8, 0x70, 0x51, 0x7f, 0x18,
						0xfe, 0x34, 0xa2, 0x03, 0x92, 0xb0, 0xdd, 0x06, 0x18, 0xfe, 0x34, 0x04, 0x01,
						additional_byte 
					};
					for (int i = 0; i < 6; i++) {
						data[42 + i] = destinationMAC[i]; // Replace destinationMAC in data[42] to data[47]
						data[54 + i] = destinationMAC[i]; // Replace destinationMAC in data[54] to data[59]
					}
					
					printf("Sending to ESP32\n");
					sendto(sock_fd, data, sizeof(data), 0,NULL, 0);
				} else {
					printf("Who knows what I have received");
				}
			} else if (bytes_read == 0) {
				printf("UNIX socket closed by client\n");
				break;
			} else {
				perror("read");
			}
		}
		
			// Handle data from raw Ethernet socket
		if (FD_ISSET(sock_fd, &readfds)) {



			
		
		
		
		
		
			int bytes_read = recv(sock_fd, buffer, BUFFER_SIZE, 0);
			if (bytes_read > 0) {
				printf("Received data on raw Ethernet socket: %d bytes\n", bytes_read);
				
				if (bytes_read >= 50) {  // Ensure packet has at least 50 bytes
					uint16_t seq_num = (buffer[48] << 8) | buffer[49]; // Convert to 16-bit (big-endian)
					
					printf("Extracted Sequence Number: 0x%04X\n", seq_num);
					
					if (is_duplicate_seq(seq_num)) {
						printf("Duplicate packet detected! Dropping packet with sequence: 0x%04X\n", seq_num);
					} else {
						add_sequence(seq_num);
						send(uds_conn, buffer, bytes_read, 0);  // Forward only unique packets
					}
				} else {
					printf("Packet too short (%d bytes), forwarding by default.\n", bytes_read);
					send(uds_conn, buffer, bytes_read, 0);
				}
			} else {
				perror("recv");
			}
		
		
		
		
		
		
		}
	}
	
		// Cleanup
	close(uds_conn);
	close(uds_sock);
	close(sock_fd);
	unlink(UDS_PATH);
	return 0;
}



