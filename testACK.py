import serial
import time
import struct
from collections import deque
from math import sqrt
import socket
import subprocess

# Configuration
SERIAL_PORT = '/dev/serial0'  # Replace with your serial port
BAUD_RATE = 115200
MAC_ADDRESS_LENGTH = 6
PACKET_ID_LENGTH = 2
HEADER_LENGTH = MAC_ADDRESS_LENGTH + PACKET_ID_LENGTH + 3  # MAC + Packet ID + Total Packets, Sequence, Payload Length
FOOTER_LENGTH = 2  # Checksum length
PACKET_DELIMITER = '\x0D\x0A'  # Delimiter: 0D 0A
DATA_TYPE_HEARTBEAT = '\xFF'
DATA_TYPE_ADC0 = '\x10'
DATA_TYPE_ADC1 = '\x11'
# Path for the Unix Domain Socket
ESP_UDS_PATH = "/tmp/raw_socket_uds_esp"
LORA_UDS_PATH = "/tmp/raw_socket_uds_lora"

# ESP32 MAC address for identification
ESP32_MAC_ADDR = '\x58\xBF\x25\x82\x8E\xD8'  # Replace with the actual MAC address

RESET = '\x40'
ACK = '\x41'
LED_ON = '\x42'
LED_OFF = '\x43'

# Set to track received Packet IDs
PACKET_ID_TRACKER = deque(maxlen=50)  # FIFO queue with a max size of 50

# Dictionary to store segmented packets
pending_segments = {}

# Initialize a global variable to track LED state
led_state = False  # False means OFF, True means ON

#global Switch for using ESP01 via serial vs Direct Pi Zero Wifi Packet injection to connect with ESP nodes
ESP01 = False # True means using Serial, False means using raw socket packet injection

# Create a Unix domain socket to receive data from the C program
esp_uds_socket = None
lora_uds_socket = None

#Create a global Serial device
ser = None

# Circular buffer implementation
class CircularBuffer:
    def __init__(self, size):
        self.size = size
        self.buffer = [0] * size
        self.index = 0

    def add(self, value):
        self.buffer[self.index] = value
        self.index = (self.index + 1) % self.size

    def get_data(self):
        return self.buffer

# Initialize buffers and RMS variables for each channel
BUFFER_SIZE = 50
adc_channel_0 = CircularBuffer(BUFFER_SIZE)
adc_channel_1 = CircularBuffer(BUFFER_SIZE)
rms_channel_0 = 0
rms_channel_1 = 0


def calculate_checksum(data):
    """Calculate the checksum for the given data."""
    return sum(map(ord, data)) & 0xFFFF


def is_duplicate_packet(packet_id):
    """
    Check if a packet ID is a duplicate.
    If not a duplicate, add it to the tracker.
    """
    if packet_id in PACKET_ID_TRACKER:
        return True
    PACKET_ID_TRACKER.append(packet_id)
    return False


def validate_data(data):
    """
    Validate the received data based on checksum and other criteria.
    """
    # Print raw packet in hex
	#print_packet_hex(data)
    if len(data) < HEADER_LENGTH + FOOTER_LENGTH:
        return False, "Invalid data length."

    # Extract header components
    mac_addr = data[:MAC_ADDRESS_LENGTH]
    packet_id = struct.unpack('<H', data[MAC_ADDRESS_LENGTH:MAC_ADDRESS_LENGTH + PACKET_ID_LENGTH])[0]
    total_packets, sequence, payload_length = struct.unpack('BBB', data[MAC_ADDRESS_LENGTH + PACKET_ID_LENGTH:HEADER_LENGTH])
    payload_length = int(payload_length)
    # Define the position where the checksum starts (header + payload length byte)
    checksum_position = HEADER_LENGTH + payload_length

    # Validate checksum
    payload = data[HEADER_LENGTH:checksum_position]  # Payload is between the header and checksum
    checksum_received = struct.unpack('<H', data[checksum_position:checksum_position + FOOTER_LENGTH])[0]
    checksum_calculated = calculate_checksum(data[:checksum_position])  # Include header + payload for checksum calculation
    if checksum_received != checksum_calculated:
        return False, "Checksum mismatch."

    return True, {
        "mac_addr": mac_addr,
        "packet_id": packet_id,
        "total_packets": total_packets,
        "sequence": sequence,
        "payload_length": payload_length,
        "payload": payload,
    }


def process_data_esp32(packets):
    """
    Process ESP32 data packets, handling segmented and non-segmented data.
    """
    for packet in packets:
        packet_id = packet["packet_id"]
        total_packets = packet["total_packets"]
        sequence = packet["sequence"]
        payload = packet["payload"]
        
        # Calculate the range of expected packet IDs
        expected_packet_ids = set(packet_id + i for i in range(-sequence + 1, total_packets - sequence + 1))
        
        # Unique key for grouping packets
        key = tuple(sorted(expected_packet_ids))

        if total_packets == 1:
            # Non-segmented data
            data_type = payload[0]  # First byte is the data type
            call_handler(data_type, payload)
        else:
            # Segmented data
            if key not in pending_segments:
                pending_segments[key] = {
                    "total_packets": total_packets,
                    "received": {},
                    "start_time": time.time(),
                }

            # Add packet to the received dictionary
            pending_segments[key]["received"][sequence] = payload

            # Check if all packets are received
            if len(pending_segments[key]["received"]) == total_packets:
                # Reassemble packets in sequence order
                reassembled_payload = b''.join(
                    pending_segments[key]["received"][i] for i in range(1, total_packets + 1)
                )

                # Determine data type from the first byte of the first packet
                data_type = reassembled_payload[0]
                call_handler(data_type, reassembled_payload)

                # Remove entry from pending_segments
                del pending_segments[key]
            else:
                # Handle timeout for incomplete segments
                current_time = time.time()
                if current_time - pending_segments[key]["start_time"] > 5.0:  # 5-second timeout
                    # Concatenate received packets
                    partial_payload = b''.join(
                        pending_segments[key]["received"].get(i, b'') for i in range(1, total_packets + 1)
                    )
                    print("Timeout for segmented data: Missing packets for key {}. Proceeding with partial data.".format(key))

                    # Determine data type from partial data
                    data_type = partial_payload[0] if partial_payload else None
                    if data_type:
                        call_handler(data_type, partial_payload)

                    # Remove entry from pending_segments
                    del pending_segments[key]

def call_handler(data_type, payload):
    """
    Calls the appropriate handler based on the data type.
    """
    if data_type == DATA_TYPE_HEARTBEAT:
        process_esp32_heartbeat(payload)
    elif data_type in [DATA_TYPE_ADC0, DATA_TYPE_ADC1]:
        process_esp32_adc_data(payload)
    else:
        process_esp32_unknown_data_type(payload)
def process_esp32_heartbeat(payload):
    global rms_channel_0, rms_channel_1

    print("Heartbeat from ESP32")
    
    # Remove the first byte (data type)
    payload = payload[1:]

    # Extract ADC channel 0 and channel 1 data
    adc_data_0 = payload[:100]  # First 100 bytes
    adc_data_1 = payload[100:200]  # Next 100 bytes
    padding = payload[200:]  # Remaining bytes (padding)

    # Convert raw ADC data into 16-bit integers
    adc_samples_0 = [struct.unpack('<H', adc_data_0[i:i + 2])[0] for i in range(0, len(adc_data_0), 2)]
    adc_samples_1 = [struct.unpack('<H', adc_data_1[i:i + 2])[0] for i in range(0, len(adc_data_1), 2)]
    # Update circular buffers
    for sample in adc_samples_0:
        adc_channel_0.add(sample)
    for sample in adc_samples_1:
        adc_channel_1.add(sample)

    # Calculate RMS for each channel
    rms_channel_0 = sqrt(sum(x**2 for x in adc_channel_0.get_data()) / BUFFER_SIZE)
    rms_channel_1 = sqrt(sum(x**2 for x in adc_channel_1.get_data()) / BUFFER_SIZE)

    # Pass padding to the status bits handler
    handlestatusbits(padding)

    # Print or log the results
    print("Channel 0 RMS: {:.2f}".format(rms_channel_0))
    print("Channel 1 RMS: {:.2f}".format(rms_channel_1))
    print("Channel 0 Data: {}".format(adc_channel_0.get_data()))
    print("Channel 1 Data: {}".format(adc_channel_1.get_data()))

def handlestatusbits(padding):
    global led_state  # Use the global state variable
    # Process padding bytes
    #print(f"Handling status bits: {padding}")

    # Toggle LED state
    if led_state:
        send_msg_to_ESP32(ESP32_MAC_ADDR+LED_OFF)
    else:
        send_msg_to_ESP32(ESP32_MAC_ADDR+LED_ON)
    # Update the LED state
    led_state = not led_state

def process_esp32_adc_data(payload):
	print("ADC data")
def process_esp32_unknown_data_type(payload):
	print("unknown data type")
def process_data_other_node(payload):
    """
    Process data for other sensor nodes (different format).
    """
    print("Processed data from other node: {}".format(payload))


def send_ack(ser, mac_addr, packet_id):
    """
    Send acknowledgment for a packet.
    """
    import struct
    ack_message = mac_addr + b'\x41' + struct.pack('<H', packet_id)
    send_msg_to_ESP32(ack_message)
    print("Sent ACK for packet ID {}: {}".format(packet_id, ack_message))


def print_packet_hex(data):
    """
    Print received packet data in hexadecimal format.
    """
    print("Received Packet (Hex):", " ".join("{:02X}".format(ord(byte) if isinstance(byte, str) else byte) for byte in data))

def send_msg_to_ESP32(msg):
    if ESP01:
        try:
            ser.write(msg)
        except serial.SerialException as e:
            print("Serial error: {}".format(str(e)))
    else:
        try:
            esp_uds_socket.send(msg)
        except socket.error as e:
            print("UNIX socket connection error: {}".format(str(e)))
def receive_data_from_c_program():
    global esp_uds_socket
    data = b''
    try:
        while True:
            data = esp_uds_socket.recv(2048)
            if data:
                handlepacket(data[63:]) # Raw socket packets have 63 bytes of header compared with packets from Serial
                #print_packet_hex(data)
            time.sleep(0.1)  # Adjust if needed, based on how often data is expected
    except Exception as e:
        print("An error occurred: {}".format(e))
    except KeyboardInterrupt:
        print("Exiting program.")
    finally:
        try:
            esp_uds_socket.close()
        except NameError:
            pass

def receive_data_from_c_plus_program():
    global lora_uds_socket
    data = b''
    try:
        while True:
            data = lora_uds_socket.recv(2048)
            if data:
                handlepacket(data) # Raw socket packets have 63 bytes of header compared with packets from Serial
                print_packet_hex(data)
            time.sleep(0.1)  # Adjust if needed, based on how often data is expected
    except Exception as e:
        print("An error occurred: {}".format(e))
    except KeyboardInterrupt:
        print("Exiting program.")
    finally:
        try:
            lora_uds_socket.close()
        except NameError:
            pass

def handlepacket(packet):
    """
    Handles the processing of a received packet.
    Processes ESP32 packets and hands over non-ESP32 packets for further handling.
    """
    # Ensure the packet has at least 6 bytes for MAC address check
    if len(packet) < MAC_ADDRESS_LENGTH:
        print("Incomplete packet received. Skipping.")
        return  # Exit function for incomplete packet

    # Extract the MAC address
    mac_addr = packet[:MAC_ADDRESS_LENGTH]

    if mac_addr == ESP32_MAC_ADDR:
        print("ESP32 packet detected. MAC: {}".format(mac_addr.encode("hex").upper()))

        # Extract Packet ID
        try:
            packet_id = struct.unpack('<H', packet[MAC_ADDRESS_LENGTH:MAC_ADDRESS_LENGTH + PACKET_ID_LENGTH])[0]

            # Check for duplicate Packet ID
            if is_duplicate_packet(packet_id):
                print("Duplicate packet detected. Packet ID: {}".format(packet_id))
                return  # Exit function for duplicate packet

            # Validate the data
            is_valid, result = validate_data(packet)

            if not is_valid:
                print("Invalid ESP32 data: {}".format(result))
                return  # Exit function for invalid data

            # Process valid ESP32 packet
            process_data_esp32([result])

        except struct.error as e:
            print("Error processing ESP32 data: {}".format(e))

    else:
        # Hand over non-ESP32 data
        print("Non-ESP32 MAC detected: {}".format(mac_addr.encode("hex").upper()))
        process_data_other_node(packet)
def receive_data_from_serial():
    """Main function to handle serial communication."""
    global ser
    try:
        buffer = b'' 
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            print("Listening on {} at {} baud rate.".format(SERIAL_PORT, BAUD_RATE))
            # ser.write(ESP32_MAC_ADDR + RESET)
            while True:
                # Read incoming data
                if ser.in_waiting > 0:
                    buffer += ser.read(ser.in_waiting)

                    # Split buffer into packets based on delimiter
                    while PACKET_DELIMITER in buffer:
                        # Split the buffer into one packet and the rest
                        packet, buffer = buffer.split(PACKET_DELIMITER, 1)
                        handlepacket(packet)
                # Small delay to prevent busy-waiting
                time.sleep(0.1)

    except serial.SerialException as e:
        print("Serial error: {}".format(str(e)))

    except KeyboardInterrupt:
        print("Exiting program.")

    finally:
        try:
            ser.close()
        except NameError:
            pass



if __name__ == "__main__":
    try:
        subprocess.Popen(["./LoRaReceiver"])
        time.sleep(50)
        lora_uds_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        lora_uds_socket.connect(LORA_UDS_PATH)
        receive_data_from_c_plus_program()
    except socket.error as e:
        print("UNIX socket connection error: {}".format(str(e)))
    if ESP01:
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            receive_data_from_serial()
        except serial.SerialException as e:
            print("Serial error: {}".format(str(e)))
    else:
        try:
            subprocess.Popen(["./mysendrecv.o", "mon0"])
            time.sleep(5)
            esp_uds_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            esp_uds_socket.connect(ESP_UDS_PATH)
            receive_data_from_c_program()
        except socket.error as e:
            print("UNIX socket connection error: {}".format(str(e)))
