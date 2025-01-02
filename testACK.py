import serial
import time
import struct
from collections import deque

# Configuration
SERIAL_PORT = '/dev/serial0'  # Replace with your serial port
BAUD_RATE = 115200
MAC_ADDRESS_LENGTH = 6
PACKET_ID_LENGTH = 2
HEADER_LENGTH = MAC_ADDRESS_LENGTH + PACKET_ID_LENGTH + 3  # MAC + Packet ID + Total Packets, Sequence, Payload Length
FOOTER_LENGTH = 2  # Checksum length
PACKET_DELIMITER = b'\x0D\x0A'  # Delimiter: 0D 0A

# ESP32 MAC address for identification
ESP32_MAC_ADDR = b'\x58\xBF\x25\x82\x8E\xD8'  # Replace with the actual MAC address

# Set to track received Packet IDs
PACKET_ID_TRACKER = deque(maxlen=50)  # FIFO queue with a max size of 50

# Dictionary to store segmented packets
pending_segments = {}


def calculate_checksum(data):
    """Calculate the checksum for the given data."""
    return sum(data) & 0xFFFF


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
    total_packets, sequence, payload_length = data[MAC_ADDRESS_LENGTH + PACKET_ID_LENGTH:HEADER_LENGTH]

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
                    print(f"Timeout for segmented data: Missing packets for key {key}. Proceeding with partial data.")

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
    if data_type == 0xFF:
        process_esp32_heartbeat(payload)
    elif data_type in [0x10, 0x11]:
        process_esp32_adc_data(payload)
    else:
        process_esp32_unknown_data_type(payload)
def process_esp32_heartbeat(payload):
	print("heartBeat from ESP32")
def process_esp32_adc_data(payload):
	print("ADC data")
def process_esp32_unknown_data_type(payload):
	print("unknown data type")
def process_data_other_node(payload):
    """
    Process data for other sensor nodes (different format).
    """
    print(f"Processed data from other node: {payload}")


def send_ack(ser, mac_addr, packet_id):
    """
    Send acknowledgment for a packet.
    """
    ack_message = mac_addr + b'ACK' + struct.pack('<H', packet_id)
    ser.write(ack_message)
    print(f"Sent ACK for packet ID {packet_id}: {ack_message}")


def print_packet_hex(data):
    print("Received Packet (Hex):", " ".join(f"{byte:02X}" for byte in data))


def main():
    buffer = b''  # Temporary buffer to store incoming data

    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            print(f"Listening on {SERIAL_PORT} at {BAUD_RATE} baud rate.")

            while True:
                # Read incoming data
                if ser.in_waiting > 0:
                    buffer += ser.read(ser.in_waiting)

                    # Split buffer into packets based on delimiter
                    while PACKET_DELIMITER in buffer:
                        # Split the buffer into one packet and the rest
                        packet, buffer = buffer.split(PACKET_DELIMITER, 1)

                        # Ensure the packet has at least 6 bytes for MAC address check
                        if len(packet) < 6:
                            print("Incomplete packet received. Skipping.")
                            continue

                        # Extract the MAC address
                        mac_addr = packet[:MAC_ADDRESS_LENGTH]

                        if mac_addr == ESP32_MAC_ADDR:
                            print(f"ESP32 packet detected. MAC: {mac_addr.hex().upper()}")

                            # Extract Packet ID
                            try:
                                packet_id = struct.unpack('<H', packet[MAC_ADDRESS_LENGTH:MAC_ADDRESS_LENGTH + PACKET_ID_LENGTH])[0]

                                # Check for duplicate Packet ID
                                if is_duplicate_packet(packet_id):
                                    print(f"Duplicate packet detected. Packet ID: {packet_id}")
                                    continue

                                # Validate the data
                                is_valid, result = validate_data(packet)

                                if not is_valid:
                                    print(f"Invalid ESP32 data: {result}")
                                    continue

                                # Process valid ESP32 packet
                                send_ack(ser, mac_addr, result["packet_id"])
                                process_data_esp32([result])

                            except struct.error as e:
                                print(f"Error processing ESP32 data: {e}")

                        else:
                            # Hand over non-ESP32 data
                            print(f"Non-ESP32 MAC detected: {mac_addr.hex().upper()}")
                            process_data_other_node(packet)

                # Small delay to prevent busy-waiting
                time.sleep(0.1)

    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("\nExiting program.")


if __name__ == "__main__":
    main()
