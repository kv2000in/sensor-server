import serial
import time
import struct

# Configuration
SERIAL_PORT = '/dev/serial0'  # Replace with your serial port
BAUD_RATE = 115200
MAC_ADDRESS_LENGTH = 6
PACKET_ID_LENGTH = 2
HEADER_LENGTH = MAC_ADDRESS_LENGTH + PACKET_ID_LENGTH + 3  # MAC + Packet ID + Total Packets, Sequence, Payload Length
FOOTER_LENGTH = 2  # Checksum length

# In-memory store for processed packet IDs to avoid duplicates
processed_packet_ids = set()

# ESP32 MAC address for identification
ESP32_MAC_ADDR = b'\x58\xBF\x25\x82\x8E\xD8'  # Replace with the actual MAC address

def calculate_checksum(data):
    """Calculate the checksum for the given data."""
    return sum(data) & 0xFFFF

def validate_data(data):
    """
    Validate the received data based on checksum and other criteria.
    """
    # Print raw packet in hex
    print_packet_hex(data)
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
    Process the data for ESP32 (specific format).
    Reconstructs the data from packets.
    """
    payload = b''.join(packet["payload"] for packet in packets)
    print(f"Processed data from ESP32: {payload}")

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
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            print(f"Listening on {SERIAL_PORT} at {BAUD_RATE} baud rate.")

            while True:
                # Read incoming data
                if ser.in_waiting > 0:
                    received_data = ser.read(ser.in_waiting)

                    # Ensure the chunk has at least 6 bytes for MAC address check
                    if len(received_data) < 6:
                        print("Incomplete data received (less than 6 bytes). Skipping.")
                        continue

                    # Extract the first six bytes as MAC address
                    mac_addr = received_data[:6]

                    if mac_addr == ESP32_MAC_ADDR:
                        print(f"ESP32 packet detected. MAC: {mac_addr.hex().upper()}")

                        try:
                            is_valid, result = validate_data(received_data)

                            if not is_valid:
                                print(f"Invalid ESP32 data: {result}")
                                continue
                            # Extract packet ID from result
                            packet_id = result["packet_id"]
                            # Process valid ESP32 packet
                            send_ack(ser, mac_addr, packet_id)
                            process_data_esp32([result])

                        except struct.error as e:
                            print(f"Error processing ESP32 data: {e}")

                    else:
                        # Hand over non-ESP32 data
                        print(f"Non-ESP32 MAC detected: {mac_addr.hex().upper()}")
                        process_data_other_node(received_data)

                # Small delay to prevent busy-waiting
                time.sleep(0.1)

    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("\nExiting program.")
if __name__ == "__main__":
    main()
