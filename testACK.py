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
		
		incomplete_packets = {}
			
			# Buffer to accumulate incoming data
			buffer = b""
				
				while True:
					# Read incoming data
					if ser.in_waiting > 0:
						received_data = ser.read(ser.in_waiting)
						print(f"Received: {received_data}")
						
						# Add received data to buffer
						buffer += received_data
							
							# Process data in buffer if complete packet is received
							while len(buffer) >= HEADER_LENGTH:  # At least the header is available
								# Check for full packet length based on the header and payload length
								mac_addr = buffer[:MAC_ADDRESS_LENGTH]
								packet_id = struct.unpack('<H', buffer[MAC_ADDRESS_LENGTH:MAC_ADDRESS_LENGTH + PACKET_ID_LENGTH])[0]
								total_packets, sequence, payload_length = buffer[MAC_ADDRESS_LENGTH + PACKET_ID_LENGTH:HEADER_LENGTH]
								expected_packet_length = HEADER_LENGTH + payload_length + FOOTER_LENGTH
									
									if len(buffer) >= expected_packet_length:
										# Full packet received, process it
										packet = buffer[:expected_packet_length]
										
										# Validate the received data
										is_valid, result = validate_data(packet)
										if not is_valid:
											print(f"Invalid data: {result}")
											buffer = buffer[expected_packet_length:]  # Discard invalid data
											continue
												
												# Extract details
												mac_addr = result["mac_addr"]
												packet_id = result["packet_id"]
												total_packets = result["total_packets"]
												sequence = result["sequence"]
												payload_length = result["payload_length"]
												payload = result["payload"]
												
												# Check for duplicate packets
												if packet_id in processed_packet_ids:
													print(f"Duplicate packet ID {packet_id} ignored.")
													buffer = buffer[expected_packet_length:]  # Discard duplicate packet
													continue
														
														# Add packet to processed list
														processed_packet_ids.add(packet_id)
														
														# Send ACK for the packet
														send_ack(ser, mac_addr, packet_id)
														
														# Handle multi-packet data
														if total_packets > 1:
															if packet_id not in incomplete_packets:
																incomplete_packets[packet_id] = {}
																	
																	# Store the packet by its sequence number
																	incomplete_packets[packet_id][sequence] = result
																	
																	# If all packets are received
																	if len(incomplete_packets[packet_id]) == total_packets:
																		# Sort packets by sequence to ensure correct order
																		sorted_packets = [incomplete_packets[packet_id][seq] for seq in range(total_packets)]
																		# Process the data for ESP32 or another node
																		if mac_addr == ESP32_MAC_ADDR:
																			process_data_esp32(sorted_packets)
																				else:
																					for packet in sorted_packets:
																						process_data_other_node(packet["payload"])
																							
																							# Remove the packet group from incomplete_packets
																							del incomplete_packets[packet_id]
																								else:
																									# Process single-packet data immediately
																									if mac_addr == ESP32_MAC_ADDR:
																										process_data_esp32([result])
																											else:
																												process_data_other_node(payload)
																													
																													# Remove processed data from buffer
																													buffer = buffer[expected_packet_length:]
																														
																														else:
																															# If the full packet is not received, break the loop and wait for more data
																															break
																																
																																# Small delay to prevent busy-waiting
																																time.sleep(0.1)
																																	
																																	except serial.SerialException as e:
																																		print(f"Serial error: {e}")
																																			except KeyboardInterrupt:
																																				print("\nExiting program.")

if __name__ == "__main__":
	main()
