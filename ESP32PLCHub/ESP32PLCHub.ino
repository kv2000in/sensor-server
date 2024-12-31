#include <esp_now.h>
#include <WiFi.h>
#include <Ticker.h>


#define ONBOARD_LED 5
#define MAX_PACKET_SIZE 250
#define MAC_ADDR_SIZE 6
#define ADC_CHANNELS 2
#define SAMPLES_PER_SECOND 500
#define BUFFER_SIZE 1000 // Circular buffer size per channel
#define MAX_RETRIES 3
#define MAX_SEND_QUEUE_SIZE 30
#define MAX_RECV_QUEUE_SIZE 10
#define APPLICATION_LAYER_ACK_TIMEOUT 1000


uint8_t piMacAddr[] = {0x84, 0xCC, 0xA8, 0xA9, 0xE1, 0xE8}; // Replace with receiver's MAC
int16_t circularBuffer[ADC_CHANNELS][BUFFER_SIZE];
volatile size_t bufferIndex = 0; // Circular buffer index
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 2000;
bool macLayerAckReceived = false;
bool applicationLayerAckReceived = false;
uint8_t packetBuffer[MAX_PACKET_SIZE];

// Define a packet queue handle
QueueHandle_t packetQueue;
//Structure of packet to be sent.
struct Packet {
    uint8_t dataType;
    uint8_t payload[MAX_PACKET_SIZE];
    size_t payloadLength;
    uint8_t totalPackets;
    uint8_t sequence;
    uint16_t packetId;
    uint8_t retryCount;
};

struct SentPacket {
    uint16_t packetId;    // Unique packet ID
    Packet packet;         // The packet data
    int retryCount;        // Retry count
    bool acknowledged;     // Whether the packet has been acknowledged
};

// Create a queue for storing sent packets
QueueHandle_t sentPacketQueue;

// Define a receive queue handle
QueueHandle_t recvQueue;

// Define a structure for received data
struct ReceivedPacket {
   
    uint8_t data[250];
    int length;

};

Ticker cleanupTicker; // Ticker object for periodic cleanup


void cleanupSentPackets() {
    // Process the queue and clean up acknowledged packets
    SentPacket packet;
    while (xQueueReceive(sentPacketQueue, &packet, 0) == pdTRUE) {
        if (packet.acknowledged) {
            // Do your clean up here
            Serial.println("Cleaned up acknowledged packet");
        } else {
            // Reinsert unacknowledged packet back into the queue if needed
            xQueueSend(sentPacketQueue, &packet, portMAX_DELAY);
        }
    }
}

// Calculate checksum
uint16_t calculateChecksum(uint8_t *data, size_t length) {
    uint16_t checksum = 0;
    for (size_t i = 0; i < length; i++) {
        checksum += data[i];
    }
    return checksum;
}




void sampleADC(void *param) {
    const uint32_t samplingInterval = 2000; // 500 Hz = 2000 microseconds
    uint32_t lastSampleTime = micros();

    while (true) {
        uint32_t currentTime = micros();

        // Check if it's time to sample
        if (currentTime - lastSampleTime >= samplingInterval) {
            lastSampleTime += samplingInterval; // Update the last sample time

            // Read ADC channels and store in circular buffer
            circularBuffer[0][bufferIndex] = analogRead(34);
            circularBuffer[1][bufferIndex] = analogRead(35);

            // Update circular buffer index
            bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
        }

        // Yield to allow other tasks to run
        vTaskDelay(1 / portTICK_PERIOD_MS); // Yield 1 ms to the FreeRTOS scheduler
    }
}

// Calculate RMS
void calculateRMS(uint16_t *rmsValues) {
    for (size_t ch = 0; ch < ADC_CHANNELS; ch++) {
        uint32_t sumOfSquares = 0;
        for (size_t i = 0; i < BUFFER_SIZE; i++) {
            sumOfSquares += circularBuffer[ch][i] * circularBuffer[ch][i];
        }
        rmsValues[ch] = sqrt(sumOfSquares / BUFFER_SIZE);
    }
}

void sendHeartbeat() {
    uint16_t rmsValues[ADC_CHANNELS];
    calculateRMS(rmsValues);

    uint8_t payload[32];
    payload[0] = 0xFF; // Data type: Heartbeat
    memcpy(&payload[1], rmsValues, sizeof(rmsValues));
    memset(&payload[1 + sizeof(rmsValues)], 0xAA, 24); // Padding

    // Enqueue the packet
    queuePacket(0xFF, payload, sizeof(payload), 1, 1);
    
    
}

// Function to send data for a specific ADC channel (on demand)
void requestSendADCData(uint8_t channel) {
    // Send the data for the selected ADC channel (0 or 1)
    sendADCData(channel);
}

// Function to send the entire buffer from either ADC channel 0 or 1
void sendADCData(uint8_t channel) {
    const size_t MAX_PAYLOAD_SIZE = MAX_PACKET_SIZE - 12; // Header + Footer size
    size_t totalPackets = (BUFFER_SIZE * sizeof(int16_t)) / MAX_PAYLOAD_SIZE;
    if ((BUFFER_SIZE * sizeof(int16_t)) % MAX_PAYLOAD_SIZE != 0) {
        totalPackets++; // Round up if not evenly divisible
    }

    // Buffer to hold data for each packet
    uint8_t payload[MAX_PAYLOAD_SIZE];

    // Determine which channel to send (channel 0 or channel 1)
    for (uint8_t seq = 0; seq < totalPackets; seq++) {
        size_t startIdx = seq * MAX_PAYLOAD_SIZE / sizeof(int16_t);
        size_t endIdx = min<size_t>((seq + 1) * MAX_PAYLOAD_SIZE / sizeof(int16_t), (size_t)BUFFER_SIZE);
        
        // Prepare the payload for this packet
        size_t payloadLength = 0;
        for (size_t i = startIdx; i < endIdx; i++) {
            int16_t value = circularBuffer[channel][i];
            memcpy(&payload[payloadLength], &value, sizeof(value));
            payloadLength += sizeof(value);
        }

        // Queue the packet with appropriate data type for the channel
        uint8_t dataType = (channel == 0) ? 10 : 11; // Channel 0 -> 10, Channel 1 -> 11
        queuePacket(dataType, payload, payloadLength, totalPackets, seq + 1);
    }
}

void queuePacket(uint8_t dataType, uint8_t *payload, size_t payloadLength, uint8_t totalPackets, uint8_t sequence) {
    Packet packet;
    packet.dataType = dataType;
    memcpy(packet.payload, payload, payloadLength);
    packet.payloadLength = payloadLength;
    packet.totalPackets = totalPackets;
    packet.sequence = sequence;
    packet.retryCount = 0;

    if (xQueueSend(packetQueue, &packet, 0) != pdPASS) {
        Serial.println("Packet queue full! Dropping packet.");
    }
}

bool sendData(uint8_t dataType, uint8_t *payload, size_t payloadLength, uint8_t totalPackets, uint8_t sequence, uint16_t packetId) {
    size_t headerSize = MAC_ADDR_SIZE + 5;  // Header size with packet ID
    size_t footerSize = 2;  // Footer size for checksum
    size_t packetLength = headerSize + payloadLength + footerSize;

    // Check packet size validity
    if (packetLength > MAX_PACKET_SIZE) return false;

    // Fill the header with MAC address
    uint8_t macAddr[MAC_ADDR_SIZE];
    WiFi.macAddress(macAddr);  // Fill the MAC address buffer
    memcpy(packetBuffer, macAddr, MAC_ADDR_SIZE);

    // Add packet ID (2 random bytes)
    packetBuffer[6] = packetId & 0xFF;     // Lower byte
    packetBuffer[7] = (packetId >> 8) & 0xFF;  // Upper byte

    // Add other information
    packetBuffer[8] = totalPackets;
    packetBuffer[9] = sequence;
    packetBuffer[10] = payloadLength;

    // Fill the payload
    memcpy(&packetBuffer[11], payload, payloadLength);

    // Calculate and append checksum
    uint16_t checksum = calculateChecksum(packetBuffer, headerSize + payloadLength);
    packetBuffer[headerSize + payloadLength] = checksum & 0xFF;        // Lower byte
    packetBuffer[headerSize + payloadLength + 1] = (checksum >> 8) & 0xFF;  // Upper byte

    // Reset ACK flags
    macLayerAckReceived = false;
    applicationLayerAckReceived = false;

    printPacketHex(packetBuffer, packetLength);

    // Send the packet via ESP-NOW
    esp_err_t result = esp_now_send(piMacAddr, packetBuffer, packetLength);
    return result == ESP_OK;
}
void printPacketHex(const uint8_t *packet, size_t length) {
    Serial.print("Packet (Hex): ");
    for (size_t i = 0; i < length; i++) {
        if (i > 0) Serial.print(" ");
        Serial.printf("%02X", packet[i]);
    }
    Serial.println();
}

// Prepare ESP-NOW communication
void prepareESPNOW() {


    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Initialization Failed!");
        return;
    }

    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataReceived);

    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, piMacAddr, MAC_ADDR_SIZE);
    peerInfo.channel = 1;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer!");
        return;
    }


}

// ESP-NOW callbacks
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
      macLayerAckReceived=true;
        Serial.println("MAC Layer ACK recieved");
    } else {
        Serial.println("no MAC Layer ACK");
        macLayerAckReceived=false;
    }
}

void onDataReceived(const uint8_t *mac, const uint8_t *incomingData, int len) {
    ReceivedPacket packet;

    memcpy(packet.data, incomingData, len); // Copy the incoming data
    packet.length = len;

    // Post the packet to the queue
    if (xQueueSend(recvQueue, &packet, 0) != pdPASS) {
        Serial.println("Queue full! Dropping packet.");
    }
}

void handleApplicationLayerAck(const uint8_t *data, int len) {
    Serial.println("DATA in ACK Packet");
    printPacketHex(data, len);
    
    // Ensure we have enough data (minimum 5 bytes for "ACK" + 2 bytes for packet ID)
    if (len > 4) {
        // Extract packet ID from the last two bytes (4th and 5th byte)
        uint16_t receivedPacketId = (data[3] << 8) | data[4];  // Packet ID is in positions 3 and 4
        Serial.print("Received packet ID: ");
        Serial.println(receivedPacketId);

        // Process ACK or NACK
        if (String((char*)data).startsWith("ACK")) {
            // It's an ACK, mark as acknowledged
            Serial.print("ACK received for packet ID: ");
            Serial.println(receivedPacketId);

            // Find the matching packet in the sentPacketQueue and mark as acknowledged
            SentPacket receivedPacket;
            bool found = false;
            for (int i = 0; i < 10; i++) {
                if (xQueueReceive(sentPacketQueue, &receivedPacket, 0) == pdPASS) {
                    if (receivedPacket.packetId == receivedPacketId && !receivedPacket.acknowledged) {
                        receivedPacket.acknowledged = true;  // Mark as acknowledged
                        Serial.println("Packet acknowledged and removed from retry list.");
                        found = true;
                        break;
                    } else {
                        xQueueSend(sentPacketQueue, &receivedPacket, 0);  // Re-add to queue if not matched
                    }
                }
            }

            if (!found) {
                Serial.println("ACK received but packet ID not found in the queue.");
            }

        } else if (String((char*)data).startsWith("NACK")) {
            // It's a NACK, reattempt sending the packet
            Serial.print("NACK received for packet ID: ");
            Serial.println(receivedPacketId);

            // Retry the NACKed packet
            SentPacket retryPacket;
            bool found = false;
            for (int i = 0; i < 10; i++) {
                if (xQueueReceive(sentPacketQueue, &retryPacket, 0) == pdPASS) {
                    if (retryPacket.packetId == receivedPacketId && !retryPacket.acknowledged) {
                        Serial.println("Retrying NACKed packet...");
                        // Reattempt sending the packet
                        sendData(retryPacket.packet.dataType, retryPacket.packet.payload,
                                 retryPacket.packet.payloadLength, retryPacket.packet.totalPackets,
                                 retryPacket.packet.sequence, receivedPacketId);
                        retryPacket.retryCount++;  // Increment retry count
                        found = true;
                        break;
                    } else {
                        xQueueSend(sentPacketQueue, &retryPacket, 0);  // Re-add to queue if not matched
                    }
                }
            }

            if (!found) {
                Serial.println("NACK received but packet ID not found in the queue.");
            }
        }
    } else {
        Serial.println("Invalid packet length or data too short for packet ID extraction.");
    }
    Serial.println();
}

void processReceivedDataTask(void *param) {
    ReceivedPacket packet;

    while (true) {
        // Wait for data in the queue
        if (xQueueReceive(recvQueue, &packet, portMAX_DELAY) == pdPASS) {
            // Extract command
            String command = String((char*)packet.data).substring(0, packet.length);
            Serial.print("Processing command: ");
            Serial.println(command);

            if (command == "sendstatus") {
                // Prepare status response
                Serial.println("Status = ready");
                
            } else if (command == "reset") {
                ESP.restart();
            } else if (command == "LED_ON") {
                digitalWrite(ONBOARD_LED, HIGH);
            } else if (command == "LED_OFF") {
                digitalWrite(ONBOARD_LED, LOW);
            } else if (command.startsWith("ACK") || command.startsWith("NACK")) {
                // Directly handle the ACK or NACK command
                handleApplicationLayerAck(packet.data, packet.length);
            } else {
                Serial.println("Unknown command received.");
            }
        }
    }

}


void packetTransmitTask(void *param) {
    Packet currentPacket;

    while (true) {
        if (xQueueReceive(packetQueue, &currentPacket, portMAX_DELAY) == pdPASS) {
            currentPacket.retryCount = 0;
            uint16_t packetId = random(0, 65536);  // Generate a random packet ID

            // Add packet to sentPacketQueue with an initial state
            SentPacket sentPacket;
            sentPacket.packetId = packetId;
            sentPacket.packet = currentPacket;
            sentPacket.retryCount = 0;
            sentPacket.acknowledged = false;
            xQueueSend(sentPacketQueue, &sentPacket, portMAX_DELAY);

            bool packetSent = false;

            // Retry logic
            while (!packetSent && currentPacket.retryCount < MAX_RETRIES) {
                if (sendData(currentPacket.dataType, currentPacket.payload, currentPacket.payloadLength,
                             currentPacket.totalPackets, currentPacket.sequence, packetId)) {
                    Serial.println("Packet sent. Waiting for acknowledgment...");

                    // Reset ACK flag before waiting
                    applicationLayerAckReceived = false;

                    // Wait for ACK with timeout
                    int timeout = APPLICATION_LAYER_ACK_TIMEOUT;  
                    while (timeout > 0 && !applicationLayerAckReceived) {
                        vTaskDelay(10 / portTICK_PERIOD_MS);
                        timeout -= 10;
                    }

                    if (applicationLayerAckReceived) {
                        Serial.println("Packet acknowledged.");
                        packetSent = true;  // Successfully acknowledged
                        // Mark the packet as acknowledged
                        SentPacket acknowledgedPacket;
                        bool found = false;

                        // Search for the acknowledged packet in the sentPacketQueue
                        for (int i = 0; i < 10; i++) {
                            if (xQueueReceive(sentPacketQueue, &acknowledgedPacket, 0) == pdPASS) {
                                if (acknowledgedPacket.packetId == packetId) {
                                    acknowledgedPacket.acknowledged = true;
                                    found = true;
                                    Serial.println("Packet acknowledged and removed from retry list.");
                                    break;
                                } else {
                                    // Re-add to queue if not matched
                                    xQueueSend(sentPacketQueue, &acknowledgedPacket, 0);
                                }
                            }
                        }

                        if (!found) {
                            Serial.println("Acknowledgment received but packet ID not found in the queue.");
                        }
                    } else {
                        Serial.println("Acknowledgment not received. Retrying...");
                        currentPacket.retryCount++;
                    }
                } else {
                    Serial.println("Failed to send packet.");
                    currentPacket.retryCount++;
                }
            }

            if (!packetSent) {
                Serial.println("Packet not acknowledged after retries. Moving to next packet.");
            }
        }
    }
}




void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    prepareESPNOW();

    pinMode(ONBOARD_LED, OUTPUT);
    digitalWrite(ONBOARD_LED, LOW);

    // Create packet recv queue 
    recvQueue = xQueueCreate(MAX_RECV_QUEUE_SIZE, sizeof(ReceivedPacket));

    // Create a task to process the queue
    xTaskCreate(processReceivedDataTask, "ProcessReceivedData", 4096, NULL, 1, NULL);

    //Task to sample ADC
    xTaskCreate(sampleADC, "SampleADC", 2048, NULL, 1, NULL);
    Serial.println("System ready.");

    // Create packet send queue
    packetQueue = xQueueCreate(MAX_SEND_QUEUE_SIZE, sizeof(Packet));

    // Create task for packet transmission
    xTaskCreate(packetTransmitTask, "PacketTransmitTask", 4096, NULL, 1, NULL);

  // Initialize the queue with a size of 10 SentPacket items
    sentPacketQueue = xQueueCreate(10, sizeof(SentPacket));
    
    //clean up sent packets every 20 seconds to free up memory
    cleanupTicker.attach(20, cleanupSentPackets);
}


  void loop() {
    if (millis() - lastSendTime >= sendInterval) {
        sendHeartbeat();
        lastSendTime = millis();
    }
}
