
#include <esp_now.h>
#include <WiFi.h>
#include <Ticker.h>


#define ONBOARD_LED 5
#define MAX_PACKET_SIZE 250
#define MAX_PAYLOAD_SIZE 237 //250 - 13  {header (6 MAC Address + 2 Packet ID + 1 Total Packets + 1 sequence + 1 payload length =11) + footer = Checksum 2 bytes)}
#define MAC_ADDR_SIZE 6
#define ADC_CHANNELS 2
#define SAMPLES_PER_SECOND 500
#define BUFFER_SIZE 1000 // Circular buffer size per channel
#define MAX_RETRIES 2
#define MAX_SEND_QUEUE_SIZE 20
#define MAX_RECV_QUEUE_SIZE 20
#define MAX_SENTPACKETS_QUEUE_SIZE 24
#define APPLICATION_LAYER_ACK_TIMEOUT 2000

bool DEBUG = true;

uint8_t piMacAddr[] = {0x84, 0xCC, 0xA8, 0xA9, 0xE1, 0xE8}; // Replace with receiver's MAC
int16_t circularBuffer[ADC_CHANNELS][BUFFER_SIZE];
volatile size_t bufferIndex = 0; // Circular buffer index
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 10000;
bool macLayerAckReceived = false;
bool onDataSentReturned = false;
uint8_t packetBuffer[MAX_PACKET_SIZE];

// Define a packet queue handle
QueueHandle_t packetQueue;
//Structure of packet to be sent.
struct Packet {
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

    uint8_t payload[33]; // Allocate enough space for dataType and RMS values
    payload[0] = 0xFF;   // Set the dataType as the first byte
    memcpy(&payload[1], rmsValues, sizeof(rmsValues)); // Copy RMS values after the dataType
    memset(&payload[1 + sizeof(rmsValues)], 0xAA, 24); // Add padding after RMS values

    uint16_t packetId = random(0, 65536); // Generate a random Packet ID
    queuePacket(payload, sizeof(payload), 1, 1, packetId); // Only pass the payload, as dataType is part of it
}

// Function to send data for a specific ADC channel (on demand)
void requestSendADCData(uint8_t channel) {
    // Send the data for the selected ADC channel (0 or 1)
    sendADCData(channel);
}

void sendADCData(uint8_t channel) {

    size_t totalPackets = (BUFFER_SIZE * sizeof(int16_t)) / (MAX_PAYLOAD_SIZE - 1); // -1 for dataType
    if ((BUFFER_SIZE * sizeof(int16_t)) % (MAX_PAYLOAD_SIZE - 1) != 0) {
        totalPackets++; // Round up if not evenly divisible
    }

    uint16_t initialPacketId = random(0, 65536);
    uint8_t payload[MAX_PAYLOAD_SIZE];

    for (uint8_t seq = 0; seq < totalPackets; seq++) {
        size_t startIdx = seq * (MAX_PAYLOAD_SIZE - 1) / sizeof(int16_t);
        size_t endIdx = min<size_t>((seq + 1) * (MAX_PAYLOAD_SIZE - 1) / sizeof(int16_t), BUFFER_SIZE);

        size_t payloadLength = 1; // Start with 1 for dataType
        payload[0] = (channel == 0) ? 0x10 : 0x11; // Set dataType as the first byte

        for (size_t i = startIdx; i < endIdx; i++) {
            int16_t value = circularBuffer[channel][i];
            memcpy(&payload[payloadLength], &value, sizeof(value));
            payloadLength += sizeof(value);
        }

        uint16_t packetId = initialPacketId + seq;
        queuePacket(payload, payloadLength, totalPackets, seq + 1, packetId); // Pass payload with dataType
    }
}

void queuePacket(uint8_t *payload, size_t payloadLength, uint8_t totalPackets, uint8_t sequence, uint16_t packetId) {
    Packet packet;
    memcpy(packet.payload, payload, payloadLength); // Copy the entire payload, including dataType
    packet.payloadLength = payloadLength;
    packet.totalPackets = totalPackets;
    packet.sequence = sequence;
    packet.packetId = packetId;
    packet.retryCount = 0;

    if (xQueueSend(packetQueue, &packet, 0) != pdPASS) {
        if (DEBUG) { Serial.println("Packet queue full! Dropping packet."); }
    } else {
        if (DEBUG) { Serial.println("Packet added to the queue"); }
    }
}



bool sendData( uint8_t *payload, size_t payloadLength, uint8_t totalPackets, uint8_t sequence, uint16_t packetId) {
    size_t headerSize = MAC_ADDR_SIZE + 5;  // Header size with packet ID
    size_t footerSize = 2;  // Footer size for checksum
    size_t packetLength = headerSize + payloadLength + footerSize;

  if (DEBUG) { Serial.print("Packet length = "); Serial.println(packetLength); }
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

    //printPacketHex(packetBuffer, packetLength);

    // Send the packet via ESP-NOW
    esp_err_t result = esp_now_send(piMacAddr, packetBuffer, packetLength);
    return result == ESP_OK;
}
void printPacketHex(const uint8_t *packet, size_t length) {
   if (DEBUG) { Serial.print("Packet (Hex): ");
    for (size_t i = 0; i < length; i++) {
        if (i > 0) Serial.print(" ");
     Serial.printf("%02X", packet[i]);
    }
    Serial.println();}
}

// Prepare ESP-NOW communication
void prepareESPNOW() {


    if (esp_now_init() != ESP_OK) {
        if (DEBUG) {Serial.println("ESP-NOW Initialization Failed!");}
        return;
    }

    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataReceived);

    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, piMacAddr, MAC_ADDR_SIZE);
    peerInfo.channel = 1;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        if (DEBUG) {Serial.println("Failed to add peer!");}
        return;
    }


}

// ESP-NOW callbacks
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
      macLayerAckReceived=true;
        if (DEBUG) {Serial.println("MAC Layer ACK recieved");}
    } else {
       if (DEBUG) { Serial.println("no MAC Layer ACK");}
        macLayerAckReceived=false;
    }
    onDataSentReturned=true;
}

void onDataReceived(const uint8_t *mac, const uint8_t *incomingData, int len) {
    ReceivedPacket packet;

    memcpy(packet.data, incomingData, len); // Copy the incoming data
    packet.length = len;

    // Post the packet to the queue
    if (xQueueSend(recvQueue, &packet, 0) != pdPASS) {
      if (DEBUG)  { Serial.println("Queue full! Dropping packet.");}
    }
}




void handleApplicationLayerAck(const uint8_t *data, int len) {
    if (DEBUG) {
        Serial.println("DATA in ACK Packet:");
        printPacketHex(data, len);
    }

    // Ensure we have 3 bytes: 0x41 + 2 bytes for packet ID
    if (len == 3 && data[0] == 0x41) {
        // Extract packet ID from the last two bytes
        uint16_t receivedPacketId = (data[2] << 8) | data[1];
        if (DEBUG) {
            Serial.print("Received ACK for packet ID: ");
            Serial.println(receivedPacketId);
        }

        // Find and remove the acknowledged packet in the sentPacketQueue
        SentPacket packet;
        bool found = false;

        for (int i = 0; i < 10; i++) { // Iterate queue up to its size
            if (xQueueReceive(sentPacketQueue, &packet, 0) == pdPASS) {
                if (packet.packetId == receivedPacketId) {
                    found = true;
                    if (DEBUG) { Serial.println("Packet acknowledged and removed from sentPacketQueue."); }
                } else {
                    xQueueSend(sentPacketQueue, &packet, 0);  // Re-add to queue if not matched
                }
            }
        }

        if (!found) {
            if (DEBUG) { Serial.println("ACK received but no matching packet found in the queue."); }
        }
    } else {
        if (DEBUG) { Serial.println("Invalid ACK packet or length."); }
    }
    if (DEBUG) { Serial.println(); }
}

void retryTask(void *param) {
    while (true) {
        vTaskDelay(APPLICATION_LAYER_ACK_TIMEOUT / portTICK_PERIOD_MS);

        // Temporary buffer for queue packets
        SentPacket tempBuffer[MAX_SENTPACKETS_QUEUE_SIZE];
        int packetCount = 0;

        // Drain the queue
        SentPacket packet;
        while (xQueueReceive(sentPacketQueue, &packet, 0) == pdPASS) {
            tempBuffer[packetCount++] = packet;
        }

        // Resend all packets in the buffer
        for (int i = 0; i < packetCount; i++) {
            if (tempBuffer[i].retryCount < MAX_RETRIES && !tempBuffer[i].acknowledged) {
                if (sendData(tempBuffer[i].packet.payload,
                             tempBuffer[i].packet.payloadLength, tempBuffer[i].packet.totalPackets,
                             tempBuffer[i].packet.sequence, tempBuffer[i].packetId)) {
                   if (DEBUG) {Serial.print("Resent packet ID: ");
                    Serial.println(tempBuffer[i].packetId);}
                    tempBuffer[i].retryCount++;
                } else {
                   if (DEBUG)  {Serial.print("Failed to resend packet ID: ");
                    Serial.println(tempBuffer[i].packetId);}
                    tempBuffer[i].retryCount++;
                }
            } else if (tempBuffer[i].retryCount >= MAX_RETRIES) {
               if (DEBUG) {Serial.print("Packet ID exceeded max retries and will be dropped: ");
                Serial.println(tempBuffer[i].packetId);}
            }
        }

        // Re-add unacknowledged packets to the queue
        for (int i = 0; i < packetCount; i++) {
            if (!tempBuffer[i].acknowledged && tempBuffer[i].retryCount < MAX_RETRIES) {
                xQueueSend(sentPacketQueue, &tempBuffer[i], portMAX_DELAY);
            }
        }
    }
}

void processReceivedDataTask(void *param) {
    ReceivedPacket packet;

    while (true) {
        // Wait for data in the queue
        if (xQueueReceive(recvQueue, &packet, portMAX_DELAY) == pdPASS) {
            if (packet.length < 1) {
                if (DEBUG) { Serial.println("Invalid packet length."); }
                continue;
            }

            uint8_t command = packet.data[0];
            if (DEBUG) {
                Serial.print("Processing command: 0x");
                Serial.println(command, HEX);
            }

            // Process based on command type
            switch (command) {
                case 0x40: // Reset
                    if (DEBUG) { Serial.println("Command: Reset"); }
                    ESP.restart();
                    break;

                case 0x42: // LED_ON
                    if (DEBUG) { Serial.println("Command: LED_ON"); }
                    digitalWrite(ONBOARD_LED, HIGH);
                    break;

                case 0x43: // LED_OFF
                    if (DEBUG) { Serial.println("Command: LED_OFF"); }
                    digitalWrite(ONBOARD_LED, LOW);
                    break;

                case 0x41: // ACK
                    if (packet.length == 3) {
                        handleApplicationLayerAck(packet.data, packet.length);
                    } else {
                        if (DEBUG) { Serial.println("Invalid ACK packet length."); }
                    }
                    break;

                default:
                    if (DEBUG) { Serial.println("Unknown command received."); }
                    break;
            }
        }
    }
}

void packetTransmitTask(void *param) {
    Packet currentPacket;

    while (true) {
        if (xQueueReceive(packetQueue, &currentPacket, portMAX_DELAY) == pdPASS) {
            uint16_t packetId = currentPacket.packetId;  // Use Packet ID from the Packet structure

            // Reset the call back function flag before sending
            onDataSentReturned = false;

            // Send the packet
            if (sendData(currentPacket.payload, currentPacket.payloadLength,
                         currentPacket.totalPackets, currentPacket.sequence, packetId)) {
                if (DEBUG) { Serial.println("Packet sent successfully. Waiting for esp_send_now_callback function to return..."); }

                // Wait for the MAC Layer ACK callback
                unsigned long startTime = millis();
                while (!onDataSentReturned) {
                    // Timeout mechanism to avoid infinite waiting
                    if (millis() - startTime > 500) { // Adjust the timeout as needed (e.g., 500ms)
                        if (DEBUG) { Serial.println("Timeout waiting for esp_send_now_callback function."); }
                        break;
                    }
                    vTaskDelay(10 / portTICK_PERIOD_MS); // Short delay to yield the CPU
                }

                // Add packet to sentPacketQueue if it was sent
                SentPacket sentPacket;
                sentPacket.packetId = packetId; // Retain the Packet ID
                sentPacket.packet = currentPacket;
                sentPacket.retryCount = 0;
                sentPacket.acknowledged = false;
                xQueueSend(sentPacketQueue, &sentPacket, portMAX_DELAY);

            } else {
                if (DEBUG) { Serial.println("Failed to send packet."); }
            }
        }
    }
}



void setup() {
    if (DEBUG) {Serial.begin(115200);}
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
    if (DEBUG) {Serial.println("System ready. DEBUG ON");}

    // Create packet send queue
    packetQueue = xQueueCreate(MAX_SEND_QUEUE_SIZE, sizeof(Packet));

    // Create task for packet transmission
    xTaskCreate(packetTransmitTask, "PacketTransmitTask", 4096, NULL, 1, NULL);

  // Initialize the queue with a size of 10 SentPacket items
    sentPacketQueue = xQueueCreate(MAX_SENTPACKETS_QUEUE_SIZE, sizeof(SentPacket));

   // Create task for retrying packet transmission
    xTaskCreate(retryTask, "retryTask", 12288, NULL, 1, NULL);
    

}


  void loop() {
    if (millis() - lastSendTime >= sendInterval) {
        //sendHeartbeat();
        sendADCData(1);
        lastSendTime = millis();
    }
}
