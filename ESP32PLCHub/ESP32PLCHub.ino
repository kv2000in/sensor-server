#include <esp_now.h>
#include <WiFi.h>

#define ONBOARD_LED 5
#define MAX_PACKET_SIZE 250
#define MAC_ADDR_SIZE 6
#define ADC_CHANNELS 2
#define SAMPLES_PER_SECOND 500
#define BUFFER_SIZE 1000 // Circular buffer size per channel
#define MAX_RETRIES 3
#define MAX_SEND_QUEUE_SIZE 30
#define MAX_RECV_QUEUE_SIZE 10

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
    uint8_t retryCount;
};



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

    uint8_t payload[32];
    payload[0] = 0xFF; // Data type: Heartbeat
    memcpy(&payload[1], rmsValues, sizeof(rmsValues));
    memset(&payload[1 + sizeof(rmsValues)], 0xAA, 24); // Padding

    // Enqueue the packet
    queuePacket(0xFF, payload, sizeof(payload), 1, 1);
    
    
}

// Send data function
bool sendData(uint8_t dataType, uint8_t *payload, size_t payloadLength, uint8_t totalPackets, uint8_t sequence) {
    
    size_t headerSize = MAC_ADDR_SIZE + 3; // Header size
    size_t footerSize = 2; // Footer size
    size_t packetLength = headerSize + payloadLength + footerSize;

    if (packetLength > MAX_PACKET_SIZE) return false;

    // Fill header
    memcpy(packetBuffer, WiFi.macAddress().c_str(), MAC_ADDR_SIZE);
    packetBuffer[6] = totalPackets;
    packetBuffer[7] = sequence;
    packetBuffer[8] = payloadLength;

    // Fill payload
    memcpy(&packetBuffer[9], payload, payloadLength);

    // Calculate and append checksum
    uint16_t checksum = calculateChecksum(packetBuffer, headerSize + payloadLength);
    packetBuffer[headerSize + payloadLength] = checksum & 0xFF;
    packetBuffer[headerSize + payloadLength + 1] = (checksum >> 8) & 0xFF;

    //reset maclayer and application layer
    macLayerAckReceived=false;
    applicationLayerAckReceived=false;
    
    // Send data
    esp_err_t result = esp_now_send(piMacAddr, packetBuffer, packetLength);
    return result == ESP_OK; 
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
            } else if (command == "ACK") {
              applicationLayerAckReceived = true;
            
            } else if (command == "NoACK") {
              applicationLayerAckReceived = false;
              
            } else if (command.startsWith("ACK")) {
                handleApplicationLayerAck(packet.data, packet.length);
            } else {
                Serial.println("Unknown command received.");
            }
        }
    }
}

void handleApplicationLayerAck(const uint8_t *data, int len) {
  
  }


void packetTransmitTask(void *param) {
    Packet currentPacket;

    while (true) {
        // Wait for a packet in the queue
        if (xQueueReceive(packetQueue, &currentPacket, portMAX_DELAY) == pdPASS) {
            currentPacket.retryCount = 0;

            // Retry logic
            while (!applicationLayerAckReceived && currentPacket.retryCount < MAX_RETRIES) {
                // Attempt to send the packet
                if (sendData(currentPacket.dataType, currentPacket.payload, currentPacket.payloadLength,
                             currentPacket.totalPackets, currentPacket.sequence)) {
                    Serial.println("Packet sent. Waiting for acknowledgment...");

                    // Wait for ACK with timeout
                    int timeout = 100;  // 1 second
                    while (timeout > 0 && !applicationLayerAckReceived) {
                        vTaskDelay(10 / portTICK_PERIOD_MS);
                        timeout -= 10;
                    }
                } else {
                    Serial.println("Failed to send packet.");
                }

                if (applicationLayerAckReceived) {
                    Serial.println("Packet acknowledged.");
                } else {
                    Serial.println("Acknowledgment not received. Retrying...");
                    currentPacket.retryCount++;
                }
            }

            if (!applicationLayerAckReceived) {
                Serial.println("Packet not acknowledged. Moving to next packet.");
            }
        }
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
    
}


  void loop() {
    if (millis() - lastSendTime >= sendInterval) {
        sendHeartbeat();
        lastSendTime = millis();
    }
}
