
#include <esp_now.h>
#include <WiFi.h>
#include "esp_log.h"



#define ONBOARD_LED 5
#define MAX_PACKET_SIZE 250
#define MAX_PAYLOAD_SIZE 237 //250 - 13  {header (6 MAC Address + 2 Packet ID + 1 Total Packets + 1 sequence + 1 payload length =11) + footer = Checksum 2 bytes)}
#define MAC_ADDR_SIZE 6
#define ADC_CHANNELS 2
#define SAMPLES_PER_SECOND 500
#define BUFFER_SIZE 50 // Circular buffer size per channel 50 samples of 2 bytes each = 100 bytes per channel = 200 bytes for 2 channels
#define MAX_RETRIES 1
#define MAX_SEND_QUEUE_SIZE 20
#define MAX_RECV_QUEUE_SIZE 20
#define MAX_SENTPACKETS_QUEUE_SIZE 24
#define APPLICATION_LAYER_ACK_TIMEOUT 2000


static const char *TAG = "APP";


uint8_t piMacAddr[] = {0x84, 0xCC, 0xA8, 0xA9, 0xE1, 0xE8}; // Replace with receiver's MAC [b8:27:eb:f9:9f:40 for pi Zero W DUT]

//uint8_t piMacAddr[] = {0xB8, 0x27, 0xEB, 0xF9, 0x9F, 0x40}; // Replace with receiver's MAC [b8:27:eb:f9:9f:40 for pi Zero W DUT]

//uint8_t piMacAddr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Better to send it to broadcast - that way - this sketch becomes independent of the receiver.
int16_t circularBuffer[ADC_CHANNELS][BUFFER_SIZE];

volatile size_t bufferIndex = 0; // Circular buffer index
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 2000;
bool macLayerAckReceived = false;
bool onDataSentReturned = false;
uint8_t packetBuffer[MAX_PACKET_SIZE];

int16_t testBuffer[ADC_CHANNELS][BUFFER_SIZE] = {
    { // Buffer 0
        2047, 2670, 3250, 3754, 4157, 4435, 4571, 4559, 4398, 4099, 3674, 3137, 2507, 1807, 1057, 286,
        0, 267, 1040, 1812, 2502, 3132, 3668, 4094, 4395, 4558, 4570, 4433, 4155, 3751, 3246, 2666,
        2043, 1419, 837, 327, 0, 261, 1035, 1807, 2497, 3128, 3664, 4090, 4391, 4555, 4567, 4430,
        4153, 3749
    },
    { // Buffer 1
        2893, 3423, 3876, 4230, 4452, 4526, 4442, 4212, 3845, 3359, 2775, 2118, 1414, 688, 69, 0,
        264, 1039, 1806, 2491, 3126, 3660, 4089, 4390, 4553, 4567, 4431, 4153, 3751, 3249, 2668,
        2046, 1423, 839, 328, 0, 260, 1035, 1807, 2497, 3128, 3664, 4090, 4391, 4555, 4567, 4430,
        4153, 3749
    }
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




// Task to sample ADC channels at 1000 Hz for 100 ms
void sampleADC(void *param) {
    const uint32_t samplingInterval = 1000; // 1000 Hz = 1 ms sampling interval
    uint32_t lastSampleTime = micros();
    uint32_t endTime = micros() + 50000; // Sample for 50 ms

    while (micros() < endTime) {
        uint32_t currentTime = micros();

        // Check if it's time to sample
        if (currentTime - lastSampleTime >= samplingInterval) {
            lastSampleTime += samplingInterval; // Update the last sample time

            // Read ADC channels and store in circular buffer
            circularBuffer[0][bufferIndex] = analogRead(34); // Channel 0 (ADC)
            circularBuffer[1][bufferIndex] = analogRead(35); // Channel 1 (ADC)

            // Update circular buffer index
            bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
        }

        // Yield to allow other tasks to run
        vTaskDelay(1 / portTICK_PERIOD_MS); // Yield 1 ms to FreeRTOS scheduler
    }

 // Delete the task if it's no longer needed (or loop if you need it to repeat)
    vTaskDelete(NULL);  // Deletes this task and prevents it from returning
}





// Function to send heartbeat with ADC data (without RMS calculation)
void sendHeartbeat() {
  // Start ADC sampling task on core 1 with high priority
   //xTaskCreatePinnedToCore(sampleADC, "SampleADC", 2048, NULL, 1, NULL, 1); // High-priority task on core 1

  xTaskCreate(sampleADC, "SampleADC", 2048, NULL, 1, NULL); // High-priority task on core 1
   
    uint8_t payload[MAX_PAYLOAD_SIZE]; // Allocate enough space for dataType and ADC channel data
    payload[0] = 0xFF;   // Set the dataType as the first byte

    //50 samples of 2 bytes each per channel
    for (int ch = 0; ch < ADC_CHANNELS; ch++) {
    memcpy(&payload[1 + ch * BUFFER_SIZE * sizeof(uint16_t)], &circularBuffer[ch], BUFFER_SIZE * sizeof(uint16_t)); // Copy full 100 bytes
    }

    // Add padding after ADC data (using 0xAA for padding)
memset(&payload[1 + ADC_CHANNELS * BUFFER_SIZE * sizeof(uint16_t)], 0xAA, MAX_PAYLOAD_SIZE - (1 + ADC_CHANNELS * BUFFER_SIZE * sizeof(uint16_t)));

    // Create packet ID and other header values
    uint16_t packetId = random(0, 65536); // Generate a random Packet ID


    // Queue the packet (sending mechanism)
   // queuePacket(payload, sizeof(payload), 1, 1, packetId); // Send the payload with the header and data

        if (sendData(payload, sizeof(payload),
                         1, 1, packetId)) {
                ESP_LOGD(TAG"Packet sent successfully. Waiting for esp_send_now_callback function to return..."); }
//    ESP_LOGE(TAG, "This is an error message!");
//ESP_LOGW(TAG, "This is a warning message.");
//ESP_LOGI(TAG, "This is an informational message.");
//ESP_LOGD(TAG, "This is a debug message.");
//ESP_LOGV(TAG, "This is a verbose message.");
}











bool sendData( uint8_t *payload, size_t payloadLength, uint8_t totalPackets, uint8_t sequence, uint16_t packetId) {
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

    //printPacketHex(packetBuffer, packetLength);

    // Send the packet via ESP-NOW
    esp_err_t result = esp_now_send(piMacAddr, packetBuffer, packetLength);
    return result == ESP_OK;
}
void printPacketHex(const uint8_t *packet, size_t length) {
   ESP_LOGD(TAG,"Packet (Hex): ");
    for (size_t i = 0; i < length; i++) {
        if (i > 0) ESP_LOGD(TAG," ");
      ESP_LOGD(TAG,"%02X", packet[i]);
    }
    
}

// Prepare ESP-NOW communication
void prepareESPNOW() {


    if (esp_now_init() != ESP_OK) {
         ESP_LOGD(TAG,"ESP-NOW Initialization Failed!");
        return;
    }

    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataReceived);

    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, piMacAddr, MAC_ADDR_SIZE);
    peerInfo.channel = 1;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
         ESP_LOGD(TAG,"Failed to add peer!");
        return;
    }


}

// ESP-NOW callbacks
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
      macLayerAckReceived=true;
         ESP_LOGI(TAG,"MAC Layer ACK recieved");
    } else {
        ESP_LOGI(TAG,"no MAC Layer ACK");
        macLayerAckReceived=false;
    }
    
}

void onDataReceived(const uint8_t *mac, const uint8_t *incomingData, int len) {
    ReceivedPacket packet;

    memcpy(packet.data, incomingData, len); // Copy the incoming data
    packet.length = len;

    // Post the packet to the queue
    if (xQueueSend(recvQueue, &packet, 0) != pdPASS) {
       ESP_LOGE(TAG,"Queue full! Dropping packet.");
    }
}

//void onDataReceived(const uint8_t *mac, const uint8_t *incomingData, int len) {
//    
//uint8_t command = incomingData[0];
//            // Process based on command type
//            switch (command) {
//                case 0x40: // Reset
//                   ESP_LOGD(TAG,"Command: Reset");
//                    ESP.restart();
//                    break;
//
//                case 0x42: // LED_ON
//                    ESP_LOGD(TAG,"Command: LED_ON");
//                    digitalWrite(ONBOARD_LED, HIGH);
//                    break;
//
//                case 0x43: // LED_OFF
//                    ESP_LOGD(TAG,"Command: LED_OFF"); 
//                    digitalWrite(ONBOARD_LED, LOW);
//                    break;
//
//                case 0x41: // ACK
//                    
//                       ESP_LOGD(TAG,"ACK Application Layer Recvd."); 
//                  
//                    
//                    break;
//
//                default:
//                    ESP_LOGD(TAG,"Unknown command received.");
//                    break;
//            }
//    
//}



void processReceivedDataTask(void *param) {
    ReceivedPacket packet;
    
    while (true) {
        // Wait for data in the queue
        if (xQueueReceive(recvQueue, &packet, portMAX_DELAY) == pdPASS) {
            if (packet.length < 1) {
                ESP_LOGD(TAG,"Invalid packet length.");
                continue;
            }

            uint8_t command = packet.data[0];
            
                ESP_LOGD(TAG,"Processing command: 0x");
               
           

            // Process based on command type
            switch (command) {
                case 0x40: // Reset
                   ESP_LOGD(TAG,"Command: Reset");
                    ESP.restart();
                    break;

                case 0x42: // LED_ON
                    ESP_LOGD(TAG,"Command: LED_ON");
                    digitalWrite(ONBOARD_LED, HIGH);
                    break;

                case 0x43: // LED_OFF
                    ESP_LOGD(TAG,"Command: LED_OFF"); 
                    digitalWrite(ONBOARD_LED, LOW);
                    break;

                case 0x41: // ACK
                    
                       ESP_LOGD(TAG,"ACK Application Layer Recvd."); 
                  
                    
                    break;

                default:
                    ESP_LOGD(TAG,"Unknown command received.");
                    break;
            }
        }
    }
}




void setup() {

    WiFi.mode(WIFI_STA);
    prepareESPNOW();

    pinMode(ONBOARD_LED, OUTPUT);
    digitalWrite(ONBOARD_LED, LOW);

    // Create packet recv queue 
 recvQueue = xQueueCreate(MAX_RECV_QUEUE_SIZE, sizeof(ReceivedPacket));

    // Create a task to process the queue
  xTaskCreate(processReceivedDataTask, "ProcessReceivedData", 4096, NULL, 1, NULL);

    

}


  void loop() {
    if (millis() - lastSendTime >= sendInterval) {
        sendHeartbeat();
        //sendADCData(1);
        lastSendTime = millis();
    }
}
