#include <esp_now.h>
#include <WiFi.h>

// Configuration
#define ONBOARD_LED 5
#define MAX_PACKET_SIZE 250
#define MAC_ADDR_SIZE 6

unsigned long lastSendTime = 0; // Store the last send time
const unsigned long sendInterval = 2000; // Interval between data sends in milliseconds


// Replace with the receiver's MAC address
uint8_t piMacAddr[] = {0x84, 0xCC, 0xA8, 0xA9, 0xE1, 0xE8};

// Data buffers for ADC readings
#define ADC_CHANNELS 2
#define SAMPLES_PER_SECOND 250
int16_t adcBuffer[ADC_CHANNELS][SAMPLES_PER_SECOND];

// Packet buffer
uint8_t packetBuffer[MAX_PACKET_SIZE];

// Retransmission queue
struct Packet {
    uint8_t data[MAX_PACKET_SIZE];
    size_t length;
    uint8_t retries;
};

QueueHandle_t packetQueue;

// ESP-NOW callbacks
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        Serial.println("Packet sent successfully.");
    } else {
        Serial.println("Packet failed. Retrying...");
    }
}

void onDataReceived(const uint8_t* mac, const uint8_t* incomingData, int len) {
    String command = String((char*)incomingData).substring(0, len);
    Serial.print("Command received: ");
    Serial.println(command);

    if (command == "sendstatus") {
        // Prepare status response
        String status = "ESP32 is ready!";
        esp_now_send(mac, (uint8_t*)status.c_str(), status.length());
       
    } else if (command == "reset") {
        ESP.restart();
     } else if (command == "LED_ON") {
        digitalWrite(ONBOARD_LED, HIGH);
     } else if (command == "LED_OFF") {
        digitalWrite(ONBOARD_LED, LOW);
        
    } else {
        Serial.println("Unknown command received.");
    }
}


// Function to calculate checksum
uint8_t calculateChecksum(uint8_t *data, size_t length) {
    uint8_t sum = 0;
    for (size_t i = 0; i < length; i++) {
        sum += data[i];
    }
    return sum;
}

// Function to send data packets
void sendDataTask(void *param) {
    Packet packet;

    while (true) {
        if (xQueueReceive(packetQueue, &packet, portMAX_DELAY) == pdTRUE) {
            esp_err_t result = esp_now_send(piMacAddr, packet.data, packet.length);

            if (result != ESP_OK) {
                Serial.println("Failed to send packet. Retrying...");
                packet.retries++;
                if (packet.retries < 3) {
                    xQueueSend(packetQueue, &packet, 0);
                } else {
                    Serial.println("Max retries reached. Dropping packet.");
                }
            }
        }
    }
}

// Function to prepare and queue data packets
void prepareData() {

    size_t packetCount = 0;
    size_t payloadSize = min((size_t)(MAX_PACKET_SIZE - 9), sizeof(adcBuffer) - packetCount);
    size_t totalPackets = (sizeof(adcBuffer) + MAX_PACKET_SIZE - 9 - 1) / (MAX_PACKET_SIZE - 9); // Calculate total packets needed


    for (size_t i = 0; i < totalPackets; i++) {
        memcpy(packetBuffer, WiFi.macAddress().c_str(), MAC_ADDR_SIZE);
        packetBuffer[6] = totalPackets;
        packetBuffer[7] = i;

        size_t payloadStart = 8;
        memcpy(&packetBuffer[payloadStart], &adcBuffer[0][packetCount], payloadSize);

        packetBuffer[payloadStart + payloadSize] = calculateChecksum(packetBuffer, payloadStart + payloadSize);

        Packet packet;
        memcpy(packet.data, packetBuffer, payloadStart + payloadSize + 1);
        packet.length = payloadStart + payloadSize + 1;
        packet.retries = 0;

        xQueueSend(packetQueue, &packet, 0);

        packetCount += payloadSize;
    }
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



void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);

 pinMode(ONBOARD_LED, OUTPUT);
    digitalWrite(ONBOARD_LED, LOW);
    
prepareESPNOW();

    

    packetQueue = xQueueCreate(10, sizeof(Packet));
    if (packetQueue == NULL) {
        Serial.println("Failed to create packet queue.");
        return;
    }

    xTaskCreate(sendDataTask, "SendDataTask", 4096, NULL, 1, NULL);

    Serial.println("ESP-NOW Initialized. Ready for bidirectional communication.");
}

void loop() {
    // Continuously sample ADC values
    for (size_t i = 0; i < SAMPLES_PER_SECOND; i++) {
        adcBuffer[0][i] = analogRead(34);
        adcBuffer[1][i] = analogRead(35);
    }

    // Check if it's time to send data (every 2 seconds)
    if (millis() - lastSendTime >= sendInterval) {
        prepareData();
        lastSendTime = millis(); // Update the last send time
    }

    // No delay, ADC samples continuously
}
