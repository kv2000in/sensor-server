#include <espnow.h>
#include <ESP8266WiFi.h>

// Maximum ESP-NOW payload size (250 bytes for ESP8266)
#define ESPNOW_MAX_PAYLOAD 250

// Callback for receiving ESP-NOW data
void onDataRecv(uint8_t *macAddr, uint8_t *data, uint8_t len) {
  os_printf("Received data from MAC: ");
  for (int i = 0; i < 6; i++) {
    os_printf("%02X", macAddr[i]);
    if (i < 5) os_printf(":");
  }
  os_printf("\n");

  // Forward data to Raspberry Pi via Serial
  Serial.write(data, len); // Send raw binary data to the Pi
  Serial.println(); // Add a newline for easier parsing on Pi side
}

// Function to dynamically add peer if not already added
void addPeerIfNeeded(uint8_t *macAddr) {
  if (!esp_now_is_peer_exist(macAddr)) {
    if (esp_now_add_peer(macAddr, ESP_NOW_ROLE_SLAVE, 1, NULL, 0) == 0) {
      os_printf("Added new peer: ");
      for (int i = 0; i < 6; i++) {
        os_printf("%02X", macAddr[i]);
        if (i < 5) os_printf(":");
      }
      os_printf("\n");
    } else {
      os_printf("Failed to add peer!\n");
    }
  }
}

// Function to send data from Raspberry Pi to the intended MAC address
void sendDataToMAC(uint8_t *macAddr, uint8_t *data, size_t len) {
  addPeerIfNeeded(macAddr); // Ensure the MAC address is added as a peer
  if (esp_now_send(macAddr, data, len) == 0) {
    os_printf("Data sent successfully!\n");
  } else {
    os_printf("Failed to send data!\n");
  }
}

// Callback for ESP-NOW send completion
void onDataSent(uint8_t *macAddr, uint8_t sendStatus) {
  os_printf("Send status for MAC: %s\n", sendStatus == 0 ? "Success" : "Failed");
}

void prepareESPNOW() {
  // Initialize ESP-NOW
  if (esp_now_init() != 0) {
    os_printf("Error initializing ESP-NOW\n");
    return;
  }
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO); // Allow both send and receive
  esp_now_register_recv_cb(onDataRecv);     // Register receive callback
  esp_now_register_send_cb(onDataSent);     // Register send callback
}

void setup() {
  Serial.begin(115200);   // Serial communication for Pi
  WiFi.mode(WIFI_STA);    // Set ESP8266 to Station mode
  prepareESPNOW();        // Initialize ESP-NOW
}

void loop() {
  // Check for data from Raspberry Pi
  if (Serial.available() > 0) {
    uint8_t buffer[ESPNOW_MAX_PAYLOAD];
    size_t len = Serial.readBytes(buffer, ESPNOW_MAX_PAYLOAD);

    // Extract MAC address (first 6 bytes)
    if (len > 6) {
      uint8_t macAddr[6];
      memcpy(macAddr, buffer, 6);
      sendDataToMAC(macAddr, buffer + 6, len - 6); // Forward data to the specified MAC
    }
  }
}
