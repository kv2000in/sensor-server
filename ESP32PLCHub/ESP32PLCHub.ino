#include <esp_now.h>
#include <WiFi.h>






// Structure for sending and receiving data
typedef struct struct_message {
int id;
int adcValue;
char command[32];
#define ONBOARD_LED  5

} struct_message;
uint8_t peerMAC[] = {0x84, 0xCC, 0xA8, 0xA9, 0xE1, 0xE8}; // Replace with ESP-01 MAC address
struct_message dataToSend, receivedData;

// Callback for receiving data
void onDataRecv(const uint8_t *macAddr, const uint8_t *data, int len) {
memcpy(&receivedData, data, sizeof(receivedData));
Serial.print("Received Command: ");
Serial.println(receivedData.command);
digitalWrite(ONBOARD_LED,HIGH);
}


void prepareESPNOW() {


    // Initializing the ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Problem during ESP-NOW init");
    return;
  }
  // Define the peer information
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(esp_now_peer_info_t)); // Clear the structure
  memcpy(peerInfo.peer_addr, peerMAC, ESP_NOW_ETH_ALEN); // Set peer MAC address
  peerInfo.channel = 1; // Replace with the actual channel of the peer
  peerInfo.encrypt = false; // Set to true if encryption is used

  // Add the peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("Peer added successfully");

}

void onsentviaESPNOW(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.println("Great Success with ESP Now ");
}

void sendDatatoPi()
{
  // Read ADC value
int adcValue = analogRead(34); // Replace with your ADC pin
dataToSend.id = 1;
dataToSend.adcValue = adcValue;

// Send data

esp_now_send(peerMAC, (uint8_t *)&dataToSend, sizeof(dataToSend));

Serial.print("Sent ADC Value: ");
Serial.println(adcValue);
digitalWrite(ONBOARD_LED,LOW);
  }


void setup() {
  pinMode(ONBOARD_LED,OUTPUT);
Serial.begin(115200);
WiFi.mode(WIFI_STA); // Set to Station mode

prepareESPNOW();


// Register receive callback
esp_now_register_recv_cb(onDataRecv);
}

void loop() {
sendDatatoPi();
delay(1000); // Adjust as needed

}
