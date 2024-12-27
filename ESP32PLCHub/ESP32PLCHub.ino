#include <esp_now.h>
#include <WiFi.h>

// Structure for sending and receiving data
typedef struct struct_message {
int id;
int adcValue;
char command[32];
#define ONBOARD_LED  5

} struct_message;

struct_message dataToSend, receivedData;

// Callback for receiving data
void onDataRecv(const uint8_t *macAddr, const uint8_t *data, int len) {
memcpy(&receivedData, data, sizeof(receivedData));
Serial.print("Received Command: ");
Serial.println(receivedData.command);
digitalWrite(ONBOARD_LED,HIGH);
}

void setup() {
  pinMode(ONBOARD_LED,OUTPUT);
Serial.begin(115200);
WiFi.mode(WIFI_STA); // Set to Station mode

// Initialize ESP-NOW
if (esp_now_init() != ESP_OK) {
Serial.println("Error initializing ESP-NOW");
return;
}

// Register receive callback
esp_now_register_recv_cb(onDataRecv);
}

void loop() {
// Read ADC value
int adcValue = analogRead(34); // Replace with your ADC pin
dataToSend.id = 1;
dataToSend.adcValue = adcValue;

// Send data
uint8_t receiverMacAddr[] = {0x84, 0xCC, 0xA8, 0xA9, 0xE1, 0xE8}; // Replace with ESP-01 MAC address
esp_now_send(receiverMacAddr, (uint8_t *)&dataToSend, sizeof(dataToSend));

Serial.print("Sent ADC Value: ");
Serial.println(adcValue);
digitalWrite(ONBOARD_LED,LOW);
delay(1000); // Adjust as needed

}
