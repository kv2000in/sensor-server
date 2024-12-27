#include <espnow.h>
#include <ESP8266WiFi.h>

/*
*  Copyright (C) 2015 -2018  Espressif System
*

#ifndef __ESPNOW_H__
#define __ESPNOW_H__

enum esp_now_role {
ESP_NOW_ROLE_IDLE = 0,
ESP_NOW_ROLE_CONTROLLER,
ESP_NOW_ROLE_SLAVE,
ESP_NOW_ROLE_MAX,
};

typedef void (*esp_now_recv_cb_t)(u8 *mac_addr, u8 *data, u8 len);
typedef void (*esp_now_send_cb_t)(u8 *mac_addr, u8 status);

int esp_now_init(void);
int esp_now_deinit(void);

int esp_now_register_send_cb(esp_now_send_cb_t cb);
int esp_now_unregister_send_cb(void);

int esp_now_register_recv_cb(esp_now_recv_cb_t cb);
int esp_now_unregister_recv_cb(void);

int esp_now_send(u8 *da, u8 *data, int len);

int esp_now_add_peer(u8 *mac_addr, u8 role, u8 channel, u8 *key, u8 key_len);
int esp_now_del_peer(u8 *mac_addr);

int esp_now_set_self_role(u8 role);
int esp_now_get_self_role(void);

int esp_now_set_peer_role(u8 *mac_addr, u8 role);
int esp_now_get_peer_role(u8 *mac_addr);

int esp_now_set_peer_channel(u8 *mac_addr, u8 channel);
int esp_now_get_peer_channel(u8 *mac_addr);

int esp_now_set_peer_key(u8 *mac_addr, u8 *key, u8 key_len);
int esp_now_get_peer_key(u8 *mac_addr, u8 *key, u8 *key_len);

u8 *esp_now_fetch_peer(bool restart);

int esp_now_is_peer_exist(u8 *mac_addr);

int esp_now_get_cnt_info(u8 *all_cnt, u8 *encrypt_cnt);

int esp_now_set_kok(u8 *key, u8 len);

#endif


*/


// Structure for bidirectional data
typedef struct struct_message {
  int id;
  int adcValue;
  char command[32];
} struct_message;


 uint8_t esp32MacAddr[] = {0x58, 0xBF, 0x25, 0x82, 0x8E, 0xD8}; // Replace with ESP32 MAC address
struct_message receivedData, dataToSend;

// Callback for receiving ESP-NOW data
void onDataRecv(uint8_t *macAddr, uint8_t *data, uint8_t len) {
  memcpy(&receivedData, data, sizeof(receivedData));
  Serial.print("Received from ESP32: ADC Value = ");
  Serial.println(receivedData.adcValue);

  // Forward to Raspberry Pi
  Serial.print("Forwarding to Pi: ");
  Serial.println(receivedData.adcValue);
}


void prepareESPNOW() {


    // Initializing the ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Problem during ESP-NOW init");
    return;
  }
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  // Register the peer
  //Serial.println("Registering a peer");
  esp_now_add_peer(esp32MacAddr, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
 

  //if(DEBUG){ swSer.println("Registering send callback function");}
  esp_now_register_send_cb(onsentviaESPNOW);

}


// Function to send ESP-NOW data
void sendDataToESP32(const char *command) {
  strcpy(dataToSend.command, command);
  dataToSend.id = 2;
 
  esp_now_send(NULL, (uint8_t *)&dataToSend, sizeof(dataToSend));
}

void onsentviaESPNOW(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.println("Great Success with ESP Now ");
}



void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW

prepareESPNOW();

  // Register callback
  esp_now_register_recv_cb(onDataRecv);
}

void loop() {
  // Check for data from Raspberry Pi
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    sendDataToESP32(command.c_str());
    Serial.print("Forwarded Command to ESP32: ");
    Serial.println(command);
  }
}
