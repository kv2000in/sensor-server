/*
Working ESP-NOW to WiFi /TCP bridge.
*/
#define STASSID "*"
#define STAPSK  "*"
#include <ESP8266WiFi.h>
#include <espnow.h>
char mydata[64];
int maxmillistotryfortcp = 4000;
bool dataavailable = false;
WiFiClient client;
const char* host = "192.168.1.152";  // TCP Server IP
const int   port = 9999;            // TCP Server Port
void onDataReceiver(uint8_t * mac, uint8_t *incomingData, uint8_t len) {

Serial.println("Data Received");
 memcpy(mydata,incomingData,64);
dataavailable = true;

}
void sendDataviaTCP(char * datatobesent){
 unsigned long tcpconnectionstartmillis = millis();
while ((!client.connect(host, port))&&((millis()-tcpconnectionstartmillis) < maxmillistotryfortcp)) {
delayMicroseconds(100000);
}
if (client.connected()){


client.print(datatobesent);
client.stop();
 
}
else {
  
  Serial.println("Unable to connect to TCP");

}
  }
void setup() {
Serial.begin(115200);
Serial.printf("ESP-NOW to WiFi Client");
Serial.printf("Heap on start: %d\n", ESP.getFreeHeap());
  // Initializing the ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Problem during ESP-NOW init");
    return;
  }
  
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  // We can register the receiver callback function
  esp_now_register_recv_cb(onDataReceiver);

WiFi.mode(WIFI_STA);
WiFi.begin(STASSID, STAPSK);
while (WiFi.status() != WL_CONNECTED) {
Serial.print('.');
delay(500);
}

Serial.printf("\nSTA: %s (dns: %s / %s)\n",
WiFi.localIP().toString().c_str(),
WiFi.dnsIP(0).toString().c_str(),
WiFi.dnsIP(1).toString().c_str());

WiFi.softAP(STASSID "extender", STAPSK);
Serial.printf("AP: %s\n", WiFi.softAPIP().toString().c_str());
}
void loop() {
  if(dataavailable){
    sendDataviaTCP(mydata);
    dataavailable=false;
    }
}
