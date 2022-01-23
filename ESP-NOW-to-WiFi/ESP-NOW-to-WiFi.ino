
/*
*ESP-NOW to Wifi Bridge.
*/


#ifndef STASSID
#define STASSID "*"
#define STAPSK  "*"
#endif

#include <ESP8266WiFi.h>
#include <espnow.h>



WiFiClient client;
const char* host = "192.168.1.152";  // TCP Server IP
const int   port = 9999;            // TCP Server Port
char mydata[512]; //Create a 64 byte "ring buffer" to handle 16 byte data arriving from up to 4 nodes
bool dataavailable = false;
int mydatapointer=0; //increase the pointer by 16 everytime data arrives, send data via tcp and reset to zero.
void onDataReceiver(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
 memcpy(&mydata[mydatapointer],incomingData,16);
 mydatapointer=mydatapointer+16; //should check that this doesn't go higher than 48 (+16 =64)
 dataavailable = true;
}
void sendDataviaTCP(char * datatobesent){
 
if (client.connect(host,port)){
//client.print(datatobesent); - Only sends partial data - stopping if it encounters \00 or \r or \n I guess
client.write(datatobesent,mydatapointer); //Better to use - client.write. Verified with wireshark.
//Add data to the buffer and send it out all at once. 
client.stop();
mydatapointer=0;
dataavailable=false;
}
else {
    Serial.println("Unable to connect to TCP");
    //reset the buffer if unable to connect to TCP
    mydatapointer=0;
    dataavailable=false;  
}
  }

void setup() {
Serial.begin(115200);
  // Initializing the ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Problem during ESP-NOW init");
    return;
  }
  
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  // We can register the receiver callback function
  esp_now_register_recv_cb(onDataReceiver);
  

// first, connect to STA so we can get a proper local DNS server
WiFi.mode(WIFI_STA);
WiFi.begin(STASSID, STAPSK);
while (WiFi.status() != WL_CONNECTED) {
Serial.print('.');
delay(500);
}
Serial.printf("\nSTA: %s \n",
WiFi.localIP().toString().c_str());


}

void loop() {
    if(dataavailable){
    sendDataviaTCP(mydata);
    dataavailable=false;
    }
}
