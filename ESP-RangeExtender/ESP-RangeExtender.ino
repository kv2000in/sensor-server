
/*
*Extend the range of wifi and also be on the lookout for ESP NOW packets. If received - send it to the TCP server.
*/

#if LWIP_FEATURES && !LWIP_IPV6

#define HAVE_NETDUMP 0

#ifndef STASSID
#define STASSID "*"
#define STAPSK  "*"
#endif

#include <ESP8266WiFi.h>
#include <lwip/napt.h>
#include <lwip/dns.h>
#include <dhcpserver.h>
#include <espnow.h>


#define NAPT 1000
#define NAPT_PORT 10

#if HAVE_NETDUMP

#include <NetDump.h>

void dump(int netif_idx, const char* data, size_t len, int out, int success) {
(void)success;
Serial.print(out ? F("out ") : F(" in "));
Serial.printf("%d ", netif_idx);

// optional filter example: if (netDump_is_ARP(data))
{
netDump(Serial, data, len);
//netDumpHex(Serial, data, len);
}
}
#endif
WiFiClient client;
const char* host = "192.168.1.152";  // TCP Server IP
const int   port = 9999;            // TCP Server Port
const int maxmillistotryfortcp = 4000;
char mydata[16];
bool dataavailable = false;

void onDataReceiver(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
 memcpy(mydata,incomingData,16);
 dataavailable = true;
}
void sendDataviaTCP(char * datatobesent){
 unsigned long tcpconnectionstartmillis = millis();
while ((!client.connect(host, port))&&((millis()-tcpconnectionstartmillis) < maxmillistotryfortcp)) {
delayMicroseconds(100000);
}
if (client.connected()){


//client.print(datatobesent); - Only sends partial data - stopping if it encounters \00 or \r or \n I guess
client.write(datatobesent,16); //Better to use - this. Verified with wireshark.
client.stop();
 
}
else {
  
  Serial.println("Unable to connect to TCP");

}
  }

void setup() {
Serial.begin(115200);
Serial.printf("\n\nNAPT Range extender\n");
Serial.printf("Heap on start: %d\n", ESP.getFreeHeap());
  // Initializing the ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Problem during ESP-NOW init");
    return;
  }
  
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  // We can register the receiver callback function
  esp_now_register_recv_cb(onDataReceiver);
  
#if HAVE_NETDUMP
phy_capture = dump;
#endif

// first, connect to STA so we can get a proper local DNS server
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

// give DNS servers to AP side
dhcps_set_dns(0, WiFi.dnsIP(0));
dhcps_set_dns(1, WiFi.dnsIP(1));

//  WiFi.softAPConfig(  // enable AP, with android-compatible google domain
//    IPAddress(172, 217, 28, 254),
//    IPAddress(172, 217, 28, 254),
//    IPAddress(255, 255, 255, 0));
WiFi.softAP(STASSID "extender", STAPSK);
Serial.printf("AP: %s\n", WiFi.softAPIP().toString().c_str());

Serial.printf("Heap before: %d\n", ESP.getFreeHeap());
err_t ret = ip_napt_init(NAPT, NAPT_PORT);
Serial.printf("ip_napt_init(%d,%d): ret=%d (OK=%d)\n", NAPT, NAPT_PORT, (int)ret, (int)ERR_OK);
if (ret == ERR_OK) {
ret = ip_napt_enable_no(SOFTAP_IF, 1);
Serial.printf("ip_napt_enable_no(SOFTAP_IF): ret=%d (OK=%d)\n", (int)ret, (int)ERR_OK);
if (ret == ERR_OK) {
Serial.printf("WiFi Network '%s' with same password is now NATed behind '%s'\n", STASSID "extender", STASSID);
}
}
Serial.printf("Heap after napt init: %d\n", ESP.getFreeHeap());
if (ret != ERR_OK) {
Serial.printf("NAPT initialization failed\n");
}
}

#else

void setup() {
Serial.begin(115200);
Serial.printf("\n\nNAPT not supported in this configuration\n");
}

#endif

void loop() {
    if(dataavailable){
    sendDataviaTCP(mydata);
    dataavailable=false;
    }
}
