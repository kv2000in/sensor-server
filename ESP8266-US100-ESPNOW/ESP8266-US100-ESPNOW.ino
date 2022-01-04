/*
Dec 2021 - 
todo:
0) Wake up- attempt to send via ESP-NOW, if success - sleep. If fail, connect via wifi.
1) ESPMultiwifi - add ability to look for extenders and connect - Will use a different approach instead of wifimulti. Connect to main one - if fails - scan and connect to strongest one 
Wifimulti tries to connect to the last connected one first anyway. 
1a) Add temperature recording to the server side
2) Reduce the total on time to reduce power usage.
3) Increase sleep duration to 1 min
4) Waterproof/weatherproof,
5) Implement MAC address based data and the server can assign where it is coming from.


*/
#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
#include <espnow.h>

// Mac addresses of peers
//uint8_t peer0[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};// Each specific peer gets 6-7 packets per send. Broadcast peer only gets one packet.
//uint8_t peer0[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};//86:CC:A8:AA:20:F9 softAP MAC of server Also - sending to broadcast always results in "success" even in the absence of ACK so need to specify peer MAC Address

uint8_t peer3[] = {0x86, 0xCC, 0xA8, 0xA9, 0xE1, 0xE8};//marked 3
uint8_t peer2[] = {0x86, 0xCC, 0xA8, 0xA9, 0x40, 0x38};//marked 2
uint8_t peer1[] = {0x86, 0xCC, 0xA8, 0xA8, 0xE7, 0xB6};//marked 1

bool DEBUG = true;
bool sendviaESPNOWsuccess;
int ESPNOWresendcounter;
int maxESPNOWresendattempts=5;
int maxmillistotryforwifi = 10000;
int maxmillistotryfortcp = 300;
char str[64];
uint8_t macAddr[6];

SoftwareSerial swSer;
WiFiClient client;


// WiFi credentials.
const char* WIFI_SSID0 = "*";
const char* WIFI_SSID1 = "*";
const char* WIFI_PASS = "*";

const char* host = "192.168.1.152";  // TCP Server IP
const int   port = 9999;            // TCP Server Port

//*************************************************
//Need to change Resistance values, static IP addresses, add wifi credentials, double check server IP address and port and First char ("1 or 2") when flashing it into different sensor nodes
//Sensor 1 R1=986k , R2 = 298k; Sensor 2 R1=974k, R2 = 296k Sensor 3 R1=1001k, R2 = 298K Sensor 4 R1=990k, R2 = 298k
//Sensor 1 static IP 192.168.1.201, sensor 2 static IP 192.168.1.202 snd so on
//For flashing - power the board with separate 5V power supply. Connect the gnd, rx, tx from USB serial to the programming board, flash mode is activated by holding GPIO 0 to ground at power on OR RESET (taking reset to GND)
//ESP201 boards are 1M flash, DIO, 40 MHz
//**************************************************

int batteryVoltage;   
int R1=990;
int R2=298;
#define TRIGGER_PIN  4  // Arduino pin tied to trigger pin on the ultrasonic sensor. //TX pin for swSerial
#define ECHO_PIN     5  // Arduino pin tied to echo pin on the ultrasonic sensor.//TX pin for swSerial
#define HCSR04SwitchPin 12
// the US-100 module has jumper cap on the back for Serial comm mode. No Jumper for direct - trig/echo mode
unsigned int HighLen = 0;
unsigned int LowLen  = 0;
unsigned int Len_mm  = 0;
int Temperature45 = 0;

void prepareESPNOW() {

WiFi.persistent( false );
    // Initializing the ESP-NOW
  if (esp_now_init() != 0) {
    if (DEBUG){swSer.println("Problem during ESP-NOW init");}
    return;
  }
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  // Register the peer
  //Serial.println("Registering a peer");
  esp_now_add_peer(peer1, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
  esp_now_add_peer(peer2, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
  esp_now_add_peer(peer3, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
  //if(DEBUG){ swSer.println("Registering send callback function");}
  esp_now_register_send_cb(onsentviaESPNOW);

}
void sendviaESPNOW(){

 esp_now_send(NULL, (uint8_t *) str, 64);
  ESPNOWresendcounter++;
}
void onsentviaESPNOW(uint8_t *mac_addr, uint8_t sendStatus) {

if (sendStatus==0)
{
sendviaESPNOWsuccess = true;
ESPNOWresendcounter=0;
}
else {
  if (ESPNOWresendcounter<maxESPNOWresendattempts){ sendviaESPNOW(); } 
  else {
    if (DEBUG){swSer.println("ESP-NOW send failed");}
    }
  }
  
}

void sendviaWIFI() {






WiFi.begin(WIFI_SSID0, WIFI_PASS);
WiFi.config(IPAddress(192,168,1,204), IPAddress(192,168,1,2), IPAddress(255,255,255,0),IPAddress(192,168,1,2));
unsigned long wifiConnect0Start = millis();

while ((WiFi.status() != WL_CONNECTED) && ((millis() - wifiConnect0Start < maxmillistotryforwifi))) {
// Check to see if
if (WiFi.status() == WL_CONNECT_FAILED) {
//Connection fails due to authentication problem etc - so just debug print and return.
if (DEBUG){swSer.println(" 0 WL_CONNECT_FAILED");}

}



// Only try for maxmillistotryforwifi.
if (millis() - wifiConnect0Start > maxmillistotryforwifi) {
if (DEBUG){swSer.println("Wifi0 connection attempt timed-out");}

}
delay(100);
}
if (WiFi.status() != WL_CONNECTED){
WiFi.begin(WIFI_SSID1, WIFI_PASS);
WiFi.config(IPAddress(192,168,4,204), IPAddress(192,168,4,1), IPAddress(255,255,255,0),IPAddress(192,168,4,1));
unsigned long wifiConnect1Start = millis();

while ((WiFi.status() != WL_CONNECTED) && ((millis() - wifiConnect1Start < maxmillistotryforwifi))) {
// Check to see if
if (WiFi.status() == WL_CONNECT_FAILED) {
//Connection fails due to authentication problem etc - so just debug print and return.
if (DEBUG){swSer.println(" 1 WL_CONNECT_FAILED");}

}



// Only try for maxmillistotryforwifi.
if (millis() - wifiConnect1Start > maxmillistotryforwifi) {
if (DEBUG){swSer.println("Wifi1 connection attempt timed-out");}

return; //Unable to connect to either SSIDs so exit.
}
delay(100);
}



}
if (DEBUG){swSer.println("Wifi connected");
swSer.println(WiFi.status());
}
unsigned long tcpconnectionstartmillis = millis();
while ((!client.connect(host, port))&&((millis()-tcpconnectionstartmillis) < maxmillistotryfortcp)) {
delay(100);
}
if (client.connected()){

client.print(str);
if (DEBUG){swSer.println("Sent via TCP client");}


delay(5);
//Close the socket - server is closing after one receive at the moment so it may not be necessary to close by the client
client.stop();


}
else
{
  if (DEBUG){swSer.println("TCP Connection Fail");}
return;
  }
}
void triggersensor(){
digitalWrite(TRIGGER_PIN, LOW);
pinMode(TRIGGER_PIN, OUTPUT);
delay(10);
digitalWrite(TRIGGER_PIN, HIGH);
delayMicroseconds(50);
digitalWrite(TRIGGER_PIN, LOW);
}

unsigned int get_distance_via_triggerecho()
{
long duration =0;
int maxcyclecount = 5; 
int cyclecount = 0;
while ((duration==0) && (cyclecount<maxcyclecount))

{
triggersensor();

// Read the signal from the sensor: a HIGH pulse whose
// duration is the time (in microseconds) from the sending
// of the ping to the reception of its echo off of an object.
pinMode(ECHO_PIN, INPUT);
long duration = pulseIn(ECHO_PIN, HIGH);

delay(10);
cyclecount++;
}
// convert the time into a distance
return (duration/2) / 29.1;


}





unsigned int get_distance_via_serial()
{
int cyclecount=0;
int maxcyclecount=5;

while ((Len_mm==0) && (cyclecount<maxcyclecount)) 
{
Serial.flush();                               // clear receive buffer of serial port
Serial.write(0X55);                           // trig US-100 begin to measure the distance
delay(500);                                   // delay 500ms to wait result
if(Serial.available() >= 2)                   // when receive 2 bytes 
{
HighLen = Serial.read();                   // High byte of distance
LowLen  = Serial.read();                   // Low byte of distance
Len_mm  = HighLen*256 + LowLen;            // Calculate the distance

}
cyclecount++;
delay(50);

}  

return Len_mm/10;

}

int get_temp_via_serial()
{
Serial.flush();       // clear receive buffer of serial port
Serial.write(0X50);   // trig US-100 begin to measure the temperature
delay(500);            //delay 500ms to wait result
if(Serial.available() >= 1)            //when receive 1 bytes 
{
Temperature45 = Serial.read();     //Get the received byte (temperature)
if((Temperature45 > 1) && (Temperature45 < 130))   //the valid range of received data is (1, 130)
{
Temperature45 -= 45;                           //Real temperature = Received_Data - 45

}
}

delay(10);                            //delay 500ms
return Temperature45;
}



void preparedata(){

WiFi.mode(WIFI_STA);
WiFi.macAddress(macAddr);

//ADC*(1.1/1024) will give the Vout at the voltage divider
//V=(Vout*((R1+R2)/R2))*1000 miliVolts
batteryVoltage = ((analogRead(A0)*(1.1/1024))*((R1+R2)/R2))*1000;

turnonsensormodule();
delay(10);

memcpy(str,macAddr,6);
str[6]='|';
//cm = ((sonar.ping_median(5))/2) / 29.1;
//convert int to ASCII and put it in the char array - adds '\0' at the end so string terminates after this - even if there is more stuff after this in the array
itoa( get_distance_via_serial(), str+7, 10 );
int alength = strlen(str);
str[alength]='|';
//Add the vcc value after ':'
itoa( batteryVoltage, str+alength+1, 10 ); // for some reason +1 outputs starnge 2:32:283:26 or  2:11305:275:26
int blength = strlen(str);
str[blength]='|';
//Add the temp value after ':'
itoa( get_temp_via_serial(), str+blength+1, 10 );

delay(10); 
turnoffsensormodule();

}
void turnonsensormodule(){
digitalWrite(HCSR04SwitchPin,HIGH);
}
void turnoffsensormodule(){
digitalWrite(HCSR04SwitchPin,LOW);
}
void setup() {
str[64]={};

  
Serial.begin(9600);
// connect RX (Pin 0 of Arduino digital IO) to Echo/Rx (US-100), TX (Pin 1 of Arduino digital IO) to Trig/Tx (US-100) 
//Baudrate 9600 for comm with US-100
delay(5);
Serial.swap(); //GPIO15 (TX) and GPIO13 (RX)
delay(5);
pinMode(HCSR04SwitchPin,OUTPUT);
pinMode(ECHO_PIN,INPUT);
pinMode(TRIGGER_PIN,OUTPUT); 
if (DEBUG) {swSer.begin(9600, SWSERIAL_8N1, ECHO_PIN, TRIGGER_PIN, false, 95, 11);}           


preparedata();

if (DEBUG){swSer.print("Data is: ");swSer.println(str);}
prepareESPNOW();
sendviaESPNOW();

delay(30);

if (!(sendviaESPNOWsuccess)){
  sendviaWIFI();
  if (DEBUG){swSer.println("Data is sent via WIFI");}
  }

ESP.deepSleep(45e6); // 20e6 is 20 microseconds RF_NO_CAL - no change in current
//ESP.deepSleep(2e6); // 20e6 is 20 microseconds



}
void loop() {

}
