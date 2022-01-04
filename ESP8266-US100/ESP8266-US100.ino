/*
Serial uses UART0, which is mapped to pins GPIO1 (TX) and GPIO3 (RX). 
Serial may be remapped to GPIO15 (TX) and GPIO13 (RX) by calling Serial.swap() after Serial.begin. 
Calling swap again maps UART0 back to GPIO1 and GPIO3.
http://arduino.esp8266.com/Arduino/versions/2.1.0-rc2/doc/reference.html#serial
Dec 2021 - 
todo:
1) ESPMultiwifi - add ability to look for extenders and connect
1a) Add temperature recording to the server side
2) Reduce the total on time to reduce power usage.
3) Increase sleep duration to 1 min
4) Waterproof/weatherproof,


*/
#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
//#include <ArduinoOTA.h> // See below for comments from Feb 2020
WiFiClient client;

// WiFi credentials.
const char* WIFI_SSID = "**";
const char* WIFI_PASS = "**";
const char* host = "192.168.1.152";  // TCP Server IP
const int   port = 9999;            // TCP Server Port

//*************************************************
//Need to change Resistance values, static IP addresses, add wifi credentials, double check server IP address and port and First char ("1 or 2") when flashing it into different sensor nodes
//Sensor 1 R1=986k , R2 = 298k; Sensor 2 R1=974k, R2 = 296k Sensor 3 R1=1001k, R2 = 298K Sensor 4 R1=990k, R2 = 298k
//Sensor 1 static IP 192.168.1.201, sensor 2 static IP 192.168.1.202 snd so on
//For flashing - power the board with separate 5V power supply. Connect the gnd, rx, tx from USB serial to the programming board, flash mode is activated by holding GPIO 0 to ground at power on OR RESET (taking reset to GND)
//ESP201 boards are 1M flash, DIO, 40 MHz
//**************************************************
SoftwareSerial swSer;
int batteryVoltage;   
int R1=990;
int R2=298;
#define TRIGGER_PIN  4  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     5  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define HCSR04SwitchPin 12
// the US-100 module has jumper cap on the back.
unsigned int HighLen = 0;
unsigned int LowLen  = 0;
unsigned int Len_mm  = 0;
int Temperature45 = 0;

//Total 13 bytes needed - [1:_999:_3222:_999\0] - Gave 14 to prevent buffer overflow

//First 1 and '-' are already initialized //Has to be moved to setup for it to compile
//char str[20] = {'2'},':','2','3','4','5',':','7','8','9','10','11',':','13','14','15','16','17','\0'};


void connect() {


WiFi.mode(WIFI_STA);
WiFi.begin(WIFI_SSID, WIFI_PASS);
WiFi.config(IPAddress(192,168,1,204), IPAddress(192,168,1,2), IPAddress(255,255,255,0),IPAddress(192,168,1,2));
unsigned long wifiConnectStart = millis();

while (WiFi.status() != WL_CONNECTED) {
// Check to see if
if (WiFi.status() == WL_CONNECT_FAILED) {
swSer.println("WiFi Connection Failed");
delay(10000);
}

delay(500);

// Only try for 5 seconds.
if (millis() - wifiConnectStart > 15000) {
swSer.println("WiFi Stopped Trying");
return;
}

}


swSer.println("WiFi Connected");


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

unsigned int get_distance_via_swserial()
{
int cyclecount=0;
int maxcyclecount=5;

while ((Len_mm==0) && (cyclecount<maxcyclecount)) 
{
swSer.flush();                               // clear receive buffer of serial port
swSer.write(0X55);                           // trig US-100 begin to measure the distance
delay(500);                                   // delay 500ms to wait result
if(swSer.available() >= 2)                   // when receive 2 bytes 
{
HighLen = swSer.read();                   // High byte of distance
LowLen  = swSer.read();                   // Low byte of distance
Len_mm  = HighLen*256 + LowLen;            // Calculate the distance

}
cyclecount++;
delay(50);

}  

return Len_mm/10;

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

int get_temp_via_swserial()
{
swSer.flush();       // clear receive buffer of serial port
swSer.write(0X50);   // trig US-100 begin to measure the temperature
delay(500);            //delay 500ms to wait result
if(swSer.available() >= 1)            //when receive 1 bytes 
{
Temperature45 = swSer.read();     //Get the received byte (temperature)
if((Temperature45 > 1) && (Temperature45 < 130))   //the valid range of received data is (1, 130)
{
Temperature45 -= 45;                           //Real temperature = Received_Data - 45

}
}

delay(10);                            //delay 500ms
return Temperature45;
}

void senddata(){

if (!client.connect(host, port)) {

return;
}
//ADC*(1.1/1024) will give the Vout at the voltage divider
//V=(Vout*((R1+R2)/R2))*1000 miliVolts
batteryVoltage = ((analogRead(A0)*(1.1/1024))*((R1+R2)/R2))*1000;
// convert the time into a distance

//US-100 
// The sensor is triggered by serial




// convert the time into a distance
//cm = get_distance_via_serial()/10;
//temp=get_temp_via_serial();


char str[20] = {'2',':'};
//cm = ((sonar.ping_median(5))/2) / 29.1;
//convert int to ASCII and put it in the char array - adds '\0' at the end so string terminates after this - even if there is more stuff after this in the array
itoa( get_distance_via_serial(), str+2, 10 );
int alength = strlen(str);
str[alength]=':';
//Add the vcc value after ':'
itoa( batteryVoltage, str+alength+1, 10 ); // for some reason +1 outputs starnge 2:32:283:26 or  2:11305:275:26
int blength = strlen(str);
str[blength]=':';
//Add the temp value after ':'
itoa( get_temp_via_serial(), str+blength+1, 10 );

swSer.println(str);
client.print(str);

swSer.println("sent");

delay(5);
//Close the socket - server is closing after one receive at the moment so it may not be necessary to close by the client
client.stop();
// Serial.println(millis()-currentmillis); //=1489
}
void turnonsensormodule(){
digitalWrite(HCSR04SwitchPin,HIGH);
}
void turnoffsensormodule(){
digitalWrite(HCSR04SwitchPin,LOW);
}
void setup() {

//Has to be moved to setup for it to compile
//str[6]=':';
//str[12]=':';

Serial.begin(9600);
// connect RX (Pin 0 of Arduino digital IO) to Echo/Rx (US-100), TX (Pin 1 of Arduino digital IO) to Trig/Tx (US-100) 
//Baudrate 9600 for comm with US-100
delay(50);
Serial.swap(); //GPIO15 (TX) and GPIO13 (RX)
delay(50);
pinMode(HCSR04SwitchPin,OUTPUT);
pinMode(ECHO_PIN,INPUT);
pinMode(TRIGGER_PIN,OUTPUT); 
swSer.begin(9600, SWSERIAL_8N1, ECHO_PIN, TRIGGER_PIN, false, 95, 11); 

//void SoftwareSerial::begin(uint32_t baud, SoftwareSerialConfig config, int8_t rxPin, int8_t txPin, bool invert, int bufCapacity, int isrBufCapacity)

//https://github.com/plerup/espsoftwareserial/blob/master/src/SoftwareSerial.h
connect();
turnonsensormodule();
delay(100);
senddata();
delay(50); 
turnoffsensormodule();
delay(20);
ESP.deepSleep(30e6); // 20e6 is 20 microseconds RF_NO_CAL - no change in current
//ESP.deepSleep(2e6); // 20e6 is 20 microseconds



}
void loop() {

//senddata();
//delay(10000);
//swSer.println("Data Sent");
}
