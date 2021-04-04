#include <ESP8266WiFi.h>
#include <NewPing.h>
//#include <ArduinoOTA.h> // See below for comments from Feb 2020
WiFiClient client;

// WiFi credentials.
const char* WIFI_SSID = "****";
const char* WIFI_PASS = "*****";
const char* host = "192.168.1.110";  // TCP Server IP
const int   port = 9999;            // TCP Server Port

//*************************************************
//Need to change Resistance values, static IP addresses, add wifi credentials, double check server IP address and port and First char ("1 or 2") when flashing it into different sensor nodes
//Sensor 1 R1=986k , R2 = 298k; Sensor 2 R1=974k, R2 = 296k Sensor 3 R1=1001k, R2 = 298K Sensor 4 R1=990k, R2 = 298k
//Sensor 1 static IP 192.168.1.201, sensor 2 static IP 192.168.1.202 snd so on
//For flashing - power the board with separate 5V power supply. Connect the gnd, rx, tx from USB serial to the programming board, flash mode is activated by holding GPIO 0 to ground at power on OR RESET (taking reset to GND)
//ESP201 boards are 1M flash, DIO, 40 MHz
//**************************************************

int HCSR04SwitchPin = 12; //Takes the ground of HCSR04 to ground via a transistor - powersaving feature
int batteryVoltage;   
int R1=990;
int R2=298;
#define TRIGGER_PIN  4  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     5  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 300 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
long cm,duration;
//NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.



//Total 13 bytes needed - [1-_999-_3222\0] - Gave 14 to prevent buffer overflow

//First 1 and '-' are already initialized
char str[14] = {'2',':',' ',' ',' ',' ',':',' ',' ',' ',' ',' ','\0'};



void connect() {

  // Connect to Wifi.
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

//  WiFi.begin(WIFI_SSID, WIFI_PASS);

  // WiFi fix: https://github.com/esp8266/Arduino/issues/2186
 // WiFi.persistent(false);
 // WiFi.mode(WIFI_OFF);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  WiFi.config(IPAddress(192,168,1,204), IPAddress(192,168,1,2), IPAddress(255,255,255,0),IPAddress(192,168,1,2));
  unsigned long wifiConnectStart = millis();

  while (WiFi.status() != WL_CONNECTED) {
    // Check to see if
    if (WiFi.status() == WL_CONNECT_FAILED) {
      Serial.println("Failed to connect to WiFi. Please verify credentials: ");
      delay(10000);
    }

    delay(500);
    Serial.println("...");
    // Only try for 5 seconds.
    if (millis() - wifiConnectStart > 15000) {
      Serial.println("Failed to connect to WiFi");
      return;
    }
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
 // Serial.println(millis()-currentmillis); //=506
  
  Serial.println();
  if (!client.connect(host, port)) {
     Serial.println("connection failed");
   return;
    }
  // Serial.println(millis()-currentmillis); //=1434
   
}




void setup() {
  
  Serial.begin(115200);
  Serial.setTimeout(2000);

  // Wait for serial to initialize.
  while (!Serial) { }

  Serial.println("Device Started-I am sensor node # 4");
  Serial.println("-------------------------------------");
  Serial.println("Running Deep Sleep Firmware! 4-3-21");
  Serial.println("-------------------------------------");
//Todo: Connect HCSR04 vcc (5v) via a GPIO (3.3v) and a transistor so that HCSR04 is not using quiescent current when not in use
//HCSR04SwitchPin is connected to the base of the transistor. HY-SRF05 - much stable readings. (1/20/18)
//Should it be turned on after the wifi connection has been made?
//It takes about 7.5 seconds to connect to wifi - total 8.5 to 9.5 seconds per loop
//By giving static IP address - whole loop completed in 2.5-3 sec
//Using US-100 now. Temp compensated and not dependent on power supply. However, power on reset is a problem. Zero value readings when running using the transistor switch.
//When US-100 is on all the time (by connecting GND to System GND instead of via transistor) - stable readings. 
// Testing if the delay after the transistor switches on - needs to be increased to get a reading otherwise - will leave it in always on mode.
//Doesn't seem to use a whole of current when always on but definitely drains the battery quicker
//Even with 1000 ms delay - still reading 0 when switched on via transistor

pinMode(HCSR04SwitchPin,OUTPUT);
digitalWrite(HCSR04SwitchPin,HIGH);
connect();
delay(1000);
  //ADC*(1.1/1024) will give the Vout at the voltage divider
  //V=(Vout*((R1+R2)/R2))*1000 miliVolts
batteryVoltage = ((analogRead(A0)*(1.1/1024))*((R1+R2)/R2))*1000;
  // convert the time into a distance

//HR-SC04 
   // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(20);
  digitalWrite(TRIGGER_PIN, LOW);
 delayMicroseconds(20);
   digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(20);
  digitalWrite(TRIGGER_PIN, LOW);
 delayMicroseconds(20);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(ECHO_PIN, INPUT);
  duration = pulseIn(ECHO_PIN, HIGH);
 
  // convert the time into a distance
  cm = (duration/2) / 29.1;
 
  
  
  
  //cm = ((sonar.ping_median(5))/2) / 29.1;
  //convert int to ASCII and put it in the char array - adds '\0' at the end so string terminates after this - even if there is more stuff after this in the array
  itoa( cm, str+2, 10 );
  int length = strlen(str);
  //replace the trailing '\0' with ':'
  str[length]=':';
  //Add the vcc value after ':'
  itoa( batteryVoltage, str+length+1, 10 );

  Serial.println(str);
 //Serial.println(millis()-currentmillis); //=1468
 client.print(str);
  delay(5);
  //Close the socket - server is closing after one receive at the moment so it may not be necessary to close by the client
  client.stop();
 // Serial.println(millis()-currentmillis); //=1489
  Serial.println("Going into deep sleep for 30 seconds. RST has to be tied to GPIO16 for wakeup");
 ESP.deepSleep(30e6); // 20e6 is 20 microseconds RF_NO_CAL - no change in current
  //ESP.deepSleep(2e6); // 20e6 is 20 microseconds

// Feb 2020 - trying to add Over the air update. Major problem is  - device is in deep sleep and wakes up and runs the setup. so, wont work unless timing is perfect. 
// May lead to increased battery consumption.
//Here is the OTA code just for reference

 /* ************OTA********************* */
 /* 
    // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");

  // No authentication by default
  // ArduinoOTA.setPassword((const char *)"123");
   
   ArduinoOTA.onStart([]() {
    Serial1.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial1.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial1.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial1.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial1.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial1.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial1.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial1.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial1.println("End Failed");
  });
  ArduinoOTA.begin();
  */
/****************************************************/  

}
void loop() {


}
