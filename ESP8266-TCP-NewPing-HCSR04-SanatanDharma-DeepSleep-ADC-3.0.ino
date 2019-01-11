#include <ESP8266WiFi.h>
#include <NewPing.h>

WiFiClient client;

// WiFi credentials.
const char* WIFI_SSID = "********";
const char* WIFI_PASS = "********";
const char* host = "192.168.1.110";  // TCP Server IP
const int   port = 9999;            // TCP Server Port

//unsigned long currentmillis=millis();
//int trigPin = 4;    
//int echoPin = 5;
int HCSR04SwitchPin = 12; 
int batteryVoltage;   
//Sensor 1 R1=986k , R2 = 298k; Sensor 2 R1=974k, R2 = 296k
int R1=986;
int R2=298;
#define TRIGGER_PIN  4  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     5  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 300 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
long cm;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.



//Total 13 bytes needed - [1-_999-_3222\0] - Gave 14 to prevent buffer overflow

//First 1 and '-' are already initialized
char str[14] = {'1',':',' ',' ',' ',' ',':',' ',' ',' ',' ',' ','\0'};



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
  WiFi.config(IPAddress(192,168,1,201), IPAddress(192,168,1,2), IPAddress(255,255,255,0),IPAddress(192,168,1,2));
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

  Serial.println("Device Started");
  Serial.println("-------------------------------------");
  Serial.println("Running Deep Sleep Firmware!");
  Serial.println("-------------------------------------");
//Todo: Connect HCSR04 vcc (5v) via a GPIO (3.3v) and a transistor so that HCSR04 is not using quiescent current when not in use
//HCSR04SwitchPin is connected to the base of the transistor. HY-SRF05 - much stable readings. (1/20/18)
//Should it be turned on after the wifi connection has been made?
//It takes about 7.5 seconds to connect to wifi - total 8.5 to 9.5 seconds per loop
//By giving static IP address - whole loop completed in 2.5-3 sec

pinMode(HCSR04SwitchPin,OUTPUT);
digitalWrite(HCSR04SwitchPin,HIGH);
connect();
delay(100);
  //ADC*(1.1/1024) will give the Vout at the voltage divider
  //V=(Vout*((R1+R2)/R2))*1000 miliVolts
batteryVoltage = ((analogRead(A0)*(1.1/1024))*((R1+R2)/R2))*1000;
  // convert the time into a distance
  cm = ((sonar.ping_median(5))/2) / 29.1;
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
  Serial.println("Going into deep sleep for 30 seconds");
 ESP.deepSleep(30e6); // 20e6 is 20 microseconds RF_NO_CAL - no change in current
  //ESP.deepSleep(2e6); // 20e6 is 20 microseconds
}
void loop() {


}
