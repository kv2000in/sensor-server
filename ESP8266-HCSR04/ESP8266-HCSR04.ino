
#include <ESP8266WiFi.h>

//#include <ArduinoOTA.h> // See below for comments from Feb 2020
WiFiClient client;

// WiFi credentials.
const char* WIFI_SSID = "****";
const char* WIFI_PASS = "****";
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
#define TRIGGER_PIN  4  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     5  // Arduino pin tied to echo pin on the ultrasonic sensor.

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
      Serial.println("WiFi Connection Failed");
      delay(10000);
    }

    delay(500);
  
    // Only try for 5 seconds.
    if (millis() - wifiConnectStart > 15000) {
  Serial.println("WiFi Stopped Trying");
      return;
    }
  
  }

  
Serial.println("WiFi Connected");
  
   
}

unsigned int get_distance_via_serial()
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

     if((Len_mm > 1) && (Len_mm < 1000))       // normal distance should between 1mm and 10000mm (1mm, 10m)
        {
           delay(10);                                    // wait 500ms
  return Len_mm/10;
        }
        else 
        return 999;
    

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

  
 client.print(str);
  delay(5);
  //Close the socket - server is closing after one receive at the moment so it may not be necessary to close by the client
  client.stop();
 // Serial.println(millis()-currentmillis); //=1489
  }

void setup() {

//Has to be moved to setup for it to compile
//str[6]=':';
//str[12]=':';
  
  Serial.begin(9600);
  // connect RX (Pin 0 of Arduino digital IO) to Echo/Rx (US-100), TX (Pin 1 of Arduino digital IO) to Trig/Tx (US-100) 
  //Baudrate 9600 for comm with US-100
  
connect();
delay(50);
//senddata();
  
 //ESP.deepSleep(30e6); // 20e6 is 20 microseconds RF_NO_CAL - no change in current
  //ESP.deepSleep(2e6); // 20e6 is 20 microseconds



}
void loop() {

senddata();
delay(10000);
}
