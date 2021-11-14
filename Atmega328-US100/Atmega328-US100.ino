/* Test sketch for US-100 ultrasonic modules. UART mode = jumper on the back. PWM mode = jumper off
 *  Connect Tx of Atmega to Tx/Trigger of US-100 and connect Rx of Atmega to Rx/Echo of US-100
 *  https://sites.google.com/site/myscratchbooks/home/projects/project-09-ultrasonic-sensor
 */
#include <SoftwareSerial.h>


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

SoftwareSerial swSer(ECHO_PIN, TRIGGER_PIN); //Rx, Tx

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
swSer.flush();                               // clear receive buffer of serial port
swSer.write(0X55);                           // trig US-100 begin to measure the distance
delay(500);                                   // delay 500ms to wait result
if(swSer.available() >= 2)                   // when receive 2 bytes 
{
HighLen = swSer.read();                   // High byte of distance
LowLen  = swSer.read();                   // Low byte of distance
Len_mm  = HighLen*256 + LowLen;            // Calculate the distance

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



void setup() {



Serial.begin(9600);
// connect RX (Pin 0 of Arduino digital IO) to Echo/Rx (US-100), TX (Pin 1 of Arduino digital IO) to Trig/Tx (US-100) 
//Baudrate 9600 for comm with US-100


pinMode(ECHO_PIN,INPUT);
pinMode(TRIGGER_PIN,OUTPUT);
swSer.begin(9600);

swSer.println("Ready");

}
void loop() {


delay(10000);
swSer.print("Distance = ");
swSer.println(get_distance_via_serial());
swSer.print("Temperature = ");
swSer.println(get_temp_via_serial());
}
