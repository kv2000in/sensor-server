/*
 * https://github.com/sandeepmistry/arduino-LoRa It is based on this library.
https://github.com/loraraspi91/LoRa4Raspi - using the corresponding C++ lib
on Pi.
Default connections per lib>
SCK - SCK
MOSI-MOSI
MISO-MISO
NSS - 10
RST - 9
DIO0 -2

  #define LORA_DEFAULT_SPI           SPI
#define LORA_DEFAULT_SPI_FREQUENCY 8E6 
#define LORA_DEFAULT_SS_PIN        10
#define LORA_DEFAULT_RESET_PIN     9
#define LORA_DEFAULT_DIO0_PIN      2

  void setPins(int ss = LORA_DEFAULT_SS_PIN, int reset = LORA_DEFAULT_RESET_PIN, int dio0 = LORA_DEFAULT_DIO0_PIN);
  void setSPI(SPIClass& spi);
  void setSPIFrequency(uint32_t frequency);
  int begin(long frequency);


Jan 2025. Welcome LORA

*/

#include <SoftwareSerial.h>
#include <SPI.h>
#include <LoRa.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

volatile int f_wdt=1;

volatile int wakeCounter = 0; 
int sleepCycles = 1; //in Seconds - Multiples of 8 . 8 x 3 = 24 seconds

// Mac addresses of peers
//uint8_t peer0[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};// Each specific peer gets 6-7 packets per send. Broadcast peer only gets one packet.
//uint8_t peer0[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};//86:CC:A8:AA:20:F9 softAP MAC of server Also - sending to broadcast always results in "success" even in the absence of ACK so need to specify peer MAC Address

uint8_t peer1[] = {0x6A, 0xC6, 0x3A, 0xF4, 0x59, 0x9D};//marked 9d
uint8_t peer0[] = {0x6A, 0xC6, 0x3A, 0xF4, 0x59, 0x42};//marked 42
int maxmillistowaitforUS100serialdata = 200; //Was working well with 500
int maxcyclecount = 1;
char str[16]; //6 bytes + macaddr + 2 bytes distance + 2 bytes temp + 2 bytes battery + 4 bytes zeros

int batteryVoltage;   
int R1=100;
int R2=47;
#define DEBUG 1
#define TRIGGER_PIN  4  // if Using trigger Mode - Arduino pin tied to trigger pin on the ultrasonic sensor. For swSerial mode comm with US100 - without altering HW - Atmega swSerial TX to US100 TX
#define ECHO_PIN     3  // If Using trigger Mode - Arduino pin tied to echo pin on the ultrasonic sensor. For swSerial mode comm with US100 - without altering HW - Atmega swSerial RX to US100 RX
#define HCSR04SwitchPin 8 //via Mosfet

// the US-100 module has jumper cap on the back for Serial comm mode. No Jumper for direct - trig/echo mode
unsigned int HighLen = 0;
unsigned int LowLen  = 0;
unsigned int Len_mm  = 0;
int Temperature45 = 0;



//SoftwareSerial swSer; //ESP Software Serial
SoftwareSerial swSer(ECHO_PIN, TRIGGER_PIN); // RX, TX (Atmega Software Serial) 


void triggersensor(){
digitalWrite(TRIGGER_PIN, LOW);
pinMode(TRIGGER_PIN, OUTPUT);
delay(10);
digitalWrite(TRIGGER_PIN, HIGH);
delayMicroseconds(50);
digitalWrite(TRIGGER_PIN, LOW);
}

void printPacketHex(const uint8_t *packet, size_t length) {
  swSer.printf("Packet (Hex): ");
  for (size_t i = 0; i < length; i++) {
    if (i > 0) swSer.printf(" ");
    swSer.printf("%02X", packet[i]);
  }
  
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



void dummy_trigger_via_serial()
{
  
  Serial.flush();                               // clear tx buffer of serial port
Serial.write(0X50); 
  }

void dummy_trigger_via_swserial()
{
  swSer.flush();                               // clear tx buffer of serial port
swSer.write(0X50); 
  }



unsigned int get_distance_via_swserial()
{
    int cyclecount = 0;
    int maxcyclecount = 1;
    const unsigned long timeout = maxmillistowaitforUS100serialdata;  // Maximum wait time in milliseconds
    unsigned long startTime;

    while ((Len_mm == 0) && (cyclecount < maxcyclecount)) 
    {
        swSer.flush();  // Clear serial send buffer
        while (swSer.available()) swSer.read(); //Clear serial recv buffer 
        swSer.write(0x55);               // Trigger US-100 measurement
        startTime = millis();            // Record start time

        // Wait for 2 bytes or timeout
        while (swSer.available() < 2) 
        {
            if (millis() - startTime > timeout) 
            {
              Serial.println("TimedOut");
                return 0;  // Return 0 if timeout occurs (no valid data)
            }
        }

        // Read two bytes as soon as they are available
        HighLen = swSer.read();  
        LowLen  = swSer.read();
    Serial.print("High Byte:");
    Serial.print(HighLen,HEX);
    Serial.print("Low Byte:");
    Serial.println(LowLen,HEX);
        Len_mm  = (HighLen << 8) | LowLen;  // Combine bytes into distance value

        cyclecount++;
        delay(50);  // Small delay before retrying (if needed)
    }  

      Serial.print("Distance: ");
      Serial.print(Len_mm, DEC);          
      Serial.println("mm"); 
    return Len_mm;
}


int get_temp_via_swserial()
{
    int cyclecount = 0;
    const unsigned long timeout = maxmillistowaitforUS100serialdata;  // Maximum wait time in milliseconds
    unsigned long startTime;

    while ((Temperature45 == 0) && (cyclecount < maxcyclecount)) 
    {
        swSer.flush();  // Clear serial send buffer
        while (swSer.available()) swSer.read(); //Clear serial recv buffer 
        swSer.write(0x50);               // Trigger US-100 measurement
        startTime = millis();            // Record start time

        // Wait for 2 bytes or timeout
        while (swSer.available() < 1) 
        {
            if (millis() - startTime > timeout) 
            {
              Serial.println("TimedOut");
                return 0;  // Return 0 if timeout occurs (no valid data)
            }
        }

        // Read one byte as soon as they are available
       Temperature45 = swSer.read(); 
    Serial.print("Temp Byte:");
    Serial.print(Temperature45,HEX);

     if((Temperature45 > 1) && (Temperature45 < 130))   //the valid range of received data is (1, 130)
{
Temperature45 -= 45;                           //Real temperature = Received_Data - 45

}

        cyclecount++;
        delay(50);  // Small delay before retrying (if needed)
    }  

      Serial.print("Temp: ");
      Serial.print(Temperature45, DEC);          
      Serial.println("deg C"); 
    return Temperature45;
}






void preparedata(){


memset(str,0,16); //reset our data buffer
memcpy(str,peer0,6); // First 6 bytes = MAC Address , then 2 bytes of distance, 2 bytes of temp and 2 bytes of batteryvoltage = total 12 bytes. Padded total 16 bytes.
unsigned int mydistance;
int mytemp;

//ADC*(1.1/1024) will give the Vout at the voltage divider
//V=(Vout*((R1+R2)/R2))*1000 miliVolts
batteryVoltage = ((analogRead(A0)*(3.3/1024))*((R1+R2)/R2))*1000;

turnonsensormodule();


//memcpy(str,macAddr,6); // First 6 bytes = MAC Address , then 2 bytes of distance, 2 bytes of temp and 2 bytes of batteryvoltage = total 12 bytes. Padded total 16 bytes.
dummy_trigger_via_swserial(); // Without this - the new US-100 modules - keep the Rx line Low. delay(5) is sufficient after this.
delay(5);
mydistance=get_distance_via_swserial();
delay(10);
mytemp=get_temp_via_swserial();
memcpy(str+6,&mydistance,2);
memcpy(str+8,&mytemp,2);
memcpy(str+10,&batteryVoltage,2);

delay(10); 
turnoffsensormodule();
//#ifdef DEBUG
//swSer.print(" prepareDataend: "); swSer.println(millis());
//printPacketHex((uint8_t *)str, sizeof(str));
//#endif


}


void turnonsensormodule(){
  //reset all the variables
  Temperature45 = 0;
  HighLen = 0;
  LowLen  = 0;
Len_mm  = 0;

  //pinMode(ECHO_PIN,INPUT_PULLUP); //GPIO 5- Rx PIN for swSer
  //pinMode(ECHO_PIN,INPUT)
//pinMode(TRIGGER_PIN,OUTPUT); //GPIO 4 - Tx PIN for swSer
//swSer.begin(9600);
digitalWrite(HCSR04SwitchPin,HIGH);
delay(100);
}

void turnoffsensormodule(){
digitalWrite(HCSR04SwitchPin,LOW);
//swSer.end();
//delay(10);
//  pinMode(ECHO_PIN,OUTPUT); //GPIO 5- Rx PIN for swSer
//pinMode(TRIGGER_PIN,OUTPUT); //GPIO 4 - Tx PIN for swSer
//digitalWrite(ECHO_PIN,LOW);
//digitalWrite(TRIGGER_PIN,LOW);
//delay(20);
}

void setup() {



Serial.begin(9600);
// connect RX (Pin 0 of Arduino digital IO) to Echo/Rx (US-100), TX (Pin 1 of Arduino digital IO) to Trig/Tx (US-100) 
//Baudrate 9600 for comm with US-100

pinMode(HCSR04SwitchPin,OUTPUT);
pinMode(ECHO_PIN,INPUT); //GPIO 5- Rx PIN for swSer
pinMode(TRIGGER_PIN,OUTPUT); //GPIO 4 - Tx PIN for swSer
//if (DEBUG) {swSer.begin(9600, SWSERIAL_8N1, ECHO_PIN, TRIGGER_PIN, false, 95, 11);}     //ESP software serial
     

#ifdef DEBUG
swSer.begin(9600);
#endif




  /*** Setup the WDT ***/
  
  /* Clear the reset flag. */
  MCUSR &= ~(1<<WDRF);
  
  /* In order to change WDE or the prescaler, we need to
   * set WDCE (This will allow updates for 4 clock cycles).
   */
  WDTCSR |= (1<<WDCE) | (1<<WDE);

  /* set new watchdog timeout prescaler value */
  WDTCSR = 1<<WDP0 | 1<<WDP3; /* 8.0 seconds */
  
  /* Enable the WD interrupt (note no reset). */
  WDTCSR |= _BV(WDIE);

}


void senddata(){
//Prepare Data
preparedata();

//Send Data
if (!LoRa.begin(439E6)) {
#ifdef DEBUG
swSer.println("Starting LoRa failed!");while (1);
#endif
}
//stable radio
delay(20);

// send packet
LoRa.beginPacket();            // Start a new LoRa packet
LoRa.write((uint8_t *)str, 16); // Send all 16 bytes of the array as binary data
LoRa.endPacket();              // Finish and send the packet

//put LoRa radio to sleep
LoRa.sleep();
}



/***************************************************
 *  Name:        ISR(WDT_vect)
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Watchdog Interrupt Service. This
 *               is executed when watchdog timed out.
 *
 ***************************************************/
ISR(WDT_vect)
{
  if(f_wdt == 0)
  {
    f_wdt=1;
  }
  else
  {
    Serial.println("WDT Overrun!!!");
  }
}


/***************************************************
 *  Name:        enterSleep
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Enters the arduino into sleep mode.
 *
 ***************************************************/
void enterSleep(void)
{
  //Serial.println("Sleep called");
  //Serial.println("WDT value is");
  //Serial.println(f_wdt);
  //delay (50);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   /* EDIT: could also use SLEEP_MODE_PWR_DOWN for lowest power consumption. */
  sleep_enable();
  
  /* Now enter sleep mode. */
  sleep_mode();
  
  /* The program will continue from here after the WDT timeout*/
  sleep_disable(); /* First thing to do is disable sleep. */
  
  /* Re-enable the peripherals. */
  power_all_enable();
}


void loop(void)
{
  
if(f_wdt == 1)
  {
  /* Don't forget to clear the flag. */
  f_wdt = 0;  
  wakeCounter++; 
   if (wakeCounter >= sleepCycles) {
            wakeCounter = 0; 
    /* Do something */
   senddata();
  // Serial.println("sending");
  //delay (50-500) gave incosistent radio comm
   delay(10);
   }

    
    /* Re-enter sleep mode. */
     enterSleep();

  }
  else
  {
    /* Do nothing. */
  }
  }
  
