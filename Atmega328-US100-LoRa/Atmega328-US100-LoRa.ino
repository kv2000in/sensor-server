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

*/

#include <SPI.h>
#include <LoRa.h>

int counter = 0;

void setup() {
Serial.begin(9600);
while (!Serial);

Serial.println("LoRa Sender");

if (!LoRa.begin(439E6)) {
Serial.println("Starting LoRa failed!");
while (1);
}
}

void loop() {
Serial.print("Sending packet: ");
Serial.println(counter);

// send packet
LoRa.beginPacket();
LoRa.print("hello ");
LoRa.print(counter);
LoRa.endPacket();

counter++;

delay(5000);
}
