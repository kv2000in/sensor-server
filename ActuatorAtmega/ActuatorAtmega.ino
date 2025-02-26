/*
LoRa Duplex communication wth callback

Sends a message every half second, and uses callback
for new incoming messages. Implements a one-byte addressing scheme,
with 0xFF as the broadcast address.

Note: while sending, LoRa radio is not listening for incoming messages.
Note2: when using the callback method, you can't use any of the Stream
functions that rely on the timeout, such as readString, parseInt(), etc.

created 28 April 2017
by Tom Igoe
*/
#include <SPI.h>              // include libraries
#include <LoRa.h>

#define BUFFER_SIZE 16


byte myAddress = 0xAA;      // destination to send to
long lastSendTime = 0;        // last send time
int interval = 2000;          // interval between sends

void setup() {
Serial.begin(9600);                   // initialize serial
while (!Serial);

Serial.println("LoRa Duplex with callback");

if (!LoRa.begin(439E6)) {             // initialize ratio at 915 MHz
Serial.println("LoRa init failed. Check your connections.");
while (true);                       // if failed, do nothing
}

LoRa.onReceive(onReceive);
LoRa.receive();
Serial.println("LoRa init succeeded.");
}

void loop() {
if (millis() - lastSendTime > interval) {
String message = "HeLoRa World!";   // send a message
sendMessage(message);
Serial.println("Sending " + message);
lastSendTime = millis();            // timestamp the message
interval = random(2000) + 1000;     // 2-3 seconds
LoRa.receive();                     // go back into receive mode
}
}

void sendMessage(String outgoing) {
LoRa.beginPacket();                   // start packet
LoRa.write(myAddress);// add sender address
LoRa.write(0xCC);
//LoRa.print(outgoing);                 // add payload
LoRa.endPacket();                     // finish packet and send it
}

void printPacketHex(const uint8_t *packet, size_t length) {
  
  Serial.printf("Packet (Hex): ");
  for (size_t i = 0; i < length; i++) {
    if (i > 0) Serial.printf(" ");
    Serial.printf("%02X", packet[i]);
  
  }
  
}

void onReceive(int packetSize) {
if (packetSize == 0) return;          // if there's no packet, return

  if (packetSize) {
    uint8_t buffer[BUFFER_SIZE];
    int index = 0;
    
    //printf("Received LoRa packet: ");
    while (LoRa.available() && index < BUFFER_SIZE - 1) {
      uint8_t byte = LoRa.read();
      //printf("%02X ", byte);
      buffer[index++] = byte;
    }
     
    buffer[index] = '\0'; // Null terminate for safety
    
printPacketHex(buffer, index);
}
}
