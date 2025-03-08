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

March 2025 HW connections =
LoRa default pins CS to 10, Reset to 9 and DIO 0 to 2
Feedback Actuator Open HIGH GPIO 5
Feedback Actuator Close HIGH GPIO 6

Relay 1 GPIO 7
Relay 2 GPIO 8

*/

#include <SPI.h>
#include <LoRa.h>

#define BUFFER_SIZE 16

// Pin definitions
#define ACTUATOR_OPEN 5
#define ACTUATOR_CLOSE 6
#define RELAY1 7
#define RELAY2 8
#define OUTPUT_GPIO4 4

byte myAddress = 0xAA; // Atmega's address
long lastSendTime = 0;
int interval = 2000; // Send every 2-3 seconds

void setup() {
    Serial.begin(9600);
    while (!Serial);

    Serial.println("LoRa Duplex with callback");

    if (!LoRa.begin(439E6)) {  
        Serial.println("LoRa init failed. Check your connections.");
        while (true);
    }

    LoRa.onReceive(onReceive);
    LoRa.receive();
    Serial.println("LoRa init succeeded.");
    
    pinMode(ACTUATOR_OPEN, INPUT);
    pinMode(ACTUATOR_CLOSE, INPUT);
    pinMode(RELAY1, OUTPUT);
    pinMode(RELAY2, OUTPUT);
    pinMode(OUTPUT_GPIO4, OUTPUT);
}

void loop() {
    if (millis() - lastSendTime > interval) {
        sendSensorData();
        lastSendTime = millis();
        LoRa.receive(); 
    }
}

void sendSensorData() {
    byte actuatorState = digitalRead(ACTUATOR_OPEN) | (digitalRead(ACTUATOR_CLOSE) << 1);
    actuatorState = ~actuatorState; // One's complement encoding

    LoRa.beginPacket();
    LoRa.write(myAddress);
    LoRa.write(actuatorState);
    LoRa.endPacket();

    Serial.printf("Sent: 0x%02X 0x%02X\n", myAddress, actuatorState);
}

void onReceive(int packetSize) {
    if (packetSize != 2) return; // Only process 2-byte messages

    byte receivedAddress = LoRa.read();
    byte commandByte = LoRa.read();

    if (receivedAddress != myAddress) return; // Ignore if address doesn't match

    Serial.printf("Received: Addr=0x%02X Cmd=0x%02X\n", receivedAddress, commandByte);

    switch (commandByte) {
        case 40: digitalWrite(OUTPUT_GPIO4, LOW); break;
        case 41: digitalWrite(OUTPUT_GPIO4, HIGH); break;
        case 70: digitalWrite(RELAY1, LOW); break;
        case 71: digitalWrite(RELAY1, HIGH); break;
        case 80: digitalWrite(RELAY2, LOW); break;
        case 81: digitalWrite(RELAY2, HIGH); break;
    }
}
