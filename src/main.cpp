
#include <Arduino.h>

HardwareSerial DEBUG_PORT(PA2, PA3); // Serial line to the programming header
const int pin = PA7;

void setup() {
  //SerialUSB.begin(115200);
  pinMode(pin, OUTPUT);
  DEBUG_PORT.begin(115200);
}

void loop() {
  DEBUG_PORT.println("HELLO WORLD");
  digitalWrite(pin, HIGH);
  delay(1000);
  digitalWrite(pin, LOW);
}