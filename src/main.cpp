#include <Arduino.h>

HardwareSerial DEBUG_PORT(PA10, PA9); // Serial line to the programming header
const int pin = PA7;

void setup() {
  //SerialUSB.begin(115200);
  pinMode(pin, OUTPUT);
  DEBUG_PORT.begin(115200);
}

void loop() {
  DEBUG_PORT.println("HELLO WORLD");
  digitalWrite(pin, HIGH);
  delay(100);
  digitalWrite(pin, LOW);
  delay(1000);
}