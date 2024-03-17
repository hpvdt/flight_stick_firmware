
#include <Arduino.h>
int pin = PA11;

void setup() {
  //SerialUSB.begin(115200);
  pinMode(pin, OUTPUT);
}

void loop() {
  //SerialUSB.println(millis());
  delay(1000);
  
  digitalWrite(pin, HIGH);
  delay(1000);
  
  digitalWrite(pin, LOW);
}