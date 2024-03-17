/*#include <hx711.h>
#include <Arduino.h>

const int sheardata = PB4;
const int shearclk = PB5;

HX711 u2; //PB4 is dout-shear
            // PB5 is clk-shear

void setup() {
  Serial.begin(57600);
  u2.begin(sheardata, shearclk);
}

void loop() {

  if (u2.is_ready()) {
    u2.set_u2();    
    Serial.println("Tare... remove any weights from the u2.");
    delay(5000);
    u2.tare();
    Serial.println("Tare done...");
    Serial.print("Place a known weight on the u2...");
    delay(5000);
    long reading = u2.get_units(10);
    Serial.print("Result: ");
    Serial.println(reading);
  } 
  else {
    Serial.println("HX711 not found.");
  }
  delay(1000);
}
*/