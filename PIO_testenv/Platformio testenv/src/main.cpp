// This code has been built into submodules testing each peripheral of the microcontroller.
// This has been done so that each peripheral can be operated independently and tested in isolation.
// At any point, one submodule can simply be commented out in void loop() to disable it.

#include <Arduino.h>
#include <string.h>
#include <SPI.h>



static void MX_USART2_UART_Init(void); //initialize the USART2 UART peripheral
static void LED_Init(void); //initialize the LED pins
void LED_test(int delay_time);
uint16_t readRegister(uint8_t regAddr);


const int green_led= PA7;
const int SPI2_CS = PB6;
UART_HandleTypeDef huart2;
uint64_t counter =0;

void setup() {
  LED_Init(); // Initialize the LED
  Serial2.begin(115200); // PA2 = TX, PA3 = RX
  Serial2.println("Hello, USART2!"); // Initialize the USART2 UART peripheral
  SPI.begin(); // Initialize SPI
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1)); // ADS131M03 expects SPI mode 1
  pinMode(SPI2_CS, OUTPUT); // Set the SPI2 chip select pin as output
  digitalWrite(SPI2_CS, HIGH); // Set the chip select pin high (inactive
  delay(10);  // Give time for ADC to boot

  uint16_t id = readRegister(0x00); // Read ID register
  Serial.print("ADS131M03 ID Register: 0x");
  Serial.println(id, HEX);

  if ((id & 0xFF00) == 0x3000) {
    Serial.println("✅ SPI communication successful!");
  } else {
    Serial.println("❌ SPI communication failed or unexpected ID.");
  }
}

void loop() {
  //LED_test(50);
  Serial2.println(counter);
  counter++;
  delay(1000);
}


static void LED_Init(void) // add more LEDs as needed
{
  pinMode(green_led, OUTPUT); // Set PA7 as output for the green LED
  digitalWrite(green_led, LOW); // Initialize the LED to LOW (off) 
  return; 
}

void LED_test(int delay_time)
{
  for (int i = 0; i < 5; i++) // Blink the LED 5 times
    {
    digitalWrite(green_led, HIGH); // Turn on the green LED
    delay(delay_time); // Wait for the specified delay time
    digitalWrite(green_led, LOW); // Turn off the green LED
    delay(delay_time); // Wait for the specified delay time
    }
}

uint16_t readRegister(uint8_t regAddr) {
  // Create READ REG command: 0b001xxxxx00000000
  uint16_t command = 0x2000 | ((regAddr & 0x1F) << 7);
  
  uint8_t highByte = (command >> 8) & 0xFF;
  uint8_t lowByte  = command & 0xFF;

  digitalWrite(SPI2_CS, LOW);
  delayMicroseconds(1);

  // Send command
  SPI.transfer(highByte);
  SPI.transfer(lowByte);

  // Read 16-bit response
  uint8_t respHigh = SPI.transfer(0x00);
  uint8_t respLow  = SPI.transfer(0x00);

  digitalWrite(SPI2_CS, HIGH);

  return ((uint16_t)respHigh << 8) | respLow;
}