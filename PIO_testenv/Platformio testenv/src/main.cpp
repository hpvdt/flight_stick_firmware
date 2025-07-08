// This code has been built into submodules testing each peripheral of the microcontroller.
// This has been done so that each peripheral can be operated independently and tested in isolation.
// At any point, one submodule can simply be commented out in void loop() to disable it.

#include <Arduino.h>
#include <string.h>
#include <SPI.h>

#define green_led  PA7
#define rgb_green  PB0
#define rgb_red    PB1
#define rgb_blue   PB2

#define SPI2_CS    PB6
#define PIN_CS     PB6   // Chip Select
#define PIN_SCLK   PB3   // SPI Clock
#define PIN_MISO   PB4   // Master In Slave Out
#define PIN_MOSI   PB5   // Master Out Slave In
#define PIN_DRDY   PB7   // Data Ready (optional, but useful for sync check)
#define PIN_RESET  PA9    // SYNC/RESET pin (NOT ACTUALLY SET)
#define PIN_CLKOUT PA8    // MCO — Master Clock Output

static void MX_USART2_UART_Init(void); //initialize the USART2 UART peripheral
static void LED_Init(void); //initialize the LED pins
void LED_test(int delay_time);
void checkAdcConnection();
void adcSetup();


UART_HandleTypeDef huart2;
uint64_t counter =0;

void setup() {
  LED_Init(); // Initialize the LED
  Serial2.begin(115200); // PA2 = TX, PA3 = RX
  Serial2.println("Hello, USART2!"); // Initialize the USART2 UART peripheral
  adcSetup();
}

void loop() {
  LED_test(100);
  Serial2.println(counter);
  counter++;
  checkAdcConnection();
  delay(1000);
}


static void LED_Init(void) // add more LEDs as needed
{
  pinMode(green_led, OUTPUT); // Set PA7 as output for the green LED
  digitalWrite(green_led, LOW); // Initialize the LED to LOW (off) 
  pinMode(rgb_red, OUTPUT);
  digitalWrite(rgb_red, LOW);
  pinMode(rgb_blue, OUTPUT);
  digitalWrite(rgb_blue, LOW);
  pinMode(rgb_green, OUTPUT);
  digitalWrite(rgb_green, LOW);
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

void adcSetup() {
  // Configure MCO (Master Clock Output) on PA8 to output 8.192 MHz (assuming 72 MHz SYSCLK)
  pinMode(PIN_CLKOUT, OUTPUT);
  RCC->CFGR |= RCC_CFGR_MCO_SYSCLK; // Set MCO source to SYSCLK (72 MHz)

  // Configure GPIOs
  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_RESET, OUTPUT);
  pinMode(PIN_DRDY, INPUT);
  digitalWrite(PIN_CS, HIGH);       // Deassert CS
  digitalWrite(PIN_RESET, HIGH);    // Deassert reset

  // Perform reset pulse
  digitalWrite(PIN_RESET, LOW);
  delay(10);
  digitalWrite(PIN_RESET, HIGH);
  delay(10);

  // Begin SPI at 1 MHz, CPOL = 0, CPHA = 1
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1)); // Mode1: CPOL=0, CPHA=1
}

void checkAdcConnection() {
  digitalWrite(PIN_CS, LOW);       // Select ADC
  delayMicroseconds(1);

  uint16_t response = SPI.transfer16(0x0000); // Send NOP command

  digitalWrite(PIN_CS, HIGH);      // Deselect ADC

  if (response != 0xFFFF && response != 0x0000) {
    digitalWrite(rgb_blue,HIGH);
    delay(3000);
    digitalWrite(rgb_blue,LOW);

  } else {
    digitalWrite(rgb_red,HIGH);
    delay(3000);
    digitalWrite(rgb_red,LOW);
  }

  delay(500);
}