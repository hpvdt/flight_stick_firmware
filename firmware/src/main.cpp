// This code has been built into submodules testing each peripheral of the microcontroller.
// This has been done so that each peripheral can be operated independently and tested in isolation.
// At any point, one submodule can simply be commented out in void loop() to disable it.

#include <Arduino.h>
#include <string.h>
#include <SPI.h>

#define greenLed   PA7
#define rgbGreen   PB0
#define rgbRed     PB1
#define rgbBlue    PB2

#define spi2Cs     PB6
#define pinCs      PB6   // Chip Select
#define pinSclk    PB3   // SPI Clock
#define pinMiso    PB4   // Master In Slave Out
#define pinMosi    PB5   // Master Out Slave In
#define pinDrdy    PB7   // Data Ready (optional, but useful for sync check)
#define pinReset   PA9   // SYNC/RESET pin (NOT ACTUALLY SET)
#define pinClkOut  PA8   // MCO — Master Clock Output

static void initUsart2Uart(void); // Initialize the USART2 UART peripheral
static void initLed(void);        // Initialize the LED pins
void ledTest(uint32_t delayTime);
void checkAdcConnection();
void setupAdc();

UART_HandleTypeDef huart2;
uint64_t counter = 0;

void setup() {
  initLed(); 
  Serial2.begin(115200); // PA2 = TX, PA3 = RX
  Serial2.println("Hello, USART2!"); // Initialize the USART2 UART peripheral
  setupAdc();
}

void loop() {
  ledTest(1000);
  //Serial2.println(counter);
  //counter++;
  //checkAdcConnection();
  delay(1000);
}

static void initLed(void) {
  pinMode(greenLed, OUTPUT); 
  digitalWrite(greenLed, HIGH); 
  pinMode(rgbRed, OUTPUT);
  digitalWrite(rgbRed, HIGH);
  pinMode(rgbBlue, OUTPUT);
  digitalWrite(rgbBlue, HIGH);
  pinMode(rgbGreen, OUTPUT);
  digitalWrite(rgbGreen, HIGH);
}

void ledTest(uint32_t delayTime) {
  for (int i = 0; i < 5; i++) {
    digitalWrite(rgbBlue, HIGH);
    delay(delayTime);
    digitalWrite(rgbRed, LOW);
    delay(delayTime);
  }
}

void setupAdc() {
  pinMode(pinClkOut, OUTPUT);
  RCC->CFGR |= RCC_CFGR_MCO_SYSCLK;

  pinMode(pinCs, OUTPUT);
  pinMode(pinReset, OUTPUT);
  pinMode(pinDrdy, INPUT);
  digitalWrite(pinCs, HIGH);
  digitalWrite(pinReset, HIGH);

  digitalWrite(pinReset, LOW);
  delay(10);
  digitalWrite(pinReset, HIGH);
  delay(10);

  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
}

void checkAdcConnection() {
  digitalWrite(pinCs, LOW);
  delayMicroseconds(1);

  uint16_t response = SPI.transfer16(0x0000); 

  digitalWrite(pinCs, HIGH);

  if (response != 0xFFFF && response != 0x0000) {
    digitalWrite(rgbBlue, HIGH);
    delay(3000);
    digitalWrite(rgbBlue, LOW);
  } else {
    digitalWrite(rgbRed, HIGH);
    delay(3000);
    digitalWrite(rgbRed, LOW);
  }

  delay(500);
}
