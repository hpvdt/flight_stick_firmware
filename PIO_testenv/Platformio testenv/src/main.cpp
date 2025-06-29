// This code has been built into submodules testing each peripheral of the microcontroller.
// This has been done so that each peripheral can be operated independently and tested in isolation.
// At any point, one submodule can simply be commented out in void loop() to disable it.

#include <Arduino.h>
#include <string.h>


static void MX_USART2_UART_Init(void); //initialize the USART2 UART peripheral
static void LED_Init(void); //initialize the LED pins
void LED_test(int delay_time);

const int green_led= PA7;
UART_HandleTypeDef huart2;
uint64_t counter =0;

void setup() {
  LED_Init(); // Initialize the LED
  Serial2.begin(115200); // PA2 = TX, PA3 = RX
  Serial2.println("Hello, USART2!"); // Initialize the USART2 UART peripheral
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