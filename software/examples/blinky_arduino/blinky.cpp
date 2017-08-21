#define ARDUINO_MAIN
#include "Arduino.h"

#include <stdint.h>

// For simulation this must be defined
char MODULE_NAME[] = "BLINKY_ARDUINO";

void initVariant() __attribute__((weak));
void initVariant() {}

/**
 * Simply blink the amber LED on the DUE with 2Hz:
 */
int main(void)
{
  watchdogSetup();
  init();
  initVariant();

  delay(1);
  pinMode(13, OUTPUT);

  int char_in;
  for (;;)
  {
    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(1000);              // wait for a second
    digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
    delay(1000);   
  }
}
