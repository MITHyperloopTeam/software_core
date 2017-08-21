#define ARDUINO_MAIN
#include "../../externals/arduino_core/Arduino.h"
#include "../../externals/arduino_core/UARTClass.h"

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

  Serial.begin(9600);

  int char_in;
  for (;;)
  {
    char_in = Serial.read();
    
    if (char_in >= 0)
      Serial.write(char_in);
  }
}
