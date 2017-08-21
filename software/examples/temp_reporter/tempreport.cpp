/** 
   ADC Temperature Sensor for Arduino Due 
   http://wood-walker.org/wiki/doku.php/en/temperature_sensor
**/

#define ARDUINO_MAIN
#include "Arduino.h"

#include <stdint.h>

#include <../../externals/arduino_libraries/LiquidCrystal/src/LiquidCrystal.h>


void initVariant() __attribute__((weak));
void initVariant() {}

float trans = 3.3/4096;
float offset = 0.8;
float factor = 0.00265;
int fixtemp = 27;
 
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

uint32_t temperatur() {
  uint32_t ulValue = 0;
  uint32_t ulChannel;
 
  // Enable the corresponding channel
  adc_enable_channel(ADC, ADC_TEMPERATURE_SENSOR);
 
  // Enable the temperature sensor
  adc_enable_ts(ADC);
 
  // Start the ADC
  adc_start(ADC);
 
  // Wait for end of conversion
  while ((adc_get_status(ADC) & ADC_ISR_DRDY) != ADC_ISR_DRDY);
  
  // Read the value
  ulValue = adc_get_latest_value(ADC);
 
  // Disable the corresponding channel
  adc_disable_channel(ADC, ADC_TEMPERATURE_SENSOR);
 
  return ulValue;
}

/**
 * Simply blink the amber LED on the DUE with 2Hz:
 */
int main(void)
{
  watchdogSetup();
  init();
  initVariant();

  lcd.begin(16, 2);

  lcd.print("hello, world!");
  Serial.begin(115200);

  delay(1);
  pinMode(13, OUTPUT);
  long now = millis();
  while (1) {
    float treal = fixtemp + (( trans * temperatur() ) - offset ) / factor;
    Serial.print(treal);
    Serial.print("\n");
    lcd.setCursor(0, 1);
    lcd.print(treal);
    float ctr = 123412341234.0;
    while (millis() - now < 1000){
      ctr /= 12341234123.2341523;
    }
    now = millis();
  }
}
