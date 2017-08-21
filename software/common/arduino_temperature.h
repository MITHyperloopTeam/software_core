/** 
   ADC Temperature Sensor for Arduino Due 
   http://wood-walker.org/wiki/doku.php/en/temperature_sensor
**/

#ifndef ARDUINO_TEMPERATURE_H
#define ARDUINO_TEMPERATURE_H

#ifdef COMPILE_FOR_ARDUINO

#include "Arduino.h"
float kTempScale = 3.3/4096;
float kTempOffset = 0.8;
float kTempFactor = 0.00265;
int kTempFixed = 27;

float get_arduino_temperature() {
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
 
  return kTempFixed + (( kTempScale * ulValue ) - kTempOffset ) / kTempFactor;
}

#else

#include <stdlib.h>

float get_arduino_temperature() {
  return ((float)rand()) / RAND_MAX + 40.0;
}

#endif
#endif