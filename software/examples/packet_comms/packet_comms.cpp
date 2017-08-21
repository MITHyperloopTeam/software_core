/** 
**/

#define ARDUINO_MAIN
#include "Arduino.h"

#include "../../externals/PacketSerial/src/PacketSerialArduino.h"

#include <stdint.h>


void initVariant() __attribute__((weak));
void initVariant() {}

float trans = 3.3/4096;
float offset = 0.8;
float factor = 0.00265;
int fixtemp = 27;


PacketSerial_<COBS, 0, 256> serial;

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

// This is our packet callback.
// The buffer is delivered already decoded.
void onPacket(const uint8_t* buffer, size_t size, void * extra)
{
  // Make a temporary buffer.
  char tmp[100]; 

  // Send a string "I don't know what's going on"
  sprintf(tmp, "Wie bitte");

  // Send the reversed buffer back.
  // The send() method will encode the buffer
  // as a packet, set packet markers, etc.
  serial.send((uint8_t*)tmp, 9);
}

typedef struct temp_report_ {
   float temp;
} temp_report;

/**
 */
int main(void)
{
  watchdogSetup();
  init();
  initVariant();

  serial.setPacketHandler(&onPacket, 0);
  serial.begin(115200);

  delay(1);
  pinMode(13, OUTPUT);
  long now = millis();
  while (1) {
    float treal = fixtemp + (( trans * temperatur() ) - offset ) / factor;
    float ctr = 123412341234.0;
    while (millis() - now < 1000){
      ctr /= 12341234123.2341523;
    }
    now = millis();

    temp_report report;
    report.temp = treal;

    serial.send((uint8_t*)&report, sizeof(temp_report));
    serial.update();
  }
}
