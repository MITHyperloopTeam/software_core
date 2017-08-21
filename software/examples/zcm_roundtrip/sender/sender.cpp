/** 
**/

#define ARDUINO_MAIN
#include "Arduino.h"

#include "arduino_simul_zcm_helpers.hpp"

#include "zcmgen_c/mithl_string_t.h"

#include <stdint.h>


void initVariant() __attribute__((weak));
void initVariant() {}

// global zcm object
zcm_t * zcm;
zcm_t * zcm1;

long unsigned int last_led_switch;

// receive string, send it off on the Serial1 device
static void str_handler(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_string_t *msg, void *user)
{
  mithl_string_t strmsg = {
    .utime = millis()*1000,
    .data = msg->data
  };
  mithl_string_t_publish(zcm1, "STR", &strmsg);
  last_led_switch = 0;
}

const int READ_ENABLE_PIN = 29;
const int WRITE_ENABLE_PIN = 28;

/**
 */
int main(void)
{
  watchdogSetup();
  init();
  initVariant();
  
  ARDUINO_SIMUL_CREATE_ZCM(zcm, 0)
  ARDUINO_SIMUL_CREATE_ZCM(zcm1, 2)

  mithl_string_t_subscribe(zcm, "STR_SEND", &str_handler, NULL);
  
  ARDUINO_SIMUL_ZCM_START(zcm)
  ARDUINO_SIMUL_ZCM_START(zcm1)

  delay(1);

  // set up blinking LED to tell if we're still alive
  pinMode(13, OUTPUT);
  last_led_switch = millis();
  int blinkingLEDState = HIGH;

  pinMode(READ_ENABLE_PIN, OUTPUT);  
  pinMode(WRITE_ENABLE_PIN, OUTPUT);
  digitalWrite(READ_ENABLE_PIN, HIGH);
  digitalWrite(WRITE_ENABLE_PIN, HIGH);

  while (1) {
    // ZCM handle and blinking boilerplate
    ARDUINO_SIMUL_ZCM_HANDLE(zcm)
    ARDUINO_SIMUL_ZCM_HANDLE(zcm1)
    if (millis() - last_led_switch > 1000){
      if (blinkingLEDState == HIGH) blinkingLEDState = LOW;
      else if (blinkingLEDState == LOW) blinkingLEDState = HIGH;
      digitalWrite(13, blinkingLEDState);
      last_led_switch = millis();
    }
  }
}
