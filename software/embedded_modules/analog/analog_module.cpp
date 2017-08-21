/** 
**/

#define ARDUINO_MAIN
#include "Arduino.h"

#include "arduino_simul_zcm_helpers.hpp"

#include "zcmgen_c/mithl_vectorXf_t.h"

#include <stdint.h>

// For simulation this must be defined
char MODULE_NAME[] = "AM";

void initVariant() __attribute__((weak));
void initVariant() {}

/*
static void str_handler(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_string_t *msg, void *user)
{
  mithl_string_t strmsg = {
    .utime = millis()*1000,
    .data = msg->data
  };
  mithl_string_t_publish(zcm, "STR", &strmsg);
}
*/

const long unsigned int desired_analog_send_period = 100;
// global zcm object
zcm_t * zcm;

/**
 */
int main(void)
{
  extern zcm_t * zcm; 

  watchdogSetup();
  init();
  initVariant();
  
  ARDUINO_SIMUL_CREATE_ZCM(zcm, 0)

  //mithl_string_t_subscribe(zcm, "INC", &str_handler, NULL);
  
  ARDUINO_SIMUL_ZCM_START(zcm)

  delay(1);

  // set up blinking LED to tell if we're still alive
  pinMode(13, OUTPUT);
  long unsigned int last_led_switch = millis();
  long unsigned int last_vector_send = millis();
  int blinkingLEDState = HIGH;

  while (1) {
    // ZCM handle and blinking boilerplate
    ARDUINO_SIMUL_ZCM_HANDLE(zcm);
    if (millis() - last_led_switch > 500){
      if (blinkingLEDState == HIGH) blinkingLEDState = LOW;
      else if (blinkingLEDState == LOW) blinkingLEDState = HIGH;
      digitalWrite(13, blinkingLEDState);
      last_led_switch = millis();
    }

    // Periodically do analog reads and send the values
    if (millis() - last_vector_send > desired_analog_send_period){
      mithl_vectorXf_t vectormsg;
      vectormsg.utime = millis()*1000;
      vectormsg.rows = 11;
      float buf[11];
      for (int i=0; i < 11; i++)
        buf[i] = analogRead(i);
      vectormsg.data = buf;
    
      mithl_vectorXf_t_publish(zcm, "VEC", &vectormsg);
      last_vector_send = millis();
    }
    delay(1); // yield to  make simulation nicer
  }
}
