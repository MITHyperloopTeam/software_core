  /** 
**/

#define ARDUINO_MAIN
#include "Arduino.h"

#include "arduino_simul_zcm_helpers.hpp"
#include "zcmgen_c/mithl_vectorXf_t.h"
#include "zcmgen_c/mithl_string_t.h"

#include <stdint.h>


// For simulation this must be defined
char MODULE_NAME[] = "ZCM_COMMS";

const int READ_ENABLE_PIN = 29;
const int WRITE_ENABLE_PIN = 28;

void initVariant() __attribute__((weak));
void initVariant() {}

zcm_t * zcm;

static void str_handler(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_string_t *msg, void *user)
{
  mithl_string_t strmsg = {
    .utime = millis()*1000,
    .data = msg->data
  };
  mithl_string_t_publish(zcm, "STR", &strmsg);
}

/**
 */
int main(void)
{
  watchdogSetup();
  init();
  initVariant();
  
  ARDUINO_SIMUL_CREATE_ZCM(zcm, 0)

  mithl_string_t_subscribe(zcm, "INC", &str_handler, NULL);
  
  ARDUINO_SIMUL_ZCM_START(zcm)

  delay(1);

  //pinMode(READ_ENABLE_PIN, OUTPUT);
  //pinMode(WRITE_ENABLE_PIN, OUTPUT);
  //digitalWrite(READ_ENABLE_PIN, HIGH);
  //digitalWrite(WRITE_ENABLE_PIN, HIGH);

  pinMode(31, OUTPUT);
  pinMode(32, OUTPUT);
  long last_led_switch = millis();
  long last_vector_send = millis();
  int blinkingLEDState = HIGH;
  while (1) {
    // ZCM handle and blinking boilerplate
    ARDUINO_SIMUL_ZCM_HANDLE(zcm)
    if (millis() - last_led_switch > 500){
      if (blinkingLEDState == HIGH) blinkingLEDState = LOW;
      else if (blinkingLEDState == LOW) blinkingLEDState = HIGH;
      digitalWrite(31, blinkingLEDState);
      digitalWrite(32, blinkingLEDState);
      last_led_switch = millis();
    }
    
    if (millis() - last_vector_send >= 1){
      mithl_vectorXf_t vectormsg;
      vectormsg.utime = millis()*1000;
      vectormsg.rows = 3;
      float data[3] = {0, 1, 2};
      vectormsg.data = data;
      mithl_vectorXf_t_publish(zcm, "VEC", &vectormsg);
      last_vector_send = millis();
    }
  }
}
