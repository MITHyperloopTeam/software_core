/** 
**/

#define ARDUINO_MAIN
#include "Arduino.h"

#include "arduino_simul_zcm_helpers.hpp"

#include "net_monitor.hpp"

#include "arduino_time_manager.h"

#include <stdint.h>

// For simulation this must be defined
char MODULE_NAME[] = "NET_DIAG";

void initVariant() __attribute__((weak));
void initVariant() {}

// invoke MAKE with special extra definition'
// to change the NETMON ID of this node:
// make EXTRA_DEFINE_1="-DNETMON_ID=<new # here>"
#ifdef NETMON_ID
  int myid = NETMON_ID;
#else
  int myid = 11;
#endif

// global zcm object
zcm_t * zcm0;
zcm_t * zcm3;

/**
 */
int main(void)
{
  extern zcm_t * zcm0; 
  extern zcm_t * zcm3; 

  watchdogSetup();
  init();
  initVariant();

  ARDUINO_SIMUL_CREATE_ZCM(zcm0, 0)
  ARDUINO_SIMUL_CREATE_ZCM(zcm3, 3)



  NetMonitor::TransportInfo transports[1];
  transports[0].zcm = zcm0;
  transports[0].transport_name = "UART";
  //transports[1].zcm = zcm3;
  //transports[1].transport_name = "RS485";

  NetMonitor netmon(transports, 1, myid, "NetDiagNode", &get_current_time);

  ARDUINO_SIMUL_ZCM_START(zcm0)
  ARDUINO_SIMUL_ZCM_START(zcm3)

  delay(1);

  // set up blinking LED to tell if we're still alive
  pinMode(13, OUTPUT);
  pinMode(31, OUTPUT);
  pinMode(32, OUTPUT);
  long unsigned int last_led_switch = millis();
  int blinkingLEDState = HIGH;

  while (1) {
    // ZCM handle and blinking boilerplate
    ARDUINO_SIMUL_ZCM_HANDLE(zcm0);
    ARDUINO_SIMUL_ZCM_HANDLE(zcm3);
    if (millis() - last_led_switch > 500){
      if (blinkingLEDState == HIGH) blinkingLEDState = LOW;
      else if (blinkingLEDState == LOW) blinkingLEDState = HIGH;
      digitalWrite(13, blinkingLEDState);
      digitalWrite(31, blinkingLEDState);
      digitalWrite(32, blinkingLEDState);
      last_led_switch = millis();
    }

    netmon.update();
  }
}
