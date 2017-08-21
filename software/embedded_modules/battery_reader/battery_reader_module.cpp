/** 
**/
// For simulation this must be defined
char MODULE_NAME[] = "BR";
int myid = 55;

#define ARDUINO_MAIN
#include "Arduino.h"

#include "arduino_simul_zcm_helpers.hpp"
#include "arduino_time_manager.h"
#include "arduino_warning_sender.h"
#include "arduino_temperature.h"

#include "../../common/BatteryI2C.h"
#include "net_monitor.hpp"
#include "IMUParse.h"

#include "zcmgen_c/mithl_battery_reader_low_rate_t.h"

#include <stdint.h>

void initVariant() __attribute__((weak));
void initVariant() {}

const double low_rate_period = 0.5;
const double internal_temp_update_period = 0.01;
const double battery_update_period = 1.0;

// global zcm object
zcm_t * zcm0;
//zcm_t * zcm3;

BatteryI2C * battery;

/**
 */
int main(void)
{
  extern zcm_t * zcm0; 
  //extern zcm_t * zcm3;

  watchdogSetup();
  init();
  initVariant();

  
  ARDUINO_SIMUL_CREATE_ZCM(zcm0, 0);
  //ARDUINO_SIMUL_CREATE_ZCM(zcm3, 0);


  // set up time manager
  init_time_manager(zcm0);
  battery = new BatteryI2C(zcm0, 0x30);

  // set up network monitor
  NetMonitor::TransportInfo transports[1];
  transports[0].zcm = zcm0;
  transports[0].transport_name = "UART";
  //transports[1].zcm = zcm3;
  //transports[1].transport_name = "RS485";
  NetMonitor netmon(transports, 1, myid, "battery_reader", &get_current_time);

  ARDUINO_SIMUL_ZCM_START(zcm0)
  //ARDUINO_SIMUL_ZCM_START(zcm3)

  
  // set up blinking LED to tell if we're still alive
  pinMode(31, OUTPUT);
  pinMode(32, OUTPUT);
  double last_led_switch = get_current_time();
  double last_publish_low_rate = get_current_time();
  double last_battery_update = get_current_time();
  double last_update_internal_temp = get_current_time();
  double internal_temp = 30.0;
  int blinkingLEDState = HIGH;


  while (1) {
    // ZCM handle and blinking boilerplate
    ARDUINO_SIMUL_ZCM_HANDLE(zcm0);
    //ARDUINO_SIMUL_ZCM_HANDLE(zcm3);
    if (get_current_time() - last_led_switch > 0.5){
      if (blinkingLEDState == HIGH) blinkingLEDState = LOW;
      else if (blinkingLEDState == LOW) blinkingLEDState = HIGH;
      digitalWrite(31, blinkingLEDState);
      digitalWrite(32, blinkingLEDState);
      last_led_switch = get_current_time();
    }

    if (get_current_time() - last_update_internal_temp > internal_temp_update_period){
      last_update_internal_temp = get_current_time();
      internal_temp = get_arduino_temperature()*0.01 + internal_temp*0.99;
    }

    if (get_current_time() - last_publish_low_rate > low_rate_period){
      last_publish_low_rate = get_current_time();
      mithl_battery_reader_low_rate_t msg;
      msg.utime = get_current_time() * 1000 * 1000;
      msg.internal_temp = internal_temp;
      if (battery){
        battery->publish("_AF_BATTERY", get_current_time());
      }
      mithl_battery_reader_low_rate_t_publish(zcm0, "_BR_OL", &msg);
    }

    if (get_current_time() - last_battery_update > battery_update_period){
      last_battery_update = get_current_time();
      battery->update(get_current_time());
    }

    netmon.update();

    #ifndef COMPILE_FOR_ARDUINO
    delay(1);
    #endif

  }
}
