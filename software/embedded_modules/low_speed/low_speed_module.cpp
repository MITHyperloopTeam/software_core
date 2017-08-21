/** 
**/

#define ARDUINO_MAIN
#include "Arduino.h"

#include "arduino_simul_zcm_helpers.hpp"
#include "arduino_time_manager.h"
#include "arduino_temperature.h"

#include "net_monitor.hpp"

#include "zcmgen_c/mithl_low_speed_low_rate_t.h"

#include "RoboteqADMController.h"

#include "BrakeActuatorController.h"

#include <stdint.h>

#define BAC_UPDATE_PERIOD 0.001
#define RADM_UPDATE_PERIOD 0.05
#define LOW_RATE_STATUS_PERIOD 0.5

// For simulation this must be defined
char MODULE_NAME[] = "LS";
int myid = 54;

void initVariant() __attribute__((weak));
void initVariant() {}

// global zcm object
zcm_t * zcm0;
//zcm_t * zcm3;

BrakeActuatorController * bac;
volatile  long unsigned int last_led_switch; 

static void encoder_tick_seen(void) {
  if (bac != NULL)
    //last_led_switch = millis();
    bac->HandleEncoderTick();
}

/**
 */
int main(void)
{
  extern zcm_t * zcm0; 
  digitalWrite(34, HIGH);
  //extern zcm_t * zcm3;
  watchdogSetup();
  init();
  initVariant();

  pinMode(34, OUTPUT);
  digitalWrite(34, HIGH);

  // ZCM-relevant messaging initialization:
  ARDUINO_SIMUL_CREATE_ZCM(zcm0, 0)
  //ARDUINO_SIMUL_CREATE_ZCM(zcm3, 0)

  // set up time manager
  init_time_manager(zcm0);

  // set up network monitor
  NetMonitor::TransportInfo transports[1];
  transports[0].zcm = zcm0;
  transports[0].transport_name = "UART";
  //transports[1].zcm = zcm3;
  //transports[1].transport_name = "RS485";
  NetMonitor netmon(transports, 1, myid, "low_speed", &get_current_time);

  ARDUINO_SIMUL_ZCM_START(zcm0)
  //ARDUINO_SIMUL_ZCM_START(zcm3)

  delay(1);

  // Create the controller for the brake actuator system
  bac = NULL; // this prevents a race condition of callback getting called before bac assigned
  bac = new BrakeActuatorController(zcm0, encoder_tick_seen);

  // Create a controller for the low-speed clamp
  // on Serial1 using these default values.
  mithl_adm_config_t default_configuration_clamp;
  default_configuration_clamp.estop = true;
  default_configuration_clamp.command = -2;
  default_configuration_clamp.mode = RoboteqADMController::OPEN_LOOP_SPEED;
  default_configuration_clamp.current_lim = 12.0;
  default_configuration_clamp.Kp = 2.0;
  default_configuration_clamp.Kd = 0.0;
  default_configuration_clamp.Ki = 0.0;
  default_configuration_clamp.integral_cap = 0;
  default_configuration_clamp.max_rpm = 10000;
  default_configuration_clamp.max_power_forward = 66;
  default_configuration_clamp.max_power_reverse = 66;
  default_configuration_clamp.n_poles = -2;
  default_configuration_clamp.encoder_usage = RoboteqADMController::ENCODER_USAGE_UNUSED;
  default_configuration_clamp.switching_mode = -2;
  default_configuration_clamp.sinusoidal_mode = -2;
  default_configuration_clamp.encoder_pulse_per_rev = -2;
  default_configuration_clamp.encoder_low_count_limit = -2;
  default_configuration_clamp.encoder_high_count_limit = -2;
  default_configuration_clamp.default_pos = -2;
  default_configuration_clamp.default_vel = -2;
  default_configuration_clamp.default_current = -2;
  default_configuration_clamp.closed_loop_error_detection = RoboteqADMController::CLED_NONE;
  default_configuration_clamp.closed_loop_feedback_sensor = -2;
  RoboteqADMController radm_clamp = RoboteqADMController(&Serial1, zcm0, "CLAMP", -1, &default_configuration_clamp);

  // And also the low speed wheel
  mithl_adm_config_t default_configuration_wheel;
  default_configuration_wheel.estop = true;
  default_configuration_wheel.command = -2;
  default_configuration_wheel.mode = RoboteqADMController::OPEN_LOOP_SPEED;
  default_configuration_wheel.current_lim = 30.0;
  default_configuration_wheel.Kp = 0.2;
  default_configuration_wheel.Kd = 0.0;
  default_configuration_wheel.Ki = 0.0;
  default_configuration_wheel.integral_cap = 50;
  default_configuration_wheel.max_rpm = 5040;
  default_configuration_wheel.max_power_forward = 100;
  default_configuration_wheel.max_power_reverse = 100;
  default_configuration_wheel.n_poles = 8;
  default_configuration_wheel.encoder_usage = RoboteqADMController::ENCODER_USAGE_FEEDBACK;
  default_configuration_wheel.switching_mode = RoboteqADMController::SWITCHING_MODE_HALL;
  default_configuration_wheel.sinusoidal_mode = RoboteqADMController::SINUSOIDAL_MODE_HALL_ENCODER;
  default_configuration_wheel.encoder_pulse_per_rev = 500;
  default_configuration_wheel.encoder_low_count_limit = -100000;
  default_configuration_wheel.encoder_high_count_limit = 100000;
  default_configuration_wheel.default_pos = -2;
  default_configuration_wheel.default_vel = -2;
  default_configuration_wheel.default_current = -2;
  default_configuration_wheel.closed_loop_error_detection = RoboteqADMController::CLED_NONE;
  default_configuration_wheel.closed_loop_feedback_sensor = RoboteqADMController::FEEDBACK_SENSOR_ENCODER;
  RoboteqADMController radm_wheel = RoboteqADMController(&Serial2, zcm0, "WHEEL", A5, &default_configuration_wheel);

  // set up blinking LED to tell if we're still alive
  pinMode(31, OUTPUT);
  pinMode(32, OUTPUT);
  double last_led_switch = get_current_time();
  double last_bac_update = get_current_time();
  double last_radm_update = get_current_time();
  double last_low_rate_status_send = get_current_time();
  int blinkingLEDState = HIGH;

  double led_switch_period = 0.1;

  while (1) {
    // ZCM handle and blinking boilerplate
    for (int i=0; i < 100; i++){
      ARDUINO_SIMUL_ZCM_HANDLE(zcm0);
      //ARDUINO_SIMUL_ZCM_HANDLE(zcm3);
    }


    if (radm_clamp.latest_config.estop <= 0 && radm_wheel.latest_config.estop <= 0 && 
        bac->mode_ == BrakeActuatorController::MODE_ESTOP){
      led_switch_period = 0.5;
    } else {
      led_switch_period = 0.1;
    }

    if (get_current_time() - last_led_switch > led_switch_period){
      if (blinkingLEDState == HIGH){
       blinkingLEDState = LOW;
      }
      else if (blinkingLEDState == LOW) {
        blinkingLEDState = HIGH;
      }

      digitalWrite(31, blinkingLEDState);
      digitalWrite(32, blinkingLEDState);
      last_led_switch = get_current_time();
    }
    
    if (get_current_time() - last_low_rate_status_send > LOW_RATE_STATUS_PERIOD){
      last_low_rate_status_send = get_current_time();
      mithl_low_speed_low_rate_t msg;
      msg.utime = get_current_time() * 1000 * 1000;
      msg.internal_temp = get_arduino_temperature();
      mithl_low_speed_low_rate_t_publish(zcm0, "_LS_OL", &msg);
    }

    // Update all of our controllers
    if (get_current_time() - last_radm_update > RADM_UPDATE_PERIOD){
      last_radm_update = get_current_time();
      radm_clamp.Update();
      radm_wheel.Update();
    }

    if (get_current_time() - last_bac_update > BAC_UPDATE_PERIOD){
      last_bac_update = get_current_time();
      bac->Update();
    }

    netmon.update();

    #ifndef COMPILE_FOR_ARDUINO
    delay(1);
    #endif
  }
}
