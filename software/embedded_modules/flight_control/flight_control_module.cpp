#define ARDUINO_MAIN
#include "Arduino.h"

#include "arduino_simul_zcm_helpers.hpp"
#include "arduino_temperature.h"

#include "net_monitor.hpp"

#include "arduino_time_manager.h"

#include "zcmgen_c/mithl_flight_control_high_rate_t.h"
#include "zcmgen_c/mithl_flight_control_low_rate_t.h"
#include "zcmgen_c/mithl_vectorXf_t.h"
#include "zcmgen_c/mithl_floating_base_t.h"
#include "zcmgen_c/mithl_config_t.h"
#include "zcmgen_c/mithl_state_t.h"

#include "arduino_imu_manager.h"

#include "StateEstimator.h"

#include <stdint.h>
#include <stdio.h>

// For simulation this must be defined
char MODULE_NAME[] = "FC";
int myid = 53;

void initVariant() __attribute__((weak));
void initVariant() {}

const double high_rate_period = 0.01;
const double low_rate_period = 0.5;

// global zcm object
zcm_t * zcm0;
//zcm_t * zcm3;

//BrakeController * bc;
StateEstimator * se;
Stream * serial;

mithl_state_t last_state;
bool have_state = false;
static void state_handler(const zcm_recv_buf_t * rbuf, const char * channel, 
                           const mithl_state_t * msg, void * user){
  memcpy(&last_state, msg, sizeof(mithl_state_t));
}


/**
 */
int main(void)
{
  extern zcm_t * zcm0; 
  //extern zcm_t * zcm3;

  watchdogSetup();
  init();
  initVariant();

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
  NetMonitor netmon(transports, 1, myid, "flight_control", &get_current_time);

  ARDUINO_SIMUL_ZCM_START(zcm0)
  //ARDUINO_SIMUL_ZCM_START(zcm3)

  delay(1);

  // setup imu serial stuff
  pinMode(22, OUTPUT);
  delay(1);
  digitalWrite(22, HIGH);
  delay(10);

  IMUManager front_nav_imu(&Serial2, zcm0, "SIM_FC_IMU_F");
  IMUManager rear_nav_imu(&Serial1, zcm0, "SIM_FC_IMU_R");

  mithl_state_estimator_particle x0;
  x0.id = 0;
  x0.weight = 1.0;
  for (int i=0; i<3; i++){
    x0.mu[i] = 0.0;
    for (int j=0; j<3; j++){
      x0.Sigma[i][j] = 0.0;
    }
  }
  x0.Sigma[0][0] = 0.0;
  x0.Sigma[1][1] = 0.0;
  x0.Sigma[2][2] = 0.0;
  se = new StateEstimator(x0, 10, &front_nav_imu, &rear_nav_imu, zcm0);

  delay(10);
  
  // set up blinking LED to tell if we're still alive
  pinMode(31, OUTPUT);
  pinMode(32, OUTPUT);
  double last_led_switch = get_current_time();
  double last_command_send = get_current_time();
  double last_pf_send = get_current_time();
  double last_se_update = get_current_time();
  double last_published_high_rate = get_current_time();
  double last_published_low_rate = get_current_time();
  double last_heard_from_imu_front = get_current_time();
  double last_heard_from_imu_rear = get_current_time();
  int blinkingLEDState = HIGH;

  mithl_state_t_subscribe(zcm0, "FSM_STATE", &state_handler, NULL);
  delay(1);

  double velocity_est = 0.0;
  double position_est = 0.0;
  bool pod_stopped = false;

  while (1) {
    // ZCM handle and blinking boilerplate
    ARDUINO_SIMUL_ZCM_HANDLE(zcm0);
    //ARDUINO_SIMUL_ZCM_HANDLE(zcm3);
    
    if (get_current_time() - last_led_switch > 0.5){
      last_led_switch = get_current_time();
      if (blinkingLEDState == HIGH) blinkingLEDState = LOW;
      else if (blinkingLEDState == LOW) blinkingLEDState = HIGH;
      digitalWrite(31, blinkingLEDState);
      digitalWrite(32, blinkingLEDState);
    }

    if (last_state.currentState == MITHL_STATE_T_ARMING){
      se->reset(x0);
    }
    se->update(get_current_time());

    front_nav_imu.update();
    rear_nav_imu.update();

    if (get_current_time() - last_published_high_rate > high_rate_period){
      last_published_high_rate = get_current_time();
      mithl_flight_control_high_rate_t msg;
      msg.utime = get_current_time() * 1000 * 1000;
      msg.stopped = se->get_stopped();
      msg.front_nav_imu_xdd = front_nav_imu.imu_data_[0];
      msg.front_nav_imu_ydd = front_nav_imu.imu_data_[1];
      msg.front_nav_imu_zdd = front_nav_imu.imu_data_[2];
      msg.front_nav_imu_rd = front_nav_imu.imu_data_[3];
      msg.front_nav_imu_pd = front_nav_imu.imu_data_[4];
      msg.front_nav_imu_yd = front_nav_imu.imu_data_[5];
      msg.rear_nav_imu_xdd = rear_nav_imu.imu_data_[0];
      msg.rear_nav_imu_ydd = rear_nav_imu.imu_data_[1];
      msg.rear_nav_imu_zdd = rear_nav_imu.imu_data_[2];
      msg.rear_nav_imu_rd = rear_nav_imu.imu_data_[3];
      msg.rear_nav_imu_pd = rear_nav_imu.imu_data_[4];
      msg.rear_nav_imu_yd = rear_nav_imu.imu_data_[5];
      mithl_flight_control_high_rate_t_publish(zcm0, "_FC_OH", &msg);
    }

    if (get_current_time() - last_published_low_rate > low_rate_period){
      last_published_low_rate = get_current_time();
      mithl_flight_control_low_rate_t msg;
      msg.utime = get_current_time() * 1000 * 1000;

      se->populate_mean_and_vars(msg.front_imu_mean, msg.rear_imu_mean,
          msg.front_imu_variance, msg.rear_imu_variance);

      msg.internal_temp = get_arduino_temperature();
      mithl_flight_control_low_rate_t_publish(zcm0, "_FC_OL", &msg);
    }

    netmon.update();

    #ifndef COMPILE_FOR_ARDUINO
    delay(1);
    #endif
  }
}
