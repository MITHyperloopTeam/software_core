/** 

Tries to talk on Serial1 and Serial2 to Vectornav IMUs,
forwarding readings from them back over LCM.
**/

#define ARDUINO_MAIN
#include "Arduino.h"

#include "arduino_simul_zcm_helpers.hpp"
#include "arduino_time_manager.h"
#include "arduino_warning_sender.h"

#include "zcmgen_c/mithl_vectorXf_t.h"

#include <stdint.h>

#include "IMUParse.h"

// For simulation this must be defined
char MODULE_NAME[] = "IMU_PUBLISHER";


void initVariant() __attribute__((weak));
void initVariant() {}

const double desired_publish_period = 0.1;

// global zcm object
zcm_t * zcm;

// IMU serial streams
HardwareSerial * imu_1_serial;
HardwareSerial * imu_2_serial;

static inline void publish_vectorxf(long long now, float * buf, int N, char * channel){
  mithl_vectorXf_t msg;
  msg.utime = now;
  msg.rows = N;
  msg.data = buf;
  mithl_vectorXf_t_publish(zcm, channel, &msg);
}

/**
 */
int main(void)
{
  extern zcm_t * zcm; 

  // If you change a serial port here, remember to change
  // the corresponding RS232 shutdown pin below.
  imu_1_serial = &Serial1;
  imu_2_serial = &Serial2;

  watchdogSetup();
  init();
  initVariant();
  
  ARDUINO_SIMUL_CREATE_ZCM(zcm, 0);
  
  // set up time manager
  init_time_manager(zcm);

  ARDUINO_SIMUL_ZCM_START(zcm)

  // setup imu serial stuff on Serial 1 and 2
  pinMode(22, OUTPUT);
  pinMode(24, OUTPUT);
  delay(1);
  digitalWrite(22, HIGH);
  digitalWrite(24, HIGH);
  delay(10);
  
  imu_1_serial->begin(115200);
  imu_2_serial->begin(115200);
  IMUParse imu_1_parser;
  IMUParse imu_2_parser;
  delay(10);

  // kick off imu reporting
  char cmd[17] = "$VNWRG,06,0*XX\r\n"; // turn off IMU ASCII output
  imu_1_serial->write(cmd, 16);
  imu_2_serial->write(cmd, 16);
  //serial->write(cmd, 16);
  delay(10);
  char cmd2[27] = "$VNWRG,75,1,4,01,0028*XX\r\n";  // turn on IMU binary output
  imu_1_serial->write(cmd2, 26);    
  imu_2_serial->write(cmd2, 26);    
  delay(10);

  float imu_1_data[6];
  float imu_2_data[6];
  for (int i=0; i < 6; i++){
    imu_1_data[i] = 0.0;
    imu_2_data[i] = 0.0;
  }

  // set up blinking LED to tell if we're still alive
  pinMode(13, OUTPUT);
  double last_led_switch = get_current_time();
  double last_publish = get_current_time();
  double last_heard_from_imu_1 = get_current_time();
  double last_heard_from_imu_2 = get_current_time();
  int blinkingLEDState = HIGH;

  while (1) {
    // ZCM handle and blinking boilerplate
    ARDUINO_SIMUL_ZCM_HANDLE(zcm);
    if (get_current_time() - last_led_switch > 0.5){
      if (blinkingLEDState == HIGH) blinkingLEDState = LOW;
      else if (blinkingLEDState == LOW) blinkingLEDState = HIGH;
      digitalWrite(13, blinkingLEDState);
      last_led_switch = get_current_time();
    }

    // Periodically publish known imu values on two channels
    if (get_current_time() - last_publish > desired_publish_period){
      long long now = get_current_time() * 1000 * 1000;

      publish_vectorxf(now, imu_1_data, 6, "_IMU_1_TEST");
      publish_vectorxf(now, imu_2_data, 6, "_IMU_2_TEST");

      last_publish = get_current_time();
    }

    // TODO(gizatt): This would be cleaner as a function. Lots of code repeat.
    // Eh, it would break some scope stuff and be an ugly function sig
    while(imu_1_serial->available() > 0) {
      imu_1_parser.new_byte(imu_1_serial->read());
    }
    if(imu_1_parser.ready()){
      imu_1_parser.parse(imu_1_data, 6);
      last_heard_from_imu_1 = get_current_time();
    }
    if (get_current_time() - last_heard_from_imu_1 > 0.1){
      imu_1_serial->write(cmd, 16);
      delay(10);
      imu_1_serial->write(cmd2, 26);     
      delay(10);
      last_heard_from_imu_1 = get_current_time();
    }

    while(imu_2_serial->available() > 0) {
      imu_2_parser.new_byte(imu_2_serial->read());
    }
    if(imu_2_parser.ready()){
      imu_2_parser.parse(imu_2_data, 6);
      last_heard_from_imu_2 = get_current_time();
    }
    if (get_current_time() - last_heard_from_imu_2 > 0.1){
      imu_2_serial->write(cmd, 16);
      delay(10);
      imu_2_serial->write(cmd2, 26);     
      delay(10);
      last_heard_from_imu_2 = get_current_time();
    }

    //delay(1); // yield to  make simulation nicer
  }
}