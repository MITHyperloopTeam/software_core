#define ARDUINO_MAIN
#include "Arduino.h"

#include "arduino_simul_zcm_helpers.hpp"

#include "arduino_time_manager.h"

#include "zcmgen_c/mithl_vectorXf_t.h"

#include "IMUParse.h"

#include <stdint.h>


// For simulation this must be defined
char MODULE_NAME[] = "ANALOG_IMU_DEMO_MODULE";

void initVariant() __attribute__((weak));
void initVariant() {}

const double desired_analog_send_period = 0.033;

// global zcm object
zcm_t * zcm;
Stream * serial;

/**
 */
int main(void)
{
  extern zcm_t * zcm; 

  watchdogSetup();
  init();
  initVariant();

  // ZCM-relevant messaging initialization:
  ARDUINO_SIMUL_CREATE_ZCM(zcm, 0)

  // set up time manager
  init_time_manager(zcm);

  ARDUINO_SIMUL_ZCM_START(zcm)

  delay(1);

  // setup imu serial stuff
  pinMode(22, OUTPUT);
  delay(1);
  digitalWrite(22, HIGH);
  delay(10);
  Serial1.begin(115200);
  //serial_setup(serial, 115200, "Serial1");
  IMUParse imu_parser;
  delay(10);

  #ifdef COMPILE_FOR_ARDUINO
  char cmd[17] = "$VNWRG,06,0*XX\r\n"; // turn off IMU ASCII output

  Serial1.write(cmd, 16);
  //serial->write(cmd, 16);
  delay(10);
  char cmd2[27] = "$VNWRG,75,1,8,01,0028*XX\r\n";  // turn on IMU binary output
  Serial1.write(cmd2, 26);
  #endif
  //serial->write(cmd2, 26);

  // set up blinking LED to tell if we're still alive
  pinMode(13, OUTPUT);
  double last_led_switch = get_current_time();
  double last_analog_send = get_current_time();
  int blinkingLEDState = HIGH;

  delay(1);

  while (1) {
    // ZCM handle and blinking boilerplate
    ARDUINO_SIMUL_ZCM_HANDLE(zcm);
    
    if (get_current_time() - last_led_switch > 0.5){
      last_led_switch = get_current_time();
      if (blinkingLEDState == HIGH) blinkingLEDState = LOW;
      else if (blinkingLEDState == LOW) blinkingLEDState = HIGH;
      digitalWrite(13, blinkingLEDState);
    }

    // check for IMU serial data and put it in the parser
    #ifdef COMPILE_FOR_ARDUINO // triaging -- greg
    if( Serial1 && Serial1.available() > 0) {
    //if(serial->available() > 0) {
      imu_parser.new_byte(Serial1.read());
      //imu_parser.new_byte(serial->read());
    }
    if(imu_parser.ready()){
      float imu_data[6];
      imu_parser.parse(imu_data, 6);

      mithl_vectorXf_t vec_msg;
      vec_msg.utime = 1;
      vec_msg.rows = 6;
      vec_msg.data = imu_data;
      mithl_vectorXf_t_publish(zcm, "IMU", &vec_msg);
    }
    #endif

    // Periodically do analog reads and send the values
    if ((get_current_time() - last_analog_send > desired_analog_send_period) || (last_analog_send > get_current_time())){
      last_analog_send = get_current_time();

      mithl_vectorXf_t vectormsg;
      vectormsg.utime = millis()*1000;
      vectormsg.rows = 11;
      float buf[11];
      for (int i=0; i < 11; i++)
        buf[i] = analogRead(i);
      vectormsg.data = buf;
    
      mithl_vectorXf_t_publish(zcm, "VEC", &vectormsg);
    }
  }
}
