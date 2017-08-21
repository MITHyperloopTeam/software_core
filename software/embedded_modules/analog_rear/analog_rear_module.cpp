/** 
**/
// For simulation this must be defined
char MODULE_NAME[] = "AR";
int myid = 51;

#define ARDUINO_MAIN
#include "Arduino.h"

#include "arduino_simul_zcm_helpers.hpp"
#include "arduino_time_manager.h"
#include "arduino_warning_sender.h"
#include "arduino_temperature.h"

#include "../../common/BatteryI2C.h"
#include "net_monitor.hpp"
#include "IMUParse.h"

#include "zcmgen_c/mithl_analog_rear_high_rate_t.h"
#include "zcmgen_c/mithl_analog_rear_medium_rate_t.h"
#include "zcmgen_c/mithl_analog_rear_low_rate_t.h"

#include <stdint.h>

void initVariant() __attribute__((weak));
void initVariant() {}

// pin definitions for front analog module
const int pin_acc_1d_rear = A0;
const int pin_vacuum_sensor_rear = A1;
const int pin_ambient_temp_rear = A5;
const int pin_ski_rear_left_gh = A9;
const int pin_ski_rear_left_pot = A3;
const int pin_ski_rear_right_gh = A11;
const int pin_ski_rear_right_pot = A2;
const int pin_lcm_rear_pot = A4;
const int pin_lcm_rear_gh = A10;

const double high_rate_period = 0.01;
const double medium_rate_period = 0.033;
const double low_rate_period = 0.5;

const double battery_update_period = 1.0;

// global zcm object
zcm_t * zcm0;
//zcm_t * zcm3;

BatteryI2C * battery;

// IMU serial stream
HardwareSerial * imu_left_serial;
HardwareSerial * imu_right_serial;

// 1D Accelerometer
const float acc_1d_mv_per_m_s_s = 81.5;
// Returns 1D accelerometer reading in m/s/s.
static inline float acc_1d_read(int pin, float zero_offset) {
  float reading = ((float)analogRead(pin))*3.3/1024.;
  reading = (reading - zero_offset) * 1000.0 / acc_1d_mv_per_m_s_s;
  return reading;
}

// Gap height
// Returns gap height reading, in mm
static inline float gh_read(int pin) {
  float reading = (((float)analogRead(pin))*3.3/1024. - 0.6)*20.8333+30.0;
  // 4mA*150Ohm  = 600mV = 30mm
  // 20mA*150Ohm = 3000mV = 80mm
  // i.e. 50mm / (2.4V) conversion ratio 20.8333 mm/V
  return reading;
}

// Potentiometer
// Returns potentiometer length
// Need to supply total pot length, in whatever units output should be in
static inline float pot_read(int pin, float length) {
  float reading = (((float)analogRead(pin))*3.3/1024. - 0.6)*(length / 2.4);
  // 4mA*150Ohm = 600mV = 0 mm
  // 20mA*150Ohm = 3000mV = length mm
  return reading;
}

// Vacuum sensor
// Returns pressure in KiloPascals
static inline float vacuum_read(int pin) {
  float reading = 103.4 - (((float)analogRead(pin))*3.3/1024. - 0.6)*43.08333;
  // 4mA*150Ohm = 600 mV = 0 mbar
  // 20mA*150Ohm = 3000 mV = 103400 pascals
  // i.e. 103400 / (2.4V) conversion ratio  43083.33 Pascals / V
  return reading;
}

// Temp sensor
// Returns temperature in C
static inline float temp_read(int pin) {
  float reading = (((float)analogRead(pin))*3.3/1024.) * (1000. / 10.); // 10mV / deg C
  return reading;
}

// Calibration values
float acc_1d_zero_pt = 1.5; 
void do_calibration() {
  acc_1d_zero_pt = ((float)analogRead(pin_acc_1d_rear))*3.3/1024.;
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

  imu_left_serial = &Serial2;
  imu_right_serial = &Serial1;

  
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
  NetMonitor netmon(transports, 1, myid, "analog_rear", &get_current_time);

  ARDUINO_SIMUL_ZCM_START(zcm0)
  //ARDUINO_SIMUL_ZCM_START(zcm3)

  // setup imu serial stuff on Serial 1 and Serial 2
  pinMode(22, OUTPUT);
  delay(1);
  digitalWrite(22, HIGH);
  delay(10);
  pinMode(24, OUTPUT);
  delay(1);
  digitalWrite(24, HIGH);
  delay(10);
  
  imu_left_serial->begin(115200);
  imu_right_serial->begin(115200);
  IMUParse imu_left_parser;
  IMUParse imu_right_parser;
  delay(10);
  // kick off imu reporting
  #ifdef COMPILE_FOR_ARDUINO
    char cmd[17] = "$VNWRG,06,0*XX\r\n"; // turn off IMU ASCII output
    imu_left_serial->write(cmd, 16);
    imu_right_serial->write(cmd, 16);
    //serial->write(cmd, 16);
    delay(10);
      // 75: register 75 = IMU binary output config
    // 1: Serial output port 1 of the IMU
    // 4 Rate divider 4, 800/4 -> 200hz rate
    // 01: Output group select
    // 0028: GroupField1
    // XX: Checksum, no checksum
    char cmd2[27] = "$VNWRG,75,1,4,01,0200*XX\r\n";  // turn on IMU binary output
    imu_left_serial->write(cmd2, 26);    
    imu_right_serial->write(cmd2, 26);    
    delay(10);
  #endif

  float imu_left_data[6];
  float imu_right_data[6];

  // set up blinking LED to tell if we're still alive
  pinMode(31, OUTPUT);
  pinMode(32, OUTPUT);
  double last_led_switch = get_current_time();
  double last_publish_high_rate = get_current_time();
  double last_publish_medium_rate = get_current_time();
  double last_publish_low_rate = get_current_time();
  double last_battery_update = get_current_time();
  double last_heard_from_left_imu = get_current_time();
  double last_heard_from_right_imu = get_current_time();
  int blinkingLEDState = HIGH;

  do_calibration();

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

    if (get_current_time() - last_publish_high_rate > high_rate_period){
      last_publish_high_rate = get_current_time();
      mithl_analog_rear_high_rate_t msg;
      msg.utime = get_current_time() * 1000 * 1000;
      msg.acc_1d_rear = acc_1d_read(pin_acc_1d_rear, acc_1d_zero_pt);   
      msg.ski_left_imu_xdd = imu_left_data[0];
      msg.ski_left_imu_ydd = imu_left_data[1];
      msg.ski_left_imu_zdd = imu_left_data[2];
      msg.ski_left_imu_rd = imu_left_data[3];
      msg.ski_left_imu_pd = imu_left_data[4];
      msg.ski_left_imu_yd = imu_left_data[5];
      msg.ski_right_imu_xdd = imu_right_data[0];
      msg.ski_right_imu_ydd = imu_right_data[1];
      msg.ski_right_imu_zdd = imu_right_data[2];
      msg.ski_right_imu_rd = imu_right_data[3];
      msg.ski_right_imu_pd = imu_right_data[4];
      msg.ski_right_imu_yd = imu_right_data[5];
      mithl_analog_rear_high_rate_t_publish(zcm0, "_AR_OH", &msg);
    }

    if (get_current_time() - last_publish_medium_rate > medium_rate_period){
      last_publish_medium_rate = get_current_time();
      mithl_analog_rear_medium_rate_t msg;
      msg.utime = get_current_time() * 1000 * 1000;
      msg.vacuum_sensor = vacuum_read(pin_vacuum_sensor_rear);
      msg.ski_rear_left_gh = gh_read(pin_ski_rear_left_gh);
      msg.ski_rear_right_gh = gh_read(pin_ski_rear_right_gh);
      msg.lcm_rear_gh = gh_read(pin_lcm_rear_gh);
      msg.ski_rear_left_pot = pot_read(pin_ski_rear_left_pot, 50);
      msg.ski_rear_right_pot = pot_read(pin_ski_rear_right_pot, 50);
      msg.lcm_rear_pot = pot_read(pin_lcm_rear_pot, 50);
      mithl_analog_rear_medium_rate_t_publish(zcm0, "_AR_OM", &msg);
    }

    if (get_current_time() - last_publish_low_rate > low_rate_period){
      last_publish_low_rate = get_current_time();
      mithl_analog_rear_low_rate_t msg;
      msg.utime = get_current_time() * 1000 * 1000;
      msg.ambient_temp = temp_read(pin_ambient_temp_rear);
      msg.internal_temp = get_arduino_temperature();
      if (battery){
        battery->publish("_AR_BATTERY", get_current_time());
      }
      mithl_analog_rear_low_rate_t_publish(zcm0, "_AR_OL", &msg);
    }

    if (get_current_time() - last_battery_update > battery_update_period){
      last_battery_update = get_current_time();
      battery->update(get_current_time());
    }

    // check for IMU serial data and put it in the parser
    #ifdef COMPILE_FOR_ARDUINO // triaging -- greg
    if( imu_left_serial ) {
      while(imu_left_serial->available() > 0) {
        imu_left_parser.new_byte(imu_left_serial->read());
      }
      if(imu_left_parser.ready()){
        imu_left_parser.parse(imu_left_data, 6);
 //       last_publish_high_rate = get_current_time() - high_rate_period*2;
        last_heard_from_left_imu = get_current_time();
      }
      if (get_current_time() - last_heard_from_left_imu > 1.0){
        imu_left_serial->write(cmd, 16);
        delay(10);
        imu_left_serial->write(cmd2, 26);     
        delay(10);
        last_heard_from_left_imu = get_current_time();
      }
    }

    if( imu_right_serial ) {
      while(imu_right_serial->available() > 0) {
        imu_right_parser.new_byte(imu_right_serial->read());
      }
      if(imu_right_parser.ready()){
        imu_right_parser.parse(imu_right_data, 6);
//        last_publish_high_rate = get_current_time() - high_rate_period*2;
        last_heard_from_right_imu = get_current_time();
      }
      if (get_current_time() - last_heard_from_right_imu > 1.0){
        imu_right_serial->write(cmd, 16);
        delay(10);
        imu_right_serial->write(cmd2, 26);     
        delay(10);
        last_heard_from_right_imu = get_current_time();
      }
    }
    #endif

    netmon.update();

    #ifndef COMPILE_FOR_ARDUINO
    delay(1);
    #endif

  }
}
