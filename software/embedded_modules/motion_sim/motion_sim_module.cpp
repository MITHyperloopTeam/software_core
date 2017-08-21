/** 
**/
// For simulation this must be defined
char MODULE_NAME[] = "MOTION_SIM";
int myid = 10;

#define ARDUINO_MAIN
#include "Arduino.h"

#include "arduino_simul_zcm_helpers.hpp"
#include "arduino_time_manager.h"
#include "arduino_warning_sender.h"
#include "arduino_temperature.h"

#include "net_monitor.hpp"

#include "zcmgen_c/mithl_vectorXf_t.h"
#include <stdint.h>

// pin definitions for front analog module
const int pin_output_enable = 7;
const uint8_t _i2caddr = 0x40;
const int kPulseNinety = 400;
const int kPulseMinusNinety = 200;

void initVariant() __attribute__((weak));
void initVariant() {}

const double command_period = 0.1;

#define PCA9685_SUBADR1 0x2
#define PCA9685_SUBADR2 0x3
#define PCA9685_SUBADR3 0x4

#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE

#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9

#define ALLLED_ON_L 0xFA
#define ALLLED_ON_H 0xFB
#define ALLLED_OFF_L 0xFC
#define ALLLED_OFF_H 0xFD

// global zcm object
zcm_t * zcm0;
//zcm_t * zcm3;
double commanded_roll = 0.0;
double commanded_pitch = 0.0;
static void command_handler(const zcm_recv_buf_t * rbuf, const char * channel, 
                           const mithl_vectorXf_t * msg, void * user){
  if (msg->rows >= 2){
    commanded_roll = fmax(fmin(msg->data[0], 90.), -90.);
    commanded_pitch = fmax(fmin(msg->data[1], 90.), -90.);
    
  }
}

#ifdef COMPILE_FOR_ARDUINO
#include "../../externals/arduino_libraries/Wire/Wire.h"
#endif

void write8(uint8_t addr, uint8_t d) {
  #ifdef COMPILE_FOR_ARDUINO
  Wire.beginTransmission(_i2caddr);
  Wire.write(addr);
  Wire.write(d);
  Wire.endTransmission();
  #endif
}
uint8_t read8(uint8_t addr) {
  #ifdef COMPILE_FOR_ARDUINO
  Wire.beginTransmission(_i2caddr);
  Wire.write(addr);
  Wire.endTransmission();

  Wire.requestFrom((uint8_t)_i2caddr, (uint8_t)1);
  return Wire.read();
  #else
  return 0;
  #endif
}

void setPWMFreq(float freq) {
  //Serial.print("Attempting to set freq ");
  //Serial.println(freq);
  freq *= 0.9;  // Correct for overshoot in the frequency setting (see issue #11).
  float prescaleval = 25000000;
  prescaleval /= 4096;
  prescaleval /= freq;
  prescaleval -= 1;
  uint8_t prescale = floor(prescaleval + 0.5);
  
  uint8_t oldmode = read8(PCA9685_MODE1);
  uint8_t newmode = (oldmode&0x7F) | 0x10; // sleep
  write8(PCA9685_MODE1, newmode); // go to sleep
  write8(PCA9685_PRESCALE, prescale); // set the prescaler
  write8(PCA9685_MODE1, oldmode);
  delay(5);
  write8(PCA9685_MODE1, oldmode | 0xa1);  //  This sets the MODE1 register to turn on auto increment.
                                          // This is why the beginTransmission below was not working.
  //  Serial.print("Mode now 0x"); Serial.println(read8(PCA9685_MODE1), HEX);
}
void setPWM(uint8_t num, uint16_t on, uint16_t off) {
  //Serial.print("Setting PWM "); Serial.print(num); Serial.print(": "); Serial.print(on); Serial.print("->"); Serial.println(off);
  #ifdef COMPILE_FOR_ARDUINO
  Wire.beginTransmission(_i2caddr);
  Wire.write(LED0_ON_L+4*num);
  Wire.write(on);
  Wire.write(on>>8);
  Wire.write(off);
  Wire.write(off>>8);
  Wire.endTransmission();
  #endif
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
  
  ARDUINO_SIMUL_CREATE_ZCM(zcm0, 0);
  //ARDUINO_SIMUL_CREATE_ZCM(zcm3, 0);
  
  // set up time manager
  init_time_manager(zcm0);

  #ifdef COMPILE_FOR_ARDUINO
  Wire.begin();
  write8(PCA9685_MODE1, 0x0);
  #endif
  setPWMFreq(50);

  // set up network monitor
  NetMonitor::TransportInfo transports[1];
  transports[0].zcm = zcm0;
  transports[0].transport_name = "UART";
  //transports[1].zcm = zcm3;
  //transports[1].transport_name = "RS485";
  NetMonitor netmon(transports, 1, myid, "motion_sim", &get_current_time);


  ARDUINO_SIMUL_ZCM_START(zcm0)
  //ARDUINO_SIMUL_ZCM_START(zcm3)


  mithl_vectorXf_t_subscribe(zcm0, "MOTION_SIM_CMD", &command_handler, NULL);

  // setup imu serial stuff on Serial 1 for the brake IMU
  pinMode(pin_output_enable, OUTPUT);
  delay(1);
  digitalWrite(pin_output_enable, HIGH);
  delay(10);
  
  // set up blinking LED to tell if we're still alive
  pinMode(31, OUTPUT);
  pinMode(32, OUTPUT);
  double last_command_push = get_current_time();
  double last_led_switch = get_current_time();
  int blinkingLEDState = HIGH;

  double last_command_roll = 0.0;
  double last_command_pitch = 0.0;
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

    if (get_current_time() - last_command_push >= command_period){
      last_command_push = get_current_time();

      // normalize commands to within 0, 1
      if (fabs(last_command_roll -commanded_roll) > 0.1){
        double roll_norm = (commanded_roll + 90) / 180.0;
        last_command_roll = commanded_roll;
        setPWM(0, 0, (uint16_t)(roll_norm*kPulseNinety + (1.-roll_norm)*kPulseMinusNinety));
      }
      double pitch_norm = (commanded_pitch + 90) / 180.0;      
      if (fabs(last_command_pitch - commanded_pitch) > 0.1){
        double pitch_norm = (commanded_pitch + 90) / 180.0;
        last_command_pitch = commanded_pitch;
        setPWM(1, 0, (uint16_t)(pitch_norm*kPulseNinety + (1.-pitch_norm)*kPulseMinusNinety));
      }
    }

    netmon.update();
    //delay(1); // yield to  make simulation nicer
  }
}
