/** 
**/

#define ARDUINO_MAIN
#include "Arduino.h"

#include "arduino_simul_zcm_helpers.hpp"
#include "arduino_fake_mutex.hpp"
#include "arduino_time_manager.h"
#include "arduino_temperature.h"

#include "net_monitor.hpp"

#include "zcmgen_c/mithl_trigger_t.h"
#include "zcmgen_c/mithl_fiducial_t.h"
#include "zcmgen_c/mithl_fiducial_low_rate_t.h"

#include "FiducialController.h"

#include <stdint.h>

#define FIDUCIAL_PIN 49
#define FIDUCIAL_REPORT_PERIOD 0.025

// For simulation this must be defined
char MODULE_NAME[] = "FD";
int myid = 52;


const double fiducal_report_period = 0.05;
const double low_rate_period = 0.5;
const double fiducial_update_period = 0.01;

void initVariant() __attribute__((weak));
void initVariant() {}

Mutex fiducial_mtx;

volatile double last_fiducial_state_time[2];
volatile double average_time_before[2];
volatile int total_count[2];
volatile int recent_fiducial;

// global zcm object
zcm_t * zcm0;
//zcm_t * zcm3;

void fiducial_seen() {
  //fiducial_mtx.lock();
  int fiducial_index = 0;
  if (digitalRead(FIDUCIAL_PIN) == HIGH) {
    fiducial_index = 1;
  }
  double currtime1 = get_current_time();
  double currtime2 = get_current_time();
  double currtime = (currtime1 + currtime2)/2;
  double elapsed = currtime - last_fiducial_state_time[1 - fiducial_index];
  last_fiducial_state_time[fiducial_index] = currtime;
  average_time_before[fiducial_index] = elapsed;
  total_count[fiducial_index]++;
  
  //fiducial_mtx.unlock();
}

// DB DB DB
static int flash_now(int blinkingLEDState) {
  digitalWrite(13, blinkingLEDState);
  if (blinkingLEDState == HIGH)
    return LOW;
  return HIGH;
}
static void flash_x_times(int x) {
  int blinkingLEDState = LOW;
  for (int i=0; i<2*x + 1; i++) {
      blinkingLEDState = flash_now(blinkingLEDState);
      delay(150);
  }
}
// /DB /DB /DB

/**
 */
int main(void)
{  
  extern zcm_t * zcm0; 
  //extern zcm_t * zcm3;


  watchdogSetup();
  init();
  initVariant();

  // set up blinking LED to tell if we're still alive
  pinMode(31, OUTPUT);
  pinMode(32, OUTPUT);
  double last_led_switch = get_current_time();
  double last_publish_low_rate = get_current_time();
  double last_publish_fiducial_status = get_current_time();
  double last_update_fiducial = get_current_time();
  int blinkingLEDState = HIGH;
  delay(10); //DB
  digitalWrite(31, blinkingLEDState); //DB
  digitalWrite(32, blinkingLEDState); //DB
  
  ARDUINO_SIMUL_CREATE_ZCM(zcm0, 0);
  //ARDUINO_SIMUL_CREATE_ZCM(zcm3, 0);

  // set up time manager
  init_time_manager(zcm0);
  
  // set up network monitor
  NetMonitor::TransportInfo transports[1];
  transports[0].zcm = zcm0;
  transports[0].transport_name = "UART";
  //transports[1].zcm = zcm3;
  //transports[1].transport_name = "RS485";
  NetMonitor netmon(transports, 1, myid, "fiducial", &get_current_time);

  ARDUINO_SIMUL_ZCM_START(zcm0)
  //ARDUINO_SIMUL_ZCM_START(zcm3)

  delay(1);

  // setup imu serial stuff
  pinMode(22, OUTPUT);
  delay(1);
  digitalWrite(22, HIGH);
  delay(10);
  //Serial1.begin(115200); // From old imuparse code; different paradigm here


  // Initialize desired settings
  mithl_fiducial_config_t default_config;

  default_config.utime = 0;
  
  default_config.power =        1000; // 500/1000 Power
  default_config.powermode =    0;   // 0 = STATIC
  default_config.average =      1;  // 16 averaging window size (must be pwr of 2)
  default_config.evalmode =     1;//2;   // 0 = FIRST HIT, 1 = BEST HIT, 2 = MINDIST
  default_config.hold =         0;   // hold ERROR signal for 10 ms
  default_config.intlim =       0;   // 0/4095 (lower intensity); intensity of sensed light must be above this
  default_config.maxcolno =     1;   //  colors in teach table
  default_config.outmode =      0;   // 1 = BINARY, 0 = direct-HI (5 colors max)
  default_config.trigger =      0;   // 0 = CONT, 6 = PARA (use multiple SET NO) trigger mode
  default_config.exteach =      0; // 1;   // 0=OFF (Depr: 1 = ON, can teach either through input at IN0 or teach button being pressed)
  default_config.calcmode =     0;   // 0 = X/Y INT, threshold intensity differently from r/g-color
  default_config.dyn_win_lo =   800; // 800/1000 lo limit for dynamic power
  default_config.dyn_win_hi =   1000; // 1000/1000 hi limit for dynamic power
  default_config.color_groups = 0;   // 0 = color groups OFF
  default_config.ledmode =      0;  // 0 = DC
  default_config.gain =         6;  // 3/8, GAIN chooses amplifier to use on signals
  default_config.integral =     2;  // integration window

  default_config.strip_width =  0.0981; // strip width (m)
  default_config.strip_space_between = 30.48; // space from strip to successive strip

  mithl_fiducial_teach_table_t default_teach_table;
  default_teach_table.utime = 0;

  // First, clear all the rows of the default teach table
  for (int i=0; i<31; i++) {
    mithl_fiducial_color_row_t row;
    
    row.utime =      0;
    row.color_x =    0;
    row.color_y =    0;
    row.cto =        0;
    row.color_int =  0;
    row.ito =        0;
    row.group =      0;
    row.hold =       0;
    
    default_teach_table.rows[i] = row;
  }
  
//  Masking

//  2100
//  1800
//  70


//  Purple

//  1600
//  1300
//  280


  // Next, update the rows we care about
 /* {
    mithl_fiducial_color_row_t row_tape;

    row_tape.utime =      0;
    row_tape.color_x =    1650;// 2100; // 3500; // 2438;  // X==red-ness
    row_tape.color_y =    1480; // 1800; // 700; //1650;  // Y==green-ness
    row_tape.color_int =  440; // two: 440 or 1320 // // 70;   // INTensity
    row_tape.cto =        350;  // color tolerance (euclidean distance threshold on XY)
    row_tape.ito =        150;  // intensity tolerance (interval threshold on INT)
    row_tape.group =      0;   // color group; shouldn't matter
    row_tape.hold =       0;   // hold time. Short for fast-observation apps. Trying 0 for now.
    
    default_teach_table.rows[0] = row_tape;
  } */
  /* {
    mithl_fiducial_color_row_t row_tape;

    row_tape.utime =      0;
    row_tape.color_x =    1600; // 1580;  // X==red-ness
    row_tape.color_y =    1600; //1370;  // Y==green-ness
    row_tape.color_int =  2500; // 280; //1323;   // INTensity
    row_tape.cto =        1500;  // color tolerance (euclidean distance threshold on XY)
    row_tape.ito =        1400;  // intensity tolerance (interval threshold on INT)
    row_tape.group =      1;   // color group; shouldn't matter
    row_tape.hold =       0;   // hold time. Short for fast-observation apps. Trying 0 for now.
    
    default_teach_table.rows[0] = row_tape;
  } */
  { // "Tape Color", I hope. Without tape was reading (X,Y,INT)=(0,0,0). With: (1600,1400,500-1000)
    mithl_fiducial_color_row_t row_tape;

    row_tape.utime =      0;
    row_tape.color_x =    1600;  // X==red-ness
    row_tape.color_y =    1600;  // Y==green-ness
    row_tape.color_int =  2500;   // INTensity
    row_tape.cto =        3500;  // color tolerance (euclidean distance threshold on XY)
    row_tape.ito =        750;  // intensity tolerance (interval threshold on INT)
    row_tape.group =      0;   // color group; shouldn't matter
    row_tape.hold =       0;   // hold time. Short for fast-observation apps. Trying 0 for now.
    
    default_teach_table.rows[0] = row_tape;
  }

  FiducialController fiducial_controller(&Serial1, zcm0, &default_config, &default_teach_table);
  delay(10);

  // set up pins and flags
  last_fiducial_state_time[0] = get_current_time();
  last_fiducial_state_time[1] = get_current_time();
  average_time_before[0] = 1;
  average_time_before[1] = 1;
  total_count[0] = 0;
  total_count[1] = 0;
  recent_fiducial = 0;
  // kick it off
  pinMode(FIDUCIAL_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(FIDUCIAL_PIN), fiducial_seen, CHANGE);

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

    // Serial1.write("AAAA\n"); // DB
    // Serial1.flush(); // DB

    // Broadcast fiducial status messages
    if (get_current_time() - last_publish_fiducial_status >= FIDUCIAL_REPORT_PERIOD){
      last_publish_fiducial_status = get_current_time();
      mithl_fiducial_t fiducial_msg;
      double now = get_current_time();
      fiducial_msg.utime = now * 1000 * 1000;

      // assemble msg
      fiducial_mtx.lock();
      int rising_state = 1;

      fiducial_msg.average_time_between = average_time_before[rising_state] + average_time_before[1-rising_state];
      fiducial_msg.average_time_strip = average_time_before[1 - rising_state];
      fiducial_msg.time_since_last = now - last_fiducial_state_time[rising_state];
      fiducial_msg.total_count = total_count[rising_state];
      fiducial_mtx.unlock();

      mithl_fiducial_t_publish(zcm0, "_FD_O", &fiducial_msg);
    }

    if (get_current_time() - last_publish_low_rate >= low_rate_period){
      last_publish_low_rate = get_current_time();
      mithl_fiducial_low_rate_t msg;
      double now = get_current_time();
      msg.utime = now * 1000 * 1000;
      msg.internal_temp = get_arduino_temperature();
      mithl_fiducial_low_rate_t_publish(zcm0, "_FD_OL", &msg);
    
    }
    
    // Allow FiducialController to listen to / talk with device:
    if (get_current_time() - last_update_fiducial > fiducial_update_period){
      last_update_fiducial = get_current_time();
      fiducial_controller.Update(); 
    }

    netmon.update();

    #ifndef COMPILE_FOR_ARDUINO
    delay(1);
    #endif
  }
}
