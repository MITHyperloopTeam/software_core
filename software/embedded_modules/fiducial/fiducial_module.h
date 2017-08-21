#ifndef FIDUCIAL_MODULE_H
#define FIDUCIAL_MODULE_H

#include "Arduino.h"

#include "arduino_simul_zcm_helpers.hpp"
#include "arduino_fake_mutex.hpp"
#include "arduino_time_manager.h"

#include "zcmgen_c/mithl_trigger_t.h"
#include "zcmgen_c/mithl_fiducial_t.h"
#include "zcmgen_c/mithl_fiducial_data_t.h"

#include <stdint.h>

#define FIDUCIAL_PIN 5


void initVariant() __attribute__((weak));
void initVariant() {}

Mutex fiducial_mtx;

volatile double last_fiducial_time;
volatile double average_time_between;
volatile int total_count;
volatile int recent_fiducial;

// global zcm object
zcm_t * zcm;

Stream * serial;
mithl_fiducial_data_t fiducial_data;

// publish readings setup
int enable_read_and_publish = 0;
double last_read_time;
double read_interval = 0.05;

void fiducial_seen() {
  fiducial_mtx.lock();
  double currtime = get_current_time();
  double elapsed = currtime - last_fiducial_time;
  last_fiducial_time = currtime;
  average_time_between = average_time_between*0.95 + elapsed*0.05;
  total_count++;
  fiducial_mtx.unlock();
}

void fiducial_read() {

  char rqst[8];
  rqst[0] = 0x55;
  rqst[1] = 8;
  rqst[2] = 0;
  rqst[3] = 0;
  rqst[4] = 0;
  rqst[5] = 0;
  rqst[6] = 170;
  rqst[7] = 118;
  Serial1.write(rqst, 8);
  //serial->write(rqst, 8);

  delay(2);

  int buf_size = 36;
  char buf[buf_size];
  int buf_ptr = 0;
  while(Serial1.available()) {
  //while(serial->available()) {
    if(buf_ptr >= buf_size) {
      printf("fiducial data read overflow\n");
      break;
    }
    buf[buf_ptr] = Serial1.read();
    //buf[buf_ptr] = serial->read();
    buf_ptr++;
  }

  // TODO: check checksum

  fiducial_data.red = (((fiducial_data.red << 16) | buf[8]) << 8) | buf[9];
  fiducial_data.green = (((fiducial_data.red << 16) | buf[10]) << 8) | buf[11];
  fiducial_data.blue = (((fiducial_data.red << 16) | buf[12]) << 8) | buf[13];
  fiducial_data.x_s = (((fiducial_data.red << 16) | buf[14]) << 8) | buf[15];
  fiducial_data.y_i = (((fiducial_data.red << 16) | buf[16]) << 8) | buf[17];
  fiducial_data.int_m = (((fiducial_data.red << 16) | buf[18]) << 8) | buf[19];
  fiducial_data.delta_c = (((fiducial_data.red << 16) | buf[20]) << 8) | buf[21];
  fiducial_data.c_no = (((fiducial_data.red << 16) | buf[22]) << 8) | buf[23];
  fiducial_data.group = (((fiducial_data.red << 16) | buf[24]) << 8) | buf[25];
  fiducial_data.trig = (((fiducial_data.red << 16) | buf[26]) << 8) | buf[27];
}

#endif // FIDUCIAL_MODULE_H