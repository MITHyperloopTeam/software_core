#ifndef BATTERY_I2C_H
#define BATTERY_I2C_H

#include "Arduino.h"
#include "arduino_simul_zcm_helpers.hpp"
#include "zcmgen_c/mithl_battery_status_t.h"

#define K_CURRENT_ADDR 0x54    // strom an adr. 54h
#define K_CELL_BASE_ADDR 0x32  // start of cell voltage registers
#define K_CELL_NUM_ADDR 0x6    // # of cells register
#define K_MAX_NUM_CELLS 100
#define K_INTERNAL_TEMP_ADDR 0x52 // internal temperature sensor
#define K_READ_PERIOD 1.0
#define K_WAIT_PERIOD 5.0
#define K_PF_REG 0x18 // permanent failure register
        // 0: Deadman timer PF expired
        // 1: Cell voltage under extremely low voltage threshold
        // 2: Cell voltage over extremely high voltage thresh
        // 3: MOSFET failure
        // 4: Cell voltage unbalance PF

#define K_SHUTDOWN_REG 0x15
      // 0: Shutdown request reg
      // 1: VLPF shutdown clear reg (write 1 to clear)
      // 2: VHPF shutdown clear reg (write 1 to clear)
      // 3: MFPF clear
      // 4: CUPF clear

class BatteryI2C {

private:
  int8_t battery_address_;
  
  // Thanks German guys on 
  // http://forum.arduino.cc/index.php?topic=156030.0
  // for useful values here and also example code
  // and also James Turner from Electric Rider for sharing
  // similar example code

public:
  BatteryI2C(zcm_t * zcm, uint8_t battery_address); // if in doubt use 0x30
  void update(double t);

  float get_latest_pack_voltage() { return latest_pack_voltage_; }
  float get_latest_current() { return latest_current_; }

  void publish(const char * channel, double t);

  zcm_t * zcm_;

  uint8_t num_cells_;
  double last_receive_time_;
  float latest_pack_voltage_;
  float latest_cell_voltage_[K_MAX_NUM_CELLS];
  float latest_current_;
  float latest_internal_temp_;
  double latest_receive_time_;
  double latest_attempt_time_;
  bool comms_lock_;
  uint8_t error_code_;
  uint8_t battery_pf_error_code_;
  uint8_t battery_shutdown_reg_;
};


#endif // BATTERY_I2C_H
