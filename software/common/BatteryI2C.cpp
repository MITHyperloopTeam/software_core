#include "BatteryI2C.h"

#ifdef COMPILE_FOR_ARDUINO
#include "../../externals/arduino_libraries/Wire/Wire.h"
#endif

#define THEWIRE Wire

static inline uint8_t getByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t * b){
  #ifdef COMPILE_FOR_ARDUINO
    uint8_t ret;
    THEWIRE.beginTransmission(dev_addr);
    THEWIRE.write(reg_addr);
    ret= THEWIRE.endTransmission(false);

    if (ret == 0){
        THEWIRE.requestFrom(dev_addr, (uint8_t) 1);
        while (THEWIRE.available())
	    *b = THEWIRE.read();
    }
    return ret;
  #else
    return -1;
  #endif
}

static inline uint8_t getShort(uint8_t dev_addr, uint8_t reg_addr, uint16_t * s){
  #ifdef COMPILE_FOR_ARDUINO
    uint8_t ret;
    THEWIRE.beginTransmission(dev_addr);
    THEWIRE.write(reg_addr);
    ret= THEWIRE.endTransmission(false);

    if (ret == 0){
      THEWIRE.requestFrom(dev_addr, (uint8_t) 2);
      uint8_t low, high;
      high = THEWIRE.read();
      while (THEWIRE.available()){
          low = high;
          high = THEWIRE.read();
      }
      *s = ((uint16_t)low) + (((uint16_t)high) << 8);
    }
    return ret;
  #else
    return -1;
  #endif
}

BatteryI2C::BatteryI2C(zcm_t * zcm, uint8_t battery_address) {
  #ifdef COMPILE_FOR_ARDUINO
    THEWIRE.begin();
    THEWIRE.setClock(40000L);
  #endif
  num_cells_ = -1;
  comms_lock_ = false;
  latest_current_ = -1.0;
  latest_pack_voltage_ = -1.0;

  battery_address_ = battery_address;
  latest_receive_time_ = 0;
  latest_attempt_time_ = 0;
  zcm_ = zcm;
}

#define SDAPIN 20
#define CLKPIN 21
void BatteryI2C::update(double t){

  if (!comms_lock_ && t - latest_attempt_time_ > K_WAIT_PERIOD){
    #ifdef COMPILE_FOR_ARDUINO
    // Force BMS I2C reset if it hands
    // (which is does sometimes)
    // https://github.com/esp8266/Arduino/issues/1025
    pinMode(SDAPIN, OUTPUT);//keeping SDA high during recovery
    digitalWrite(SDAPIN, HIGH);
    pinMode(CLKPIN, OUTPUT);
    for (int i = 0; i < 10; i++) { //9nth cycle acts as NACK
      digitalWrite(CLKPIN, HIGH);
      delayMicroseconds(5);
      digitalWrite(CLKPIN, LOW);
      delayMicroseconds(5);
    }

    //a STOP signal (SDA from low to high while CLK is high)
    digitalWrite(SDAPIN, LOW);
    delayMicroseconds(5);
    digitalWrite(CLKPIN, HIGH);
    delayMicroseconds(2);
    digitalWrite(SDAPIN, HIGH);
    delayMicroseconds(2);
    //bus status is now : FREE

    //return to power up mode
    pinMode(SDAPIN, INPUT);
    pinMode(CLKPIN, INPUT);
    delay(20);
    //pins + begin advised in https://github.com/esp8266/Arduino/issues/452
    THEWIRE.begin();
    THEWIRE.setClock(40000L);
    //only pins: no signal on clk and sda
    //only begin: no signal on clk, no signal on sda
    #endif



    latest_attempt_time_ = t;
    // if we haven't heard from the device yet, get basic info
    error_code_ = getByte(battery_address_, K_CELL_NUM_ADDR, &num_cells_);
    num_cells_ = num_cells_ & 0xf;
    if (!error_code_ && num_cells_ != 0xf){
      comms_lock_ = true; 
    } else {
      comms_lock_ = false;
    }
  }
  if (comms_lock_ && (t - latest_receive_time_) > K_READ_PERIOD){
    // current
    uint16_t w;
    error_code_ = getShort(battery_address_, K_CURRENT_ADDR, &w);
    if (!error_code_){
      latest_current_ = -1.0*((float) ((int16_t) w) )*(7.63/1000./1000.)/(0.001); // 7.63 uV LSB, 1 milliohm resistor
    } else {
      comms_lock_ = false;
    }

    // voltage
    latest_pack_voltage_ = 0.0;
    for (int8_t cell_i=0; cell_i < num_cells_; cell_i++){
      error_code_ = getShort(battery_address_, K_CELL_BASE_ADDR + 2*cell_i, &w);
      if (!error_code_){
        // ???
        latest_cell_voltage_[cell_i] = ((float) (((int16_t) w)>>3) ) * 122.0 / 100.0 / 1000.0;
        latest_pack_voltage_ += latest_cell_voltage_[cell_i];
        // TODO(gizatt): some kind of scaling needed here
      } else {
        comms_lock_ = false;
        break;
      }

    }
  
    // temperature    
    error_code_ = getShort(battery_address_, K_INTERNAL_TEMP_ADDR, &w);
    if (!error_code_){
      // 0.61mV LSB, 13 bits total, 
      // 1*C / 2.0976 mV with 0C = 521.409 mV
      latest_internal_temp_ = (((float) (((int16_t) w)>>3) )*(0.61) - 521.409)/2.0976; 
    } else {
      comms_lock_ = false;
    }

    // permenant failure error code
    error_code_ = getByte(battery_address_, K_PF_REG, &battery_pf_error_code_);
    if (error_code_)
      comms_lock_ = false;

    // shutdown reg
    error_code_ = getByte(battery_address_, K_SHUTDOWN_REG, &battery_shutdown_reg_);
 
    if (error_code_)
      comms_lock_ = false;

    if (comms_lock_){
      latest_receive_time_ = t;
    }
  }
}

void BatteryI2C::publish(const char * channel, double t){
  mithl_battery_status_t msg;
  msg.utime = t*1000*1000;
  if (zcm_){
    msg.battery_comms_lock = comms_lock_;
    msg.battery_i2c_error_code = error_code_;
    msg.battery_pf_error_reg = battery_pf_error_code_;
    msg.battery_shutdown_reg = battery_shutdown_reg_;
    msg.last_battery_read_time = latest_receive_time_;
    msg.current = latest_current_;
    msg.internal_temp = latest_internal_temp_;
    msg.pack_voltage = latest_pack_voltage_;
    if (num_cells_ < 0 || num_cells_ > K_MAX_NUM_CELLS)
      msg.num_cells = K_MAX_NUM_CELLS;
    else
      msg.num_cells = num_cells_;
    if (msg.num_cells > 0){
      msg.cell_voltage = latest_cell_voltage_;
    }

    mithl_battery_status_t_publish(zcm_, channel, &msg);
  }
}
