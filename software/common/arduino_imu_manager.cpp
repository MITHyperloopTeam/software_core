#include "arduino_time_manager.h"
#include "arduino_imu_manager.h"

void imu_sim_handler_helper(const zcm_recv_buf_t * rbuf, const char * channel, 
                   const mithl_vectorXf_t * msg, void * user){
  ((IMUManager*)user)->imu_sim_handler(msg);
}


IMUManager::IMUManager(HardwareSerial * serial, zcm_t * zcm, char * sim_channel) {
      serial_ = serial;
  number_of_identical_messages_ = 0;
  setup_attempted_ = false;
  serial_->begin(115200);   
  zcm_ = zcm;
  doIMUSetup(sim_channel);
  last_heard_from_imu_ = get_current_time() - IMU_HEALTHY_TIMEOUT*2;
  last_identical_message_check_ = get_current_time();
}

void IMUManager::doIMUSetup(char * sim_channel){
  // For a real Arduino, we have to kick it off   
  #ifdef COMPILE_FOR_ARDUINO
    char cmd[17] = "$VNWRG,06,0*XX\r\n"; // turn off IMU ASCII output
    serial_->write(cmd, 16);
    delay(10);
    // 75: register 75 = IMU binary output config
    // 1: Serial output port 1 of the IMU
    // 4 Rate divider 4, 800/4 -> 200hz rate
    // 01: Output group select
    // 0028: GroupField1
    // XX: Checksum, no checksum
    char cmd2[27] = "$VNWRG,75,1,4,01,0200*XX\r\n";  // turn on IMU binary output
    serial_->write(cmd2, 26);
    delay(10);
  #else
    // subscribe
    if (!setup_attempted_ && sim_channel != NULL && zcm_ != NULL){
      mithl_vectorXf_t_subscribe(zcm_, sim_channel, &imu_sim_handler_helper, this);
    }
  #endif
  setup_attempted_ = true;
}

void IMUManager::IMUManager::update() {
  #ifdef COMPILE_FOR_ARDUINO
    while (serial_->available() > 0) {
      imu_parser_.new_byte(serial_->read());
    }
    if(imu_parser_.ready()){
      imu_parser_.parse(imu_data_, 6);
      last_heard_from_imu_ = get_current_time();
    }
    if (get_current_time() - last_heard_from_imu_ > IMU_RESTART_PERIOD){
      doIMUSetup();
      last_heard_from_imu_ = get_current_time();
    }
  #else
    ;
  #endif

  // health check
  if (last_identical_message_check_ - get_current_time() > IMU_IDENTICAL_MESSAGE_CHECK_PERIOD){
    last_identical_message_check_ = get_current_time();
    // Check against old data
    bool pass = true;
    for (int i=0; i<6; i++){
      if (fabs(imu_data_[i] - old_imu_data_0_[i]) <= IMU_IDENTICAL_MESSAGE_CHANGE_AMOUNT){
        pass = false;
      }
    }
    if (pass){
      number_of_identical_messages_ = 0;
    } else {
      number_of_identical_messages_ += 1;
    }
    // update old data
    memcpy(old_imu_data_0_, imu_data_, sizeof(float)*6);
  }
}

void IMUManager::imu_sim_handler(const mithl_vectorXf_t * msg){
  if (msg->rows != 6){
    printf("IMU Sim Handler got an IMU vectorXf_t with %d rows. WTF?\n", msg->rows);
  } else {
    memcpy(imu_data_, msg->data, sizeof(float)*6);
    last_heard_from_imu_ = get_current_time();
  }
}

bool IMUManager::is_imu_healthy(){
  // haven't heard from it at all in the last while
  if (get_current_time() - last_heard_from_imu_ > IMU_HEALTHY_TIMEOUT){
    return false;
  }
  // number of times in a row we've received values within 1E-6 of each other
  // on +x acceleration channel
  if (number_of_identical_messages_ > IMU_IDENTICAL_MESSAGE_PANIC_THRESHOLD){
    return false;
  }
  return true;
}

