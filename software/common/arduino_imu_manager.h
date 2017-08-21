#ifndef IMU_SETUP_H
#define IMU_SETUP_H

#include "Arduino.h"
#include "zcmgen_c/mithl_vectorXf_t.h"
#include "IMUParse.h"
#include "arduino_time_manager.h"

// Quick-and-dirty, super lightweight IMU sim
// to remove some code duplication.

#define IMU_RESTART_PERIOD 1.0
#define IMU_HEALTHY_TIMEOUT 0.1
#define IMU_IDENTICAL_MESSAGE_PANIC_THRESHOLD 5
#define IMU_IDENTICAL_MESSAGE_CHECK_PERIOD 0.01
#define IMU_IDENTICAL_MESSAGE_CHANGE_AMOUNT 1E-6

class IMUManager {
  public:
    IMUParse imu_parser_;
    float imu_data_[6];
    double last_heard_from_imu_;
    // Used to sanity check IMU data health by checking whether all
    // channels are changing by a small amount between chekcs
    double last_identical_message_check_;
    int number_of_identical_messages_;
    float old_imu_data_0_[6];

    HardwareSerial * serial_;
    zcm_t * zcm_;
    bool setup_attempted_;
    IMUManager(HardwareSerial * serial, zcm_t * zcm = NULL, char * sim_channel = NULL);

    void doIMUSetup(char * sim_channel = NULL);

    void update();

    void imu_sim_handler(const mithl_vectorXf_t * msg);

    bool is_imu_healthy();
};
#endif // IMU_SETUP_H