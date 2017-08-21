#ifndef FIDUCIAL_MODULE_H
#define FIDUCIAL_MODULE_H

#include "Arduino.h"

#include "zcmgen_c/mithl_fiducial_config_t.h"
#include "zcmgen_c/mithl_fiducial_color_row_t.h"
#include "zcmgen_c/mithl_fiducial_teach_table_t.h"
#include "zcmgen_c/mithl_fiducial_color_dataval_t.h"
#include "zcmgen_c/mithl_string_t.h"

#define FIDUCIAL_MAX_RECEIVE_LEN (520)
#define FIDUCIAL_MAX_COMMAND_LEN (FIDUCIAL_MAX_RECEIVE_LEN)
#define FIDUCIAL_HEADER_LEN 8
#define FIDUCIAL_COLOR_ROW_LEN 16
#define FIDUCIAL_TEACH_TABLE_ROWS 31
// #define FIDUCIAL_BAUD_RATE 19200
#define FIDUCIAL_BAUD_RATE 115200

#define ACK_TIMEOUT 1.25 // 0.05

#define REPORT_PERIOD 2.0
#define FLAG_READ_PERIOD 0.5
#define CURRENT_READ_PERIOD 0.5
#define VOLTAGE_READ_PERIOD 0.5
#define TEMP_READ_PERIOD 0.5
#define SETPOINT_WRITE_PERIOD 0.1
#define COLOR_DATAVAL_READ_PERIOD 0.5

class FiducialController {

private:
  static const mithl_fiducial_config_t the_empty_config;

  bool initialized_;
  UARTClass* serial_;
  zcm_t * zcm_;

  // timing loops
  double last_report_time_;
  double last_color_report_time_;

  // known knowledge state
  double strip_width;
  double strip_space_between;

  // config state
  mithl_fiducial_config_t latest_desired_config;  // needs to be submitted
  mithl_fiducial_config_t tentative_config;  // submitted but not confirmed
  mithl_fiducial_config_t latest_config[2];  // confirmed truth parameter sets 0 and 1
  bool latest_config_known_;

  // teach table state
  mithl_fiducial_teach_table_t latest_desired_teach_table;  // needs to be submitted
  mithl_fiducial_teach_table_t tentative_teach_table;  // submitted but not confirmed
  mithl_fiducial_teach_table_t latest_teach_table[2];  // confirmed truth
  bool latest_teach_table_known_;

  // asynchronous command-sending state:
  bool waiting_for_ack_;
  unsigned short waiting_for_ack_order_;
  unsigned short waiting_for_ack_arg_;
  uint8_t command_buf_[FIDUCIAL_MAX_COMMAND_LEN];
  double command_send_time_;

  uint8_t receive_buf_[FIDUCIAL_MAX_RECEIVE_LEN];
  int frame_state_;
  int max_chars_to_receive_;

  uint8_t receive_any_buf_[FIDUCIAL_MAX_COMMAND_LEN];
  int receive_any_index_;

  int round_robin_start_;

public:

  FiducialController(UARTClass* serial, zcm_t * zcm, mithl_fiducial_config_t * default_config, mithl_fiducial_teach_table_t * default_teach_table);
  void Reset(mithl_fiducial_config_t * default_config, mithl_fiducial_teach_table_t * default_teach_table);
  void Update();
  void HandleReceive();
  void ParseReceiveBuffer();
  void PublishReceiveBuffer();
  void PublishBuffer(uint8_t * buf, size_t length, const char * channel_name);

  void SubmitCommandBuf(uint8_t * command_buf, int len, unsigned short ack_order, unsigned short ack_arg);

  void HandleConfig(const mithl_fiducial_config_t * msg);
  void HandleTeachTable(const mithl_fiducial_teach_table_t * msg);
//  // CONFIGURATION MODIFICATION:
//  void HandleConfigModificationAck(bool ack);
//  // commands setpoint. Range -1.0 to 1.0
  void SetConfig(unsigned int channel, unsigned int set_no, mithl_fiducial_config_t * config);
  void SetTeachTable(unsigned int channel, unsigned int set_no, mithl_fiducial_teach_table_t * config);
  void ReqConfig(unsigned int channel, unsigned int set_no);
  void ReqTeachTable(unsigned int channel, unsigned int set_no);
  void ReqDataval(unsigned int channel);
//  void SetSetpoint(unsigned int channel, float setpoint);
//  void SetAmpLimit(unsigned int channel, float limit);
//  void SetOperatingMode(unsigned int channel, OperatingMode mode);
//  void SetCLEDMode(unsigned int channel, CLEDMode mode);
//  // Kp: 0 to 25, default 20.0
//  void SetPIDKp(unsigned int channel, float Kp);
//  // Kd: 0 to 25, default 20.0
//  void SetPIDKd(unsigned int channel, float Kd);
//  // Ki: 0 to 25, default 20.0
//  void SetPIDKi(unsigned int channel, float Ki);
//  // Integral limit: Percent, default 100, min 1%.
//  void SetPIDIntegralCap(unsigned int channel, int integral_cap);
//
//  void SetTelemetryString();
//  void HandleTelemetryStringAck(bool ack);
//
//  // estop set, release
//  void SetESTOP();
//  void ReleaseESTOP();
//
//  void ReqFaultFlag();
//  void ReqRuntimeFlag(unsigned int channel);
//  void ReqStatusFlag();
//  void ReqMotorCurrent();
//  void ReqBatteryCurrent();
//  void ReqVoltage();
//  void ReqTemp();
//
//  void HandleConfig(const mithl_fiducial_config_t * msg);
};

#endif // FIDUCIAL_MODULE_H
