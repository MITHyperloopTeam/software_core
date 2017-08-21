#ifndef ROBOTEQ_ADM_CONTROLLER_H
#define ROBOTEQ_ADM_CONTROLLER_H

#include "Arduino.h"

#include "zcmgen_c/mithl_adm_config_t.h"
#include "zcmgen_c/mithl_adm_status_t.h"
#include "zcmgen_c/mithl_trigger_t.h"

#define ROBOTEQ_ADM_MAX_COMMAND_LEN 50
#define ACK_TIMEOUT 0.01

#define SETPOINT_WATCHDOG_TIME 2.0
#define MAX_SETPOINT_UPWARDS_DIFF 0.1

#define REPORT_PERIOD 0.25
#define FLAG_READ_PERIOD 0.5
#define CURRENT_READ_PERIOD 0.5
#define VOLTAGE_READ_PERIOD 0.5
#define TEMP_READ_PERIOD 0.5
#define CASE_TEMP_READ_PERIOD 0.5
#define SETPOINT_WRITE_PERIOD 0.1
#define RPM_READ_PERIOD 0.5
#define HEARD_FROM_ADM_WATCHDOG_TIMEOUT 5.0

class RoboteqADMController {

private:
  bool initialized_;
  int case_temp_pin_;
  double case_temp_bias_;

  UARTClass* serial_;
  zcm_t * zcm_;

  // channel names
  char config_set_channel_[50];
  char config_cur_channel_[50];
  char status_cur_channel_[50];
  char estop_channel_[50];
  char reflash_channel_[50];

  // timing loops

  double last_report_time_;
  double last_fault_flag_read_;
  double last_runtime_flag_1_read_;
  double last_runtime_flag_2_read_;
  double last_status_flag_read_;
  double last_motor_current_read_;
  double last_battery_current_read_;
  double last_voltage_read_;
  double last_temp_read_;
  double last_case_temp_read_;
  double last_setpoint_write_;
  double last_rpm_read_;
  double last_heard_from_adm_;

  double last_received_setpoint_; 

  bool telemetry_ack_received_;
  unsigned int last_runtime_flag_channel_;

  // needs to be submitted:
  mithl_adm_config_t latest_desired_config;

  // submitted but not confirmed:
  mithl_adm_config_t tentative_config;


  // asynchronous command-sending state:
  bool waiting_for_ack_;
  char command_buf_[ROBOTEQ_ADM_MAX_COMMAND_LEN];
  double command_send_time_;
  bool ack_received_;
  void (*ack_callback_)(bool, void *);
  void * ack_callback_extra_;

  char receive_buf_[ROBOTEQ_ADM_MAX_COMMAND_LEN + 1]; // one extra space for terminator
  int receive_buf_ptr_;

  int round_robin_start_;

public:

  // confirmed truth:
  mithl_adm_status_t latest_status;
  mithl_adm_config_t latest_config;

  enum OperatingMode {
	OPEN_LOOP_SPEED = 0,
	CLOSED_LOOP_SPEED = 1,
	CLOSED_LOOP_POSITION_RELATIVE = 2,
	CLOSED_LOOP_COUNT_POSITION = 3,
	CLOSED_LOOP_POSITION_TRACKING = 4,
	TORQUE = 5
  };

  enum CLEDMode {
    CLED_NONE = 0,
    CLED_250 = 1,
    CLED_500 = 2,
    CLED_1000 = 3
  };

  enum CLFeedbackSensor {
    FEEDBACK_SENSOR_HALL = 0,
    FEEDBACK_SENSOR_ENCODER = 1
  };

  RoboteqADMController(UARTClass* serial, zcm_t * zcm, char * name, int case_temp_pin, mithl_adm_config_t * default_config);
  void Reset(mithl_adm_config_t * default_config);
  void Update();
  void HandleReceive();
  void ParseReceiveBuffer();

  void SubmitCommandBuf(char * command_buf, void (*ack_callback)(bool, void *), void * extra);

  // CONFIGURATION MODIFICATION:
  void HandleConfigModificationAck(bool ack);
  // commands setpoint. Range -1.0 to 1.0
  void SetSetpoint(unsigned int channel, float setpoint);
  // Basic operating mode and parameter limits:
  void SetAmpLimit(unsigned int channel, float limit);
  void SetOperatingMode(unsigned int channel, OperatingMode mode);
  void SetCLEDMode(unsigned int channel, CLEDMode mode);
  void SetCLFeedbackSensor(CLFeedbackSensor sensor);
  // Kp: 0 to 25, default 20.0
  void SetPIDKp(unsigned int channel, float Kp);
  // Kd: 0 to 25, default 20.0
  void SetPIDKd(unsigned int channel, float Kd);
  // Ki: 0 to 25, default 20.0
  void SetPIDKi(unsigned int channel, float Ki);
  // Integral limit: Percent, default 100, min 1%.
  void SetPIDIntegralCap(unsigned int channel, int integral_cap);

  void SetMaxRPM(unsigned int channel, int max_rpm);
  void SetMaxPowerForward(unsigned int channel, int max_power_forward);
  void SetMaxPowerReverse(unsigned int channel, int max_power_reverse);

  void SetTelemetryString();
  void HandleTelemetryStringAck(bool ack);

  // Brushless motor config
  void SetNumPoles(int n_poles);

  // Encoder settings
  enum EncoderUsage {
    ENCODER_USAGE_UNUSED = 0,
    ENCODER_USAGE_COMMAND = 1,
    ENCODER_USAGE_FEEDBACK = 2
  };
  enum SwitchingMode {
    SWITCHING_MODE_HALL = 0,
    SWITCHING_MODE_SINUSOIDAL = 1,
    SWITCHING_MODE_SENSORLESS = 2
  };
  enum SinusoidalMode {
    SINUSOIDAL_MODE_ENCODER = 0,
    SINUSOIDAL_MODE_HALL = 1,
    SINUSOIDAL_MODE_HALL_ENCODER = 2,
    SINUSOIDAL_MODE_SPI_SENSOR = 3,
    SINUSOIDAL_MODE_SIN_COS_SENSOR = 4,
    SINUSOIDAL_MODE_RESOLVER = 5
  };
  void SetEncoderUsage(unsigned int channel, EncoderUsage usage);
  void SetSwitchingMode(unsigned int channel, SwitchingMode mode);
  void SetSinusoidalMode(unsigned int channel, SinusoidalMode mode);
  void SetEncoderPulsePerRev(unsigned int channel, int ppr);
  void SetEncoderLowCountLimit(unsigned int channel, int count_lim);
  void SetEncoderHighCountLimit(unsigned int channel, int count_lim);

  // estop set, release
  void SetESTOP();
  void ReleaseESTOP();

  void ReqFaultFlag();
  void ReqRuntimeFlag(unsigned int channel);
  void ReqStatusFlag();
  void ReqMotorCurrent();
  void ReqBatteryCurrent();
  void ReqVoltage();
  void ReqTemp();
  void ReqSpeed(unsigned int channel);

  void HandleConfig(const mithl_adm_config_t * msg);
  void HandleEstop(const mithl_trigger_t * msg);
  void HandleReflashAll(const mithl_trigger_t * msg);
};

#endif // ROBOTEQ_ADM_CONTROLLER_H
