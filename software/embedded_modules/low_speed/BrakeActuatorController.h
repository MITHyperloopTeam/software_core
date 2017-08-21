#ifndef BRAKE_ACTUATOR_CONTROLLER_H
#define BRAKE_ACTUATOR_CONTROLLER_H

#include "Arduino.h"

#include "zcmgen_c/mithl_bac_config_t.h"
#include "zcmgen_c/mithl_bac_teleop_cmd_t.h"
#include "zcmgen_c/mithl_bac_auto_cmd_t.h"
#include "zcmgen_c/mithl_bac_mode_t.h"
#include "zcmgen_c/mithl_bac_state_high_rate_t.h"
#include "zcmgen_c/mithl_bac_state_low_rate_t.h"
#include "zcmgen_c/mithl_state_t.h"
#include "zcmgen_c/mithl_trigger_t.h"
#include "arduino_fake_mutex.hpp"

// rate of command submit time onto hardware
#define COMMAND_PERIOD 0.001 

// rate of publishing bac_config_t, bac_mode_t
#define CONFIG_REPORT_PERIOD 0.5

// rate of publishing auto command state
#define AUTO_COMMAND_REPORT_PERIOD 0.1

// rate of publishing teleop command state
#define TELEOP_COMMAND_REPORT_PERIOD 0.25

// bac_state_low_rate_t
#define STATE_LOW_RATE_REPORT_PERIOD 0.25

// bac_state_high_rate_t
#define STATE_HIGH_RATE_REPORT_PERIOD 0.033

#define ENCODER_RATE_CALC_PERIOD 0.05
#define SENSOR_UPDATE_PERIOD 0.001

// We'll never see faster than 10krpm 
// i.e. 60000 hz
#define MAX_ENCODER_TICK_PER_LOOP (60000. * ENCODER_RATE_CALC_PERIOD)

// Control rate estimation
#define CONTROL_RATE_ALPHA 0.98

#define PRESSURE_ALPHA 0.995

class BrakeActuatorController {
public:
    enum BAC_Mode {
    MODE_ESTOP = 0, // turns off power to release brake
    MODE_TELEOP = 1, // accepts brake inputs directly as a message
    MODE_AUTO = 2 // computes brake inputs from a setpoint input
  };

  BAC_Mode mode_;
  
  typedef void (*EncoderHandler)(void);

private:
  bool initialized_;
  zcm_t * zcm_;

  mithl_bac_teleop_cmd_t last_command_;
  mithl_bac_teleop_cmd_t last_teleop_command_;
  mithl_bac_auto_cmd_t last_auto_command_;
  mithl_bac_teleop_cmd_t estop_command_;
  mithl_bac_teleop_cmd_t arm_command_;
  mithl_bac_config_t config_;

  // timing loops
  // manages command rate:
  double last_cmd_issue_time_;
  // manages sensor read rates
  double last_sensor_update_time_;
  
  // manages watchdog:
  double last_teleop_cmd_time_;
  double last_auto_cmd_time_;

  // manages FSM arm state observation
  double last_saw_fsm_arm_state_;

  // manages report rates:
  double last_state_low_rate_report_time_;
  double last_state_high_rate_report_time_;
  double last_auto_command_report_time_;
  double last_teleop_command_report_time_;
  double last_config_report_time_;

  // control rate estimation state
  double average_time_between_control_;
  double average_time_between_control_variance_;
  double last_encoder_rate_calc_time_;
  
  // two-mode auto controller state
  double last_mode_switch_; // can only switch modes once every so often
  int closing_mode_; // whether we're in the closing mode

  // brake state estimator state
  float estimated_distance_;
  float estimated_distance_last_;
  double last_position_estimate_time_;
  float estimated_distance_velocity_;

  float estimated_distance_error_integrator_;
  float estimated_distance_error_;
  bool estimator_fault_;
  double last_control_read_;

  float estimated_motor_rate_; // in rpm

  // encoder counting state
  volatile int total_encoder_count_;

  Mutex config_mtx_;
  Mutex teleop_cmd_mtx_;
  Mutex auto_cmd_mtx_;
  Mutex mode_mtx_; // probably unnecessary but no guarantees how the enum is implemented
  Mutex command_mtx_;
  Mutex encoder_mtx_;
  Mutex fsm_state_mtx_;

  //  Definition of all controlled pins and variables
  const int Pin_PWREN = 40;  //Enable all Brake Interface PCB power
  //Pump motor
  const int Pin_PumpMotorPWM = 7;      // Speed PWM to the Pump Motor
  const int Pin_PumpMotorEnable = 41; //Enable Pump Motor
  const unsigned int Pin_Encoder = 30;

  //ON-OFF Valve
  const int Pin_ONOFFValve = 34; //Enable ON/OFF Valve
  //Propotional Valve, missing hardware pin, need to revise on second version main PCB
  const int Pin_ValvePWM = 8;      // Speed PWM to the propotional valve
  const int Pin_ValveEnable = 33;  // Enable / disable prop valve 

  // Enables disables flipped from layout due to wiring error
  //Low speed motor Enable/Disable
  const int Pin_LSMotorEnable = 39; //Enable LS motor
  const int Pin_LSMotorDisable = 38; //Disable LS motor (after turn Pin 51 LOW, has to turn this HIGH)
  //Low speed actuator Enable/Disable
  const int Pin_LSActuatorEnable = 37; //Enable LS actuator
  const int Pin_LSActuatorDisable = 36; //Disable LS actuator (after turn Pin 50 LOW, has to turn this HIGH)

  //Definitions of the sensor signals
  const int Pin_LD1 = A9; //Linear pot 1 analog pin to A1
  const int Pin_LD2 = A2; //Linear pot 2 analog pin to A2
  float XD1_;  //Linear pot 1 displacement
  float XD2_;  //Linear pot 2 displacement

  // XD_1 + XD_2 = 2.35 when closed
  // XD_1 + XD_2 = 7.65 when open
  const float DistanceConstant = -2.35; //inches

  // Encoder info
  const int Encoder_Pulse_Per_Rev = 12;

  //Pressure
  const int Pin_Pressure = A1; //Reading the pressure 
  float brake_pressure_; //float readout
  const float BrakePressureLimit = 1700.0;

  //GH front and back
  const int Pin_GH_FrontRight = A3;
  float gh_front_;
  const int Pin_GH_RearLeft = A4;
  float gh_rear_;
  const float GHAlpha = 0.5;

  // Temp
  const int Pin_MotorTemp = A7;
  float motor_temp_;
  const float MotorTempAlpha = 0.995;

  void UpdateFromSensors();
  void CalculateAndCommitAutoCommand(); // helper, should ONLY be called from within Update.
  void SetAndCommandEStop();  
  void CommitCommand(mithl_bac_teleop_cmd_t command_to_commit);

public:

  BrakeActuatorController(zcm_t * zcm, EncoderHandler encoder_handler);
  void Reset();
  void Update();

  void SetTeleopCommand(const mithl_bac_teleop_cmd_t * msg);
  void SetAutoCommand(const mithl_bac_auto_cmd_t * msg);

  void HandleConfig(const mithl_bac_config_t * msg);
  void HandleMode(const mithl_bac_mode_t * msg);
  void HandleEStop(const mithl_trigger_t * msg);
  void HandleFSMState(const mithl_state_t * msg);
  
  void HandleEncoderTick();

  BAC_Mode GetMode();
};

#endif // LINEAR_ACTUATOR_CONTROLLER_H
