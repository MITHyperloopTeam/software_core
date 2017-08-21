#include "BrakeActuatorController.h"
#include "arduino_time_manager.h"
#include "arduino_warning_sender.h" 

#include "zcmgen_c/mithl_vectorXf_t.h"
static void bac_config_handler(const zcm_recv_buf_t * rbuf, const char * channel, 
  const mithl_bac_config_t * msg, void * user){
  ((BrakeActuatorController*)user)->HandleConfig(msg);
}

static void bac_teleop_cmd_handler(const zcm_recv_buf_t * rbuf, const char * channel, 
  const mithl_bac_teleop_cmd_t * msg, void * user){
  ((BrakeActuatorController*)user)->SetTeleopCommand(msg);
}

static void bac_auto_cmd_handler(const zcm_recv_buf_t * rbuf, const char * channel, 
  const mithl_bac_auto_cmd_t * msg, void * user){
  ((BrakeActuatorController*)user)->SetAutoCommand(msg);
}

static void bac_mode_handler(const zcm_recv_buf_t * rbuf, const char * channel, 
  const mithl_bac_mode_t * msg, void * user){
  ((BrakeActuatorController*)user)->HandleMode(msg);
}

static void estop_handler(const zcm_recv_buf_t * rbuf, const char * channel, 
  const mithl_trigger_t * msg, void * user){
  ((BrakeActuatorController*)user)->HandleEStop(msg);
}

static void fsm_state_handler(const zcm_recv_buf_t * rbuf, const char * channel, 
  const mithl_state_t * msg, void * user){
  ((BrakeActuatorController*)user)->HandleFSMState(msg);
}

BrakeActuatorController::BrakeActuatorController(zcm_t * zcm, EncoderHandler encoder_handler) :
        initialized_(false),
        zcm_(zcm)
{
  Reset();
  mithl_bac_config_t_subscribe(zcm_, "_BAC_CONFIG_SET", &bac_config_handler, this);
  mithl_bac_teleop_cmd_t_subscribe(zcm_, "_BAC_TELEOP_CMD_SET", &bac_teleop_cmd_handler, this);
  mithl_bac_auto_cmd_t_subscribe(zcm_, "_BAC_AUTO_CMD_SET", &bac_auto_cmd_handler, this);
  mithl_bac_mode_t_subscribe(zcm_, "_BAC_MODE_SET", &bac_mode_handler, this);
  mithl_trigger_t_subscribe(zcm_, "_ESTOP", &estop_handler, this);
  mithl_state_t_subscribe(zcm_, "FSM_STATE", &fsm_state_handler, this);

  //pinMode(Pin_Encoder, INPUT);
  //attachInterrupt(digitalPinToInterrupt(Pin_Encoder), encoder_handler, RISING);
}

void BrakeActuatorController::Reset() {
  mode_ = MODE_ESTOP;

  motor_temp_ = 0.0;
  gh_front_ = 0.0;
  gh_rear_ = 0.0;


  closing_mode_ = 1;

  // Set default configuration.
  config_.teleop_watchdog_duration = 1.0;
  config_.teleop_watchdog_used = true;
  config_.auto_watchdog_used = true;
  config_.auto_watchdog_duration = 1.0;
  
  config_.minimum_disarm_duration = 5.0;

  config_.valve_scaling_above = 1.0;
  config_.valve_scaling_below = 1.0;
  config_.valve_bias = 0.82;
  config_.valve_Kp = 0.07;
  config_.valve_Kd = 0.025;
  config_.valve_Ki = 0.0;
  config_.valve_min = 0.7;
  config_.valve_exp = 1.0;
  config_.motor_scaling = 1.0;
  config_.motor_bias = 0.1;
  config_.motor_Kp = 1.0;
  config_.motor_Kd = 0.2;
  config_.motor_Ki = 0.0;
  config_.motor_exp = 1.0;
  config_.integrator_lim = .1;
  config_.distance_estimate_rate_constant = 0.05;
  config_.velocity_estimate_rate_constant = 0.1;
  config_.deadzone_width = 0.1;
  config_.switch_period = 0.25;

  // to ESTOP, turn everything off
  estop_command_.PWREN = false;
  estop_command_.PumpMotorPWM = 0.0;
  estop_command_.PumpMotorEnable = false;
  estop_command_.OnOffValve = false;
  estop_command_.ValvePWM = 0.0;
  estop_command_.ValveEnable = false;
  estop_command_.LSMotorEnable = false;
  estop_command_.LSMotorDisable = true;
  estop_command_.LSActuatorEnable = false;
  estop_command_.LSActuatorDisable = true;

  // to MAINTAIN ARM, hold brakes open
  arm_command_.PWREN = true;
  arm_command_.PumpMotorPWM = 0.0;
  arm_command_.PumpMotorEnable = false;
  arm_command_.OnOffValve = true;
  arm_command_.ValvePWM = 1.0;
  arm_command_.ValveEnable = true;
  arm_command_.LSMotorEnable = false;
  arm_command_.LSMotorDisable = true;
  arm_command_.LSActuatorEnable = false;
  arm_command_.LSActuatorDisable = true;

  estimated_motor_rate_ = 0.0;
  estimated_distance_ = 0.0;

  // teleop command defaults to estop
  memcpy(&last_teleop_command_, &estop_command_, sizeof(mithl_bac_teleop_cmd_t));
  memcpy(&last_command_, &estop_command_, sizeof(mithl_bac_teleop_cmd_t));

  // auto command defaults to close
  last_auto_command_.setpoint = 0.0;

  // estimator defaults to unhappy so no auto control is possible until
  // we get legitimate readings
  estimator_fault_ = true;
  estimated_distance_ = 0.0;
  estimated_distance_velocity_ = 0.0;
  estimated_distance_last_ = 0.0;

  double now = get_current_time();
  last_teleop_cmd_time_ = now - config_.teleop_watchdog_duration*2.;
  last_auto_cmd_time_ = now - config_.auto_watchdog_duration*2.;

  last_cmd_issue_time_ = now - COMMAND_PERIOD;
  last_config_report_time_ = now - CONFIG_REPORT_PERIOD;
  last_auto_command_report_time_ = now - AUTO_COMMAND_REPORT_PERIOD;
  last_teleop_command_report_time_ = now - TELEOP_COMMAND_REPORT_PERIOD;
  last_state_low_rate_report_time_ = now - STATE_LOW_RATE_REPORT_PERIOD;
  last_state_high_rate_report_time_ = now - STATE_HIGH_RATE_REPORT_PERIOD;
  last_sensor_update_time_ = now - SENSOR_UPDATE_PERIOD;
  last_saw_fsm_arm_state_ = now + 10; // give us a long window during bootup when brakes can't close if they're open
  last_control_read_ = now;
  last_encoder_rate_calc_time_ = now;
  last_mode_switch_ = now;

  brake_pressure_ = 0.0;

  average_time_between_control_ = 0.0;
  average_time_between_control_variance_ = 0.0;

  // reset encoder ticking counts
  total_encoder_count_ = 0;
  last_position_estimate_time_ = now;

  // set em all up 
  pinMode(Pin_PWREN, OUTPUT);  
  pinMode(Pin_PumpMotorEnable, OUTPUT);  
  pinMode(Pin_PumpMotorPWM, OUTPUT);
  pinMode(Pin_ONOFFValve, OUTPUT);   
  pinMode(Pin_ValvePWM, OUTPUT);  
  pinMode(Pin_ValveEnable, OUTPUT);  
  pinMode(Pin_LSMotorEnable, OUTPUT);  
  pinMode(Pin_LSMotorDisable, OUTPUT);  
  pinMode(Pin_LSActuatorEnable, OUTPUT);  
  pinMode(Pin_LSActuatorDisable, OUTPUT);

  // and commit those
  CommitCommand(arm_command_);

  initialized_ = true;
}


void BrakeActuatorController::Update() {
  double now = get_current_time();

  if (now - last_sensor_update_time_ > SENSOR_UPDATE_PERIOD){
    UpdateFromSensors();
    last_sensor_update_time_ = now;
  }

  // encoder update
  if (now - last_encoder_rate_calc_time_ > ENCODER_RATE_CALC_PERIOD){
    encoder_mtx_.lock();
    double elapsed = now - last_encoder_rate_calc_time_;
    last_encoder_rate_calc_time_ = now;
    // 12 ticks per rev, so go to revolutions, then to hz
    estimated_motor_rate_ = ((float)total_encoder_count_) / 12. / elapsed;
    total_encoder_count_ = 0;
    encoder_mtx_.unlock();
  }

  // for now, just grab low level commands and submit them
  // periodically
  if (now - last_cmd_issue_time_ > COMMAND_PERIOD){
    last_cmd_issue_time_ = now;

    // decide what kind of command to send
    BAC_Mode mode = GetMode();

    bool maintain_arm = false;
    fsm_state_mtx_.lock();
    // maintain arm state if we saw an arm state up to disarm duration ago
    if (get_current_time() - last_saw_fsm_arm_state_ < config_.minimum_disarm_duration){
      maintain_arm = true;
    }
    fsm_state_mtx_.unlock();

    if (maintain_arm){
      CommitCommand(arm_command_);
    } else {
      // we'll be checking watchdog inside so might as well lock for the duration
      config_mtx_.lock();
      if (mode == MODE_ESTOP){
        SetAndCommandEStop();
      } else if (mode == MODE_TELEOP){
        // check watchdog
        if (config_.teleop_watchdog_used && now - last_teleop_cmd_time_ >= config_.teleop_watchdog_duration){
          SetAndCommandEStop();
        } else {
          teleop_cmd_mtx_.lock();
          CommitCommand(last_teleop_command_);
          teleop_cmd_mtx_.unlock();
        }
      } else if (mode == MODE_AUTO){
        // check watchdog and estimator state
        if (estimator_fault_ || (config_.auto_watchdog_used && now - last_auto_cmd_time_ >= config_.auto_watchdog_duration)){
          SetAndCommandEStop();
        } else {
          CalculateAndCommitAutoCommand();
        }
      } else {
        // unknown mode state, so estop the pod.
        mode = MODE_ESTOP;
        char warning_msg[100];
        sprintf(warning_msg, "BAC: Somehow in invalid mode %d, ESTOPing\n", mode);
        send_warning_message(zcm_, "_BAC_WARN", warning_msg);
      }
    
      config_mtx_.unlock();
    }
  }

  if (now - last_auto_command_report_time_ > AUTO_COMMAND_REPORT_PERIOD){
    last_auto_command_report_time_ = now;
    // report back
    mithl_bac_auto_cmd_t auto_report_msg;
    auto_cmd_mtx_.lock();
    memcpy(&auto_report_msg, &last_auto_command_, sizeof(mithl_bac_auto_cmd_t));
    auto_cmd_mtx_.unlock();
    auto_report_msg.utime = now*1000*1000;
    mithl_bac_auto_cmd_t_publish(zcm_, "_BAC_AUTO_CMD_STATE", &auto_report_msg);

    mithl_vectorXf_t timeout_msg;
    timeout_msg.rows = 1;
    float data[1];
    data[0] = fmax(config_.minimum_disarm_duration - (get_current_time() - last_saw_fsm_arm_state_), 0.0);
    timeout_msg.data = data;
    mithl_vectorXf_t_publish(zcm_, "_BAC_ARM_COUNTDOWN", &timeout_msg);
  }

  if (now - last_teleop_command_report_time_ > TELEOP_COMMAND_REPORT_PERIOD){
    last_teleop_command_report_time_ = now;
    mithl_bac_teleop_cmd_t teleop_report_msg;
    teleop_cmd_mtx_.lock();
    memcpy(&teleop_report_msg, &last_teleop_command_, sizeof(mithl_bac_teleop_cmd_t));
    teleop_cmd_mtx_.unlock();
    teleop_report_msg.utime = now*1000*1000;
    mithl_bac_teleop_cmd_t_publish(zcm_, "_BAC_TELEOP_CMD_STATE", &teleop_report_msg);
  }

  if (now - last_state_low_rate_report_time_ > STATE_LOW_RATE_REPORT_PERIOD){
    last_state_low_rate_report_time_ = now;
    mithl_bac_state_low_rate_t state_msg;
    state_msg.utime = now*1000*1000;
    state_msg.estimator_fault = estimator_fault_;
    state_msg.est_control_rate = average_time_between_control_;
    state_msg.est_control_variance = average_time_between_control_variance_;
    state_msg.pump_temperature = motor_temp_;
    command_mtx_.lock();
    memcpy(&state_msg.cmd_state, &last_command_, sizeof(mithl_bac_teleop_cmd_t));
    command_mtx_.unlock();
    mithl_bac_state_low_rate_t_publish(zcm_, "_BAC_STATE_L", &state_msg);
  }
  
  if (now - last_state_high_rate_report_time_ > STATE_HIGH_RATE_REPORT_PERIOD){
    last_state_high_rate_report_time_ = now;
    mithl_bac_state_high_rate_t state_msg;
    state_msg.utime = now*1000*1000;
    state_msg.distance_1 = XD1_;
    state_msg.distance_2 = XD2_;
    state_msg.brake_pressure = brake_pressure_;
    state_msg.estimated_distance = estimated_distance_;
    state_msg.estimated_distance_velocity = estimated_distance_velocity_;
    state_msg.estimated_motor_rate = estimated_motor_rate_;
    state_msg.gh_front = gh_front_;
    state_msg.gh_rear = gh_rear_;

    command_mtx_.lock();
    state_msg.PumpMotorPWM = last_command_.PumpMotorPWM;
    state_msg.OnOffValve = last_command_.OnOffValve;
    state_msg.ValvePWM = last_command_.ValvePWM;
    command_mtx_.unlock();

    auto_cmd_mtx_.lock();
    state_msg.setpoint = last_auto_command_.setpoint;
    auto_cmd_mtx_.unlock();
    state_msg.error = estimated_distance_error_;
    state_msg.integrator = estimated_distance_error_integrator_;
    mithl_bac_state_high_rate_t_publish(zcm_, "_BAC_STATE_H", &state_msg);
  }


  if (now - last_config_report_time_ > CONFIG_REPORT_PERIOD){
    last_config_report_time_ = now;
    config_.utime = now * 1000 * 1000;
    mithl_bac_config_t_publish(zcm_, "_BAC_CONFIG_STATE", &config_);

    mithl_bac_mode_t report_mode_msg;
    report_mode_msg.utime = now*1000*1000;
    report_mode_msg.mode = GetMode();
    mithl_bac_mode_t_publish(zcm_, "_BAC_MODE_STATE", &report_mode_msg);
  }
}

void BrakeActuatorController::UpdateFromSensors() {
  //Read linear pot values and convert first to voltage, then distance.
  // 4mA*150Ohm = 600mV = 0 mm
  // 20mA*150Ohm = 3000mV = 150mm = 5.906 inches
  XD1_ = (((float)analogRead(Pin_LD1))*3.3/1024. - 0.6)*(5.906 / 2.4);
  XD2_ = (((float)analogRead(Pin_LD2))*3.3/1024. - 0.6)*(5.906 / 2.4);
  
  double now = get_current_time();
  double dt = now - last_position_estimate_time_;
  
  float alpha_dist = dt / (dt + config_.distance_estimate_rate_constant);
  float alpha_vel = dt / (dt + config_.velocity_estimate_rate_constant);

  // distance estimate is low-passed over time
  estimated_distance_ = estimated_distance_last_*(1.0 - alpha_dist) + (XD1_ + XD2_ + DistanceConstant)*(alpha_dist);

  float new_vel = (estimated_distance_ - estimated_distance_last_) / dt;
  // velocity estimate for low-passed distance is also low-passed (even more!)
  estimated_distance_velocity_ = alpha_vel * new_vel + (1. - alpha_vel) * estimated_distance_velocity_;
  estimated_distance_last_ = estimated_distance_;
  last_position_estimate_time_ = now;

  //TODO(gizatt): more sanity checking
  
  // Read pressure. TODO(gizatt): wtf are these constants
  double new_brake_pressure = (analogRead(Pin_Pressure) * 3.3 / 1024.0 - 0.6) * 3000.0 /(3.0 - 0.6);
  brake_pressure_ = brake_pressure_*PRESSURE_ALPHA + new_brake_pressure*(1.-PRESSURE_ALPHA);

  if (brake_pressure_ >= BrakePressureLimit){
    SetAndCommandEStop();
    estimator_fault_ = true;
  } else {
    estimator_fault_ = false;
  }

  float new_motor_temp = (((float)analogRead(Pin_MotorTemp))*3.3/1024.) * (1000. / 10.); // 10mV / deg C
  motor_temp_ = new_motor_temp * (1.0 - MotorTempAlpha) + motor_temp_ * (MotorTempAlpha);
  
  float new_gh_front = (((float)analogRead(Pin_GH_FrontRight))*3.3/1024. - 0.6)*20.833+30.0;  
  float new_gh_rear = (((float)analogRead(Pin_GH_RearLeft))*3.3/1024. - 0.6)*20.833+30.0;  
  // 4mA*150Ohm  = 600mV = 30mm
  // 20mA*150Ohm = 3000mV = 80mm
  // i.e. 50mm / (2.4V) conversion ratio 20.8333 mm/V
  gh_front_ = new_gh_front * (1.0 - GHAlpha) + gh_front_ * (GHAlpha);
  gh_rear_ =   new_gh_rear * (1.0 - GHAlpha) + gh_rear_  * (GHAlpha);
}

void BrakeActuatorController::SetAndCommandEStop() {
  mode_mtx_.lock();
  mode_ = MODE_ESTOP;
  mode_mtx_.unlock();

  CommitCommand(estop_command_);
}

void BrakeActuatorController::CalculateAndCommitAutoCommand() {
  auto_cmd_mtx_.lock();
  // config mtx already locked for duration
  mithl_bac_teleop_cmd_t cmd;


  double now = get_current_time();
  float dt = now - last_control_read_;
  last_control_read_ = now;

  // do PI control with both actuators
  float error = last_auto_command_.setpoint - estimated_distance_;
  estimated_distance_error_ = error;
  estimated_distance_error_integrator_ += error * dt;
  estimated_distance_error_integrator_ = fmax(fmin(estimated_distance_error_integrator_, config_.integrator_lim), -config_.integrator_lim);

  // can be in closing mode, or opening mode
  // closing mode = motor off, on/off off, prop varying. When estimated_estinace > setpoint, i.e. error negative
  // opening mode = motor varying, on/off on, prop 100%, setpoint > estimated_distance, i.e. error positive
  // can only switch between them once every SWITCH_PERIOD ms

  if (error >= config_.deadzone_width && (now - last_mode_switch_) > config_.switch_period){
    closing_mode_ = 0;
    last_mode_switch_ = now;
  }
  if (error <= -config_.deadzone_width && (now - last_mode_switch_) > config_.switch_period){
    closing_mode_ = 1;
    last_mode_switch_ = now;
  }

  cmd.PWREN = true;
  cmd.PumpMotorEnable = true;
  cmd.ValveEnable = true;
  if (closing_mode_ == 1){
    // never run motor
    cmd.PumpMotorPWM = 0.0;
    cmd.OnOffValve = false;

    // error should be negative
    if (error < 0){
      float valve_command = config_.valve_bias + config_.valve_Kp * error
        - config_.valve_Kd * (estimated_distance_velocity_);
      valve_command = fmax(valve_command, config_.valve_min);

      // when valve is commanded to 0.0, it is fully open
      cmd.ValvePWM = fmin( fmax(valve_command, 0.0), 1.0);
    } else {
      cmd.ValvePWM = 1.0;
    }

  } else {
    cmd.OnOffValve = true;
    cmd.ValvePWM = 1.0;
    if (error > 0){
      float motor_command = config_.motor_bias + config_.motor_Kp * (error)
        - config_.motor_Kd * (estimated_distance_velocity_);
      cmd.PumpMotorPWM = fmin( fmax (motor_command, 0.0), 1.0 );
    } else {
      cmd.PumpMotorPWM = 0.0;
    }
  }

/*
  // Control law is, for both valve and motor:
  float valve_error = error * config_.valve_Kp + 
                              estimated_distance_error_integrator_ * config_.valve_Ki -
                              estimated_distance_velocity_ * config_.valve_Kd;
  //if (fabs(valve_error) > 0)
  //  valve_error = (valve_error/fabs(valve_error)) * pow(fabs(valve_error), config_.valve_exp);

  if (valve_error >= 0)
    valve_error *= config_.valve_scaling_above;
  else 
    valve_error *= config_.valve_scaling_below;
  float valve_command = -valve_error
                        + (1.0 - config_.valve_bias); // we invert valve command later
                                                      // so invert bias here so it
                                                      // makes sense in the same unit
  valve_command = fmin(valve_command, 1.0-config_.valve_min);

  float motor_error =         error * config_.motor_Kp + 
                              estimated_distance_error_integrator_ * config_.motor_Ki -
                              estimated_distance_velocity_ * config_.motor_Kd;
  //if (fabs(motor_error) > 0)
  //  motor_error = (motor_error/fabs(motor_error)) * pow(fabs(motor_error), config_.motor_exp);
  float motor_command = config_.motor_scaling*motor_error
                        + config_.motor_bias;

  // construct our complete low-level command
  cmd.PWREN = true;
  if (motor_command > 0.0){
    cmd.PumpMotorPWM = fmin(1.0, motor_command);
    cmd.PumpMotorEnable = true;
    // TODO(gizatt): what to do with pump direction?

  } else {
    cmd.PumpMotorPWM = 0.0;
    cmd.PumpMotorEnable = false;
  }

  // onoff valve OFF means fluid flows freely
  // onoff valve ON means fluid is stopped 
  // for now just let prop valve do all the work
  cmd.OnOffValve = false;

  // when valve is commanded to 0.0, it is fully open
  // so invert this input
  cmd.ValvePWM = 1.0 - fmin( fmax(valve_command, 0.0), 1.0 );
  cmd.ValveEnable = (cmd.ValvePWM > 0.0);
  */

  cmd.LSMotorEnable = false;
  cmd.LSMotorDisable = true;
  cmd.LSActuatorEnable = false;
  cmd.LSActuatorDisable = true;

  CommitCommand(cmd); 

  double variance_this = fabs(dt - average_time_between_control_);
  average_time_between_control_ = CONTROL_RATE_ALPHA*average_time_between_control_ + (1.0-CONTROL_RATE_ALPHA)*dt;
  average_time_between_control_variance_ = CONTROL_RATE_ALPHA*average_time_between_control_variance_ + (1.0-CONTROL_RATE_ALPHA)*variance_this;

  auto_cmd_mtx_.unlock();
}

void BrakeActuatorController::CommitCommand(mithl_bac_teleop_cmd_t command_to_commit) {
  command_mtx_.lock();
  digitalWrite(Pin_PWREN, command_to_commit.PWREN);
  analogWrite(Pin_PumpMotorPWM, (int)(command_to_commit.PumpMotorPWM*255.0));
  digitalWrite(Pin_PumpMotorEnable, command_to_commit.PumpMotorEnable);
  digitalWrite(Pin_ONOFFValve, command_to_commit.OnOffValve);
  analogWrite(Pin_ValvePWM, (int)(command_to_commit.ValvePWM*255.0));
  digitalWrite(Pin_ValveEnable, command_to_commit.ValveEnable);
  // TODO: SANITY CHECK THESE AND ONLY SWITCH IN "SAFE" ORDERINGS

  // Sanity check enable/disable Hbridge command pairs
  if (command_to_commit.LSMotorEnable && command_to_commit.LSMotorDisable){
    command_to_commit.LSMotorEnable = false;
    command_to_commit.LSMotorDisable = true;
  }
  // same if both are off
  else if (!command_to_commit.LSMotorEnable && !command_to_commit.LSMotorDisable){
    command_to_commit.LSMotorEnable = false;
    command_to_commit.LSMotorDisable = true;
  }
  if (command_to_commit.LSActuatorEnable && command_to_commit.LSActuatorDisable){
    command_to_commit.LSActuatorEnable = false;
    command_to_commit.LSActuatorDisable = true;
  }
  else if (!command_to_commit.LSActuatorEnable && !command_to_commit.LSActuatorDisable){
    command_to_commit.LSMotorEnable = false;
    command_to_commit.LSActuatorDisable = true;
  }

  // If we're switching LS Motor, get the switch order correct -- always turn one
  // off before turning the other on
  if (command_to_commit.LSMotorEnable == true){
    digitalWrite(Pin_LSMotorDisable, false);
    digitalWrite(Pin_LSMotorEnable, true);
  }
  if (command_to_commit.LSMotorDisable == true){
    digitalWrite(Pin_LSMotorEnable, false);
    digitalWrite(Pin_LSMotorDisable, true);
  }
  // Same for the actuator
  if (command_to_commit.LSActuatorEnable == true){
    digitalWrite(Pin_LSActuatorDisable, false);
    digitalWrite(Pin_LSActuatorEnable, true);
  }
  if (command_to_commit.LSActuatorDisable == true){
    digitalWrite(Pin_LSActuatorEnable, false);
    digitalWrite(Pin_LSActuatorDisable, true);
  }

  memcpy(&last_command_, &command_to_commit, sizeof(mithl_bac_teleop_cmd_t));
  command_mtx_.unlock();
}

void BrakeActuatorController::HandleEStop(const mithl_trigger_t * msg){
  mode_mtx_.lock();
  mode_ = MODE_ESTOP;
  mode_mtx_.unlock();
}

void BrakeActuatorController::HandleMode(const mithl_bac_mode_t * msg){
  mode_mtx_.lock();
  if (msg->mode == MODE_ESTOP)
    mode_ = MODE_ESTOP;
  else if (msg->mode == MODE_TELEOP)
    mode_ = MODE_TELEOP;
  else if (msg->mode == MODE_AUTO)
    mode_ = MODE_AUTO;
  else{
    char warning_msg[100];
    sprintf(warning_msg, "BAC: Invalid mode cmd received, %d\n", msg->mode);
    send_warning_message(zcm_, "BAC_WARN", warning_msg);
  }
  mode_mtx_.unlock();
}

void BrakeActuatorController::HandleConfig(const mithl_bac_config_t * msg){
  config_mtx_.lock();
  memcpy(&config_, msg, sizeof(mithl_bac_config_t));
  config_mtx_.unlock();
}

void BrakeActuatorController::HandleFSMState(const mithl_state_t * msg){
  fsm_state_mtx_.lock();
  if (msg->currentState == MITHL_STATE_T_ARM){
    last_saw_fsm_arm_state_ = get_current_time();
  }
  fsm_state_mtx_.unlock();
}

void BrakeActuatorController::SetTeleopCommand(const mithl_bac_teleop_cmd_t * msg){
  teleop_cmd_mtx_.lock();
  last_teleop_cmd_time_ = get_current_time();
  memcpy(&last_teleop_command_, msg, sizeof(mithl_bac_teleop_cmd_t));
  teleop_cmd_mtx_.unlock();
}
  
void BrakeActuatorController::SetAutoCommand(const mithl_bac_auto_cmd_t * msg){
  auto_cmd_mtx_.lock();
  last_auto_cmd_time_ = get_current_time();
  memcpy(&last_auto_command_, msg, sizeof(mithl_bac_auto_cmd_t));
  auto_cmd_mtx_.unlock();
}

BrakeActuatorController::BAC_Mode BrakeActuatorController::GetMode() { 
  BAC_Mode ret; 
  mode_mtx_.lock();
  ret = mode_;
  mode_mtx_.unlock();
  return ret;
}

void BrakeActuatorController::HandleEncoderTick() {
  //encoder_mtx_.lock();
  total_encoder_count_+=1;
  //encoder_mtx_.unlock();
  if (total_encoder_count_ > MAX_ENCODER_TICK_PER_LOOP){
    // this has runaway... disable interrupts
    detachInterrupt(digitalPinToInterrupt(Pin_Encoder));
  }
}
