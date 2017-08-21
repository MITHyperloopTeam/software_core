#include "RoboteqADMController.h"
#include "arduino_time_manager.h"

static void adm_config_handler(const zcm_recv_buf_t * rbuf, const char * channel, 
  const mithl_adm_config_t * msg, void * user){
  ((RoboteqADMController*)user)->HandleConfig(msg);
}

static void estop_handler(const zcm_recv_buf_t * rbuf, const char * channel, 
  const mithl_trigger_t * msg, void * user){
  ((RoboteqADMController*)user)->HandleEstop(msg);
}

static void reflash_all_handler(const zcm_recv_buf_t * rbuf, const char * channel,
  const mithl_trigger_t * msg, void * user){
  ((RoboteqADMController*)user)->HandleReflashAll(msg);
}

static void config_modification_ack_handler(bool ack, void * extra){
  ((RoboteqADMController*)extra)->HandleConfigModificationAck(ack);
}
static void telemetry_string_ack_handler(bool ack, void * extra){
  ((RoboteqADMController*)extra)->HandleTelemetryStringAck(ack);
}

// maybe better with a macro but whatever, this saves some typing...
template <typename type>
static inline void copy_conditional_helper(type * dest, const type * source){
  if (*source != -2) *dest = *source;
}
static void copy_known_elements(mithl_adm_config_t * dest,
                                const mithl_adm_config_t * source){
  // our command was committed, copy over the pending changes
  copy_conditional_helper<int8_t>(&(dest->estop), &(source->estop));

  copy_conditional_helper<float>(&(dest->command), &(source->command));
  copy_conditional_helper<int8_t>(&(dest->mode), &(source->mode));
  copy_conditional_helper<float>(&(dest->current_lim), &(source->current_lim));
  copy_conditional_helper<float>(&(dest->Kp), &(source->Kp));
  copy_conditional_helper<float>(&(dest->Kd), &(source->Kd));
  copy_conditional_helper<float>(&(dest->Ki), &(source->Ki));
  copy_conditional_helper<int8_t>(&(dest->integral_cap), &(source->integral_cap));

  copy_conditional_helper<int32_t>(&(dest->max_rpm), &(source->max_rpm));
  copy_conditional_helper<int8_t>(&(dest->max_power_forward), &(source->max_power_forward));
  copy_conditional_helper<int8_t>(&(dest->max_power_reverse), &(source->max_power_reverse));

  copy_conditional_helper<int8_t>(&(dest->n_poles), &(source->n_poles));

  copy_conditional_helper<int8_t>(&(dest->encoder_usage), &(source->encoder_usage));
  copy_conditional_helper<int8_t>(&(dest->switching_mode), &(source->switching_mode));
  copy_conditional_helper<int8_t>(&(dest->sinusoidal_mode), &(source->sinusoidal_mode));

  copy_conditional_helper<int32_t>(&(dest->encoder_pulse_per_rev), &(source->encoder_pulse_per_rev));
  copy_conditional_helper<int32_t>(&(dest->encoder_low_count_limit), &(source->encoder_low_count_limit));
  copy_conditional_helper<int32_t>(&(dest->encoder_high_count_limit), &(source->encoder_high_count_limit));

  copy_conditional_helper<int8_t>(&(dest->closed_loop_error_detection), &(source->closed_loop_error_detection));
  copy_conditional_helper<int8_t>(&(dest->closed_loop_feedback_sensor), &(source->closed_loop_feedback_sensor));

  copy_conditional_helper<float>(&(dest->default_pos), &(source->default_pos));
  copy_conditional_helper<float>(&(dest->default_vel), &(source->default_vel));
  copy_conditional_helper<float>(&(dest->default_current), &(source->default_current));
}

static bool check_if_buffer_atoi_safe(const char * buffer) {
  const char * checker = buffer;
  while (*checker != 0){
    if ( !(*checker >= '0' || *checker <= '9') && (*checker != '-' && *checker != '+') ){
      return false;
    }
    checker++;
  }
  return true;
}

// reads the first N unsigned integers from an array, where they are
// separated by colons and ultimately terminated with _, \r, or 0.
// modifies the array temporarily but fixes it by return time
// drops -2 into the array where numbers could not be read due to syntax issues
// and -20 where the numbers can't be read because there weren't that many
// numbers
static void read_uints_from_array(char * start, int * output_array, int N){
  int chan = 0;
  char * begin_this_num = start;
  char * end_this_num = start;
  bool done = false;
  // step through each number
  while (chan < N && !done){
    // find end of this number
    while (*end_this_num != ':' && *end_this_num != '\r' && *end_this_num != '_' && *end_this_num != 0){
      end_this_num++;
    }
    // if we hit sometihng that wasn't a colon, no channels left after this number
    if (*end_this_num != ':')
      done = true;
    
    // null-terminate the number for a second, but be ready to fix it
    char old = *end_this_num;
    *end_this_num = 0;
    
    // sanity check that number is composed of numbers
    bool safe = check_if_buffer_atoi_safe(begin_this_num);

    // and parse it into an integer 
    if (safe)
      output_array[chan] = atoi(begin_this_num);
    else
      output_array[chan] = -2;

    // move onto next number after repairing this one
    *end_this_num = old;
    chan++;
    end_this_num++;
    begin_this_num = end_this_num;
  }

  while (chan < N){
    output_array[chan] = -20;
    chan++;
  }

}


RoboteqADMController::RoboteqADMController(UARTClass * serial, zcm_t * zcm, char * name, int case_temp_pin, mithl_adm_config_t * default_config) :
        initialized_(false),
        serial_(serial),
        zcm_(zcm),
        case_temp_pin_(case_temp_pin)
{
  Reset(default_config);
  sprintf(config_set_channel_, "_RADM_%s_SETCONF", name);
  sprintf(config_cur_channel_, "_RADM_%s_CURCONF", name);
  sprintf(status_cur_channel_, "_RADM_%s_STATUS", name);
  sprintf(reflash_channel_, "_RADM_%s_REFLASH", name);
  sprintf(estop_channel_, "_ESTOP");

  mithl_adm_config_t_subscribe(zcm_, config_set_channel_, &adm_config_handler, this);
  mithl_trigger_t_subscribe(zcm_, estop_channel_, &estop_handler, this);
  mithl_trigger_t_subscribe(zcm_, reflash_channel_, &reflash_all_handler, this);
}

void reset_config(mithl_adm_config_t * config){
  config->estop = -2;
  config->command = -2;
  config->mode = -2;
  config->current_lim = -2;
  config->Kp = -2;
  config->Kd = -2;
  config->Ki = -2;
  config->integral_cap = -2;
  config->default_pos = -2;
  config->default_vel = -2;
  config->default_current = -2;
  config->closed_loop_error_detection = -2;
  config->closed_loop_feedback_sensor = -2;
  config->max_rpm = -2;
  config->max_power_forward = -2;
  config->max_power_reverse = -2;
  config->n_poles = -2;
  config->encoder_usage = -2;
  config->switching_mode = -2;
  config->sinusoidal_mode = -2;
  config->encoder_pulse_per_rev = -2;
  config->encoder_low_count_limit = -2;
  config->encoder_high_count_limit = -2;  
}

void reset_status(mithl_adm_status_t * status, char * receive_buffer){
  for (int i=0; i < 2; i++){
    status->current_motor[i] = -2;
    status->current_battery[i] = -2;
    status->rpm[i] = -2;
  }
  for (int i=0; i < 3; i++)
    status->voltage[i] = -2;
  for (int i=0; i < 3; i++)
    status->temp[i] = -2;

  status->unexpected_ack = false;
  status->receive_buffer = receive_buffer;

  status->watchdog_timeout = false;
  status->fault_flags=-1;
  status->runtime_flags[0] = -2;
  status->runtime_flags[1] = -2;
  status->runtime_flags[1] = -2;
  status->status_flags  = -2;
}

void RoboteqADMController::Reset(mithl_adm_config_t * default_config) {
  // reset desired config to the supplied default
  memcpy(&latest_desired_config, default_config, sizeof(mithl_adm_config_t));
  
  if (case_temp_pin_ != -1)
    pinMode(case_temp_pin_, INPUT);
  // however, status and latest config is unknown
  reset_config(&latest_config);
  memcpy(&tentative_config, &latest_config, sizeof(mithl_adm_config_t));
  reset_status(&latest_status, receive_buf_);
  
  waiting_for_ack_ = false;
  receive_buf_ptr_ = 0;
  receive_buf_[ROBOTEQ_ADM_MAX_COMMAND_LEN] = 0;

  round_robin_start_ = 0;
  telemetry_ack_received_ = false;

  last_runtime_flag_channel_ = 0;

  latest_status.case_temp = -2;

  // init state
  last_report_time_ = get_current_time();
  last_fault_flag_read_ = get_current_time();
  last_runtime_flag_1_read_ = get_current_time();
  last_runtime_flag_2_read_ = get_current_time();
  last_status_flag_read_ = get_current_time();
  last_motor_current_read_ = get_current_time();
  last_battery_current_read_ = get_current_time();
  last_rpm_read_ = get_current_time();
  last_voltage_read_ = get_current_time();
  last_temp_read_ = get_current_time();
  last_case_temp_read_ = get_current_time();
  last_setpoint_write_ = get_current_time();
  last_received_setpoint_ = get_current_time() - 2*SETPOINT_WATCHDOG_TIME;
  last_heard_from_adm_ = get_current_time() - 2*HEARD_FROM_ADM_WATCHDOG_TIMEOUT;

  // requires comms at this rate
  serial_->begin((uint32_t)115200);

  //serial_->write("%RESET 321654987_");

  double wait_start = get_current_time();
  while (get_current_time() - wait_start < 0.1){
    /*
    while (serial_->available()){
      Serial.write(serial_->read());
    }
    Serial.write("\n");
    */
    //delay(100);
  }

  // reset the controller as if it had been power cycled -- get back to reasonably known state
  initialized_ = true;
}

void RoboteqADMController::Update() {
  // todo: performCommandWatchdogging
  HandleReceive();

  if (waiting_for_ack_ && get_current_time() - command_send_time_ > ACK_TIMEOUT){
    waiting_for_ack_ = false;
    if (ack_callback_){
      ack_callback_(false, ack_callback_extra_);
      ack_callback_ = NULL;
    }
  }

  latest_status.last_received_setpoint = last_received_setpoint_;
  latest_status.watchdog_elapsed = get_current_time() - last_received_setpoint_;
  if (get_current_time() - last_received_setpoint_ > SETPOINT_WATCHDOG_TIME){
    // force estop and setpoint to 0
    latest_desired_config.estop = 1; 
    latest_desired_config.command = 0.0;
    latest_status.watchdog_timeout = true;
  } else {
    latest_status.watchdog_timeout = false;
  }

  if (case_temp_pin_ > 0 && get_current_time() - last_case_temp_read_ > CASE_TEMP_READ_PERIOD){
    // read case temp pin
    last_case_temp_read_ = get_current_time();
    latest_status.case_temp = (((float)analogRead(case_temp_pin_))*3.3/1024.) * (1000. / 10.); // 10mV / deg C
  } else {
    latest_status.case_temp = -2;
  }

  if (!waiting_for_ack_){

    // take priority on estop setting
    if (latest_desired_config.estop != latest_config.estop && latest_desired_config.estop == 1){
      SetESTOP();
    }

    int offset = 0;
    // see if we have a config to update... this is so ugly, can we iterate somehow?
    // I feel strongly that there's a refactoring that makes this beautiful and easy
    // round robin and offset trickery is so that if one s*etter fails, the others
    // get opportuntities to be attempted anyway
    #define NUM_RR_CASES 31
    while (!waiting_for_ack_ && offset < NUM_RR_CASES){
      switch ((offset + round_robin_start_) % NUM_RR_CASES){
        case 0:
          if (latest_desired_config.command != -2 && 
              ((get_current_time() - last_setpoint_write_ > SETPOINT_WRITE_PERIOD) ||
                latest_desired_config.command != latest_config.command))
            SetSetpoint(0, latest_desired_config.command);
          break;
        case 1:
          if (latest_desired_config.mode != -2 && 
              latest_desired_config.mode != latest_config.mode)
            SetOperatingMode(0, (OperatingMode) latest_desired_config.mode);
          break;
        case 2:
          if (latest_desired_config.current_lim != -2 &&
              latest_desired_config.current_lim != latest_config.current_lim)
            SetAmpLimit(0, latest_desired_config.current_lim);
          break;
        case 3:
          if (!telemetry_ack_received_)
            SetTelemetryString();
          break;
        case 4:
          if (get_current_time() - last_fault_flag_read_ > FLAG_READ_PERIOD)
            ReqFaultFlag();
          break;
        case 5:
          if (get_current_time() - last_motor_current_read_ > CURRENT_READ_PERIOD)
            ReqMotorCurrent();
          break;
        case 6:
          if (get_current_time() - last_voltage_read_ > VOLTAGE_READ_PERIOD)
            ReqVoltage();
          break;
        case 7:
          if (get_current_time() - last_temp_read_ > TEMP_READ_PERIOD)
            ReqTemp();
          break;
        case 8:
          if (get_current_time() - last_temp_read_ > TEMP_READ_PERIOD)
            ReqTemp();
          break;
        case 9:
          if (get_current_time() - last_runtime_flag_1_read_ > FLAG_READ_PERIOD)
            ReqRuntimeFlag(0);
          break;
        case 10:
          if (get_current_time() - last_runtime_flag_2_read_ > FLAG_READ_PERIOD)
            ReqRuntimeFlag(1);
          break;
        case 11:
          if (get_current_time() - last_status_flag_read_ > FLAG_READ_PERIOD)
            ReqStatusFlag();
          break;
        case 12:
          if (latest_desired_config.closed_loop_error_detection != -2 && 
              latest_desired_config.closed_loop_error_detection != latest_config.closed_loop_error_detection)
            SetCLEDMode(0, (CLEDMode) latest_desired_config.closed_loop_error_detection);
          break;
        case 13:
          if (latest_desired_config.Kp != -2 && 
              latest_desired_config.Kp != latest_config.Kp)
            SetPIDKp(0, latest_desired_config.Kp);
          break;
        case 14:
          if (latest_desired_config.Kd != -2 && 
              latest_desired_config.Kd != latest_config.Kd)
            SetPIDKd(0, latest_desired_config.Kd);
          break;        
        case 15:
          if (latest_desired_config.Ki != -2 && 
              latest_desired_config.Ki != latest_config.Ki)
            SetPIDKi(0, latest_desired_config.Ki);
          break;    
        case 16:
          if (latest_desired_config.integral_cap != -2 && 
              latest_desired_config.integral_cap != latest_config.integral_cap)
            SetPIDIntegralCap(0, latest_desired_config.integral_cap);
          break;
        case 17:
          if (get_current_time() - last_battery_current_read_ > CURRENT_READ_PERIOD)
            ReqBatteryCurrent();
          break;    
        case 18:
          if (latest_desired_config.max_rpm != -2 && 
              latest_desired_config.max_rpm != latest_config.max_rpm)
            SetMaxRPM(0, latest_desired_config.max_rpm);
          break;    
        case 19:
          if (latest_desired_config.max_power_forward != -2 && 
              latest_desired_config.max_power_forward != latest_config.max_power_forward)
            SetMaxPowerForward(0, latest_desired_config.max_power_forward);
          break;    
        case 20:
          if (latest_desired_config.max_power_reverse != -2 && 
              latest_desired_config.max_power_reverse != latest_config.max_power_reverse)
            SetMaxPowerReverse(0, latest_desired_config.max_power_reverse);
          break;    
        case 21:
          if (latest_desired_config.n_poles != -2 && 
              latest_desired_config.n_poles != latest_config.n_poles)
            SetNumPoles(latest_desired_config.n_poles);
          break;   
        case 22:
          if (latest_desired_config.encoder_usage != -2 && 
              latest_desired_config.encoder_usage != latest_config.encoder_usage)
            SetEncoderUsage(0, (EncoderUsage) latest_desired_config.encoder_usage);
          break;   
        case 23:
          if (latest_desired_config.sinusoidal_mode != -2 && 
              latest_desired_config.sinusoidal_mode != latest_config.sinusoidal_mode)
            SetSinusoidalMode(0, (SinusoidalMode) latest_desired_config.sinusoidal_mode);
          break;    
        case 24:
          if (latest_desired_config.encoder_pulse_per_rev != -2 && 
              latest_desired_config.encoder_pulse_per_rev != latest_config.encoder_pulse_per_rev)
            SetEncoderPulsePerRev(0, latest_desired_config.encoder_pulse_per_rev);
          break;    
        case 25:
          if (latest_desired_config.encoder_low_count_limit != -2 && 
              latest_desired_config.encoder_low_count_limit != latest_config.encoder_low_count_limit)
            SetEncoderLowCountLimit(0, latest_desired_config.encoder_low_count_limit);
          break;    
        case 26:
          if (latest_desired_config.encoder_high_count_limit != -2 && 
              latest_desired_config.encoder_high_count_limit != latest_config.encoder_high_count_limit)
            SetEncoderHighCountLimit(0, latest_desired_config.encoder_high_count_limit);
          break;    
        case 27:
          if (latest_desired_config.closed_loop_feedback_sensor != -2 && 
              latest_desired_config.closed_loop_feedback_sensor != latest_config.closed_loop_feedback_sensor)
            SetCLFeedbackSensor((CLFeedbackSensor) latest_desired_config.closed_loop_feedback_sensor);
          break;  
        case 28:
          if (get_current_time() - last_rpm_read_ > RPM_READ_PERIOD)
            ReqSpeed(0);
          break;  
        case 29:
          if (latest_desired_config.estop == 0 && 
              latest_desired_config.estop != latest_config.estop)
            ReleaseESTOP();
        case 30:
          if (latest_desired_config.switching_mode != -2 && 
              latest_desired_config.switching_mode != latest_config.switching_mode)
            SetSwitchingMode(0, (SwitchingMode) latest_desired_config.switching_mode);
          break;
        default:
          break;
      }
      offset++;    
      HandleReceive();
    }
    round_robin_start_ = ((offset + round_robin_start_) % NUM_RR_CASES);
  }

/*
  performRS232ReceiveHandling
  performCommandSending
  */

  // Update fields we're reading based on estops
  double now = get_current_time();
  if (now - last_heard_from_adm_ > HEARD_FROM_ADM_WATCHDOG_TIMEOUT){
    reset_config(&latest_config);
    reset_status(&latest_status, receive_buf_);
  }

  if (now - last_report_time_ > REPORT_PERIOD){
    last_report_time_ = get_current_time();
    latest_config.utime = now * 1000 * 1000;
    latest_status.utime = now * 1000 * 1000;
    mithl_adm_config_t_publish(zcm_, config_cur_channel_, &latest_config);
    mithl_adm_status_t_publish(zcm_, status_cur_channel_, &latest_status);
  }
}

void RoboteqADMController::HandleReceive() {
  int num_handled = 0;
  while (num_handled < 1000 and serial_->available()){
    num_handled += 1;
    char c = serial_->read();
    // this shouldn't happen but we can be safe
    if (c == -1) break;

    // try to parse messages coming back
    if (c == '_' || c == '\r'){
      receive_buf_[receive_buf_ptr_] = 0;
      ParseReceiveBuffer();
      receive_buf_ptr_ = 0;
      latest_status.receive_overflow = false;
    } else {
      if (receive_buf_ptr_ >= ROBOTEQ_ADM_MAX_COMMAND_LEN){
        latest_status.receive_overflow = true;
        receive_buf_ptr_ = 0; // wrap back around and report error.
      }
      receive_buf_[receive_buf_ptr_] = c;
      receive_buf_ptr_++;
    }
  }
}

void RoboteqADMController::ParseReceiveBuffer(){
  bool handled = false;
  // is this receive bufer an ack?
  if (waiting_for_ack_){
    if (receive_buf_ptr_ == 1 && (receive_buf_[0] == '+' || receive_buf_[0] == '-')){
      waiting_for_ack_ = 0;
      if (ack_callback_){
        ack_callback_(receive_buf_[0] == '+', ack_callback_extra_);
        ack_callback_ = NULL;
      }
      handled = true;
    }
  }

  if (!handled){
    bool success = false;
    // look for equals signs, which indicate a result we care about, and extract from that
    for (int i=0; i < receive_buf_ptr_; i++){
      if (receive_buf_[i] == '='){
        // Fault flags FF=#
        if ( (i >= 2) && (i + 1 < receive_buf_ptr_) && // bounds
             receive_buf_[i-2] == 'F' && receive_buf_[i-1] == 'F') // character match
        {
          // sanity-check buffer
          if (check_if_buffer_atoi_safe(receive_buf_ + i + 1)){
            latest_status.fault_flags = atoi(receive_buf_ + i + 1);
            last_fault_flag_read_ = get_current_time();
            success = true;
          }
          else
            latest_status.fault_flags = -1;
        } else 
        // Runtime flags FM=# for a given channel
        if ( (i >= 2) && (i + 1 < receive_buf_ptr_) && // bounds
             receive_buf_[i-2] == 'F' && receive_buf_[i-1] == 'M') // character match
        {
          // sanity-check buffer
          if (last_runtime_flag_channel_ >= 0 && last_runtime_flag_channel_ <= 1)
            if (check_if_buffer_atoi_safe(receive_buf_ + i + 1)){
              latest_status.runtime_flags[last_runtime_flag_channel_] = atoi(receive_buf_ + i + 1);
              if (last_runtime_flag_channel_ == 1){
                last_runtime_flag_1_read_ = get_current_time();
                success = true;
              } else {
                last_runtime_flag_2_read_ = get_current_time();
                success = true;
              }
            } else {
              latest_status.runtime_flags[last_runtime_flag_channel_] = -1;
            }
        } else 
        // Status flags FS=#
        if ( (i >= 2) && (i + 1 < receive_buf_ptr_) && // bounds
             receive_buf_[i-2] == 'F' && receive_buf_[i-1] == 'S') // character match
        {
          // sanity-check buffer
          if (check_if_buffer_atoi_safe(receive_buf_ + i + 1)){
            latest_status.status_flags = atoi(receive_buf_ + i + 1);
            last_status_flag_read_ = get_current_time();
            success = true;
          }
          else
            latest_status.status_flags = -1;
        } else 
        // Motor current A=#:#:#...
        if ((i >= 1) && (i + 1 < receive_buf_ptr_) && // bounds
             receive_buf_[i-1] == 'A') // character match
        {
          int vals[2];
          read_uints_from_array(receive_buf_ + i + 1, vals, 2);
          for (int j=0; j < 2; j++){
            latest_status.current_motor[j] = ((float)vals[j]) / 10.;
          }
          last_motor_current_read_ = get_current_time();
          success = true;
        } else 
        // Battery current BA=#:#:#...
        if ((i >= 2) && (i + 1 < receive_buf_ptr_) && // bounds
            receive_buf_[i-2] == 'B' && receive_buf_[i-1] == 'A') // character match
        {
          int vals[2];
          read_uints_from_array(receive_buf_ + i + 1, vals, 2);
          for (int j=0; j < 2; j++){
            latest_status.current_battery[j] = ((float)vals[j]) / 10.;
          }
          last_battery_current_read_ = get_current_time();
          success = true;
        }else 
        // Voltage V=#:#:#...
        if ((i >= 1) && (i + 1 < receive_buf_ptr_) && // bounds
             receive_buf_[i-1] == 'V') // character match
        {
          int vals[3];
          read_uints_from_array(receive_buf_ + i + 1, vals, 3);
          for (int j=0; j < 3; j++){
            if (j == 2)
              // this one's in mV
              latest_status.voltage[j] = ((float)vals[j]) / 1000.;
            else 
              latest_status.voltage[j] = ((float)vals[j]) / 10.;
          }
          last_voltage_read_ = get_current_time();
          success = true;
        } else 
        // Temperatures T=#:#:#...
        if ((i >= 1) && (i + 1 < receive_buf_ptr_) && // bounds
             receive_buf_[i-1] == 'T') // character match
        {
          int vals[3];
          read_uints_from_array(receive_buf_ + i + 1, vals, 3);
          for (int j=0; j < 3; j++){
            latest_status.temp[j] = ((float)vals[j]);
          }
          last_temp_read_ = get_current_time();
          success = true;
        } else 
        // Speed S=#:#:#...
        if ( (i >= 1) && (i + 1 < receive_buf_ptr_) && // bounds
             receive_buf_[i-1] == 'S') // character match
        {
          int vals[2];
          read_uints_from_array(receive_buf_ + i + 1, vals, 2);
          for (int j=0; j < 2; j++){
            latest_status.rpm[j] = vals[j];
          }
          last_rpm_read_ = get_current_time();
          success = true;
        }
        break;
      } // if
    } // for
    if (success){
      last_heard_from_adm_ = get_current_time();
    }
  } // if !handled
}

// sorry about the extra void * needed... hard to get function
// pointers to member functions of classes. Refactoring could fix
// this, or figuring out member pointers, but this solution
// isn't the worst thing ever...
void RoboteqADMController::SubmitCommandBuf(char * command_buf, void (*ack_callback)(bool, void *), void * extra){
  // if another command is pending, fail us out immediately.
  if (waiting_for_ack_){
    ack_callback(false, extra);
  } else {
    strncpy(command_buf_, command_buf, ROBOTEQ_ADM_MAX_COMMAND_LEN);
    ack_callback_ = ack_callback;
    ack_callback_extra_ = extra;
    serial_->write(command_buf_);
    command_send_time_ = get_current_time();
    waiting_for_ack_ = true;
  }
}



void RoboteqADMController::HandleConfigModificationAck(bool ack)
{
  if (ack){
    copy_known_elements(&latest_config, &tentative_config);
  } else {
    // our command was not commited, so hide the changes that didn't happen
    memcpy(&tentative_config, &latest_config, sizeof(mithl_adm_config_t));
  }
  reset_config(&tentative_config);
}

void RoboteqADMController::SetSetpoint(unsigned int channel, float setpoint){
  if (fabs(setpoint)>1.0001) return;

  // step from current config towards setpoint if known
  // "current config" if unknown, then submit 0
  if (latest_config.command < -1.0001){
    setpoint = 0.0;
  } 
  // if our new setpoint is the opposite sign of the current command (allowing some
  // small threshold near zero where we don't care), go to zero first.  
  else if (setpoint >= 0.01 and latest_config.command <= -0.01){
    setpoint = 0.0;
  } else if (setpoint <= -0.01 and latest_config.command >= 0.01){
    setpoint = 0.0;
  } // if this setpoint moves outwards from zero, step it by at most MAX_SETPONT
  else if (fabs(setpoint) > fabs(latest_config.command) + MAX_SETPOINT_UPWARDS_DIFF){
    if (setpoint > latest_config.command){
      setpoint = latest_config.command + MAX_SETPOINT_UPWARDS_DIFF;
    } else {
      setpoint = latest_config.command - MAX_SETPOINT_UPWARDS_DIFF;
    }
  }


  // Small negative values default to zero
  // Maybe unnecessary, but hard to tell what the string formatter would
  // do otherwise...
  int int_command_to_submit = 0;
  if (fabs(setpoint)>0.0001)
    int_command_to_submit = (int)(1000.0*setpoint);

  tentative_config.command = setpoint;
  char command_buf[20];
  sprintf(command_buf, "!G %u %d_", channel, int_command_to_submit);
  SubmitCommandBuf(command_buf, &config_modification_ack_handler, (void *)this);
}

void RoboteqADMController::SetAmpLimit(unsigned int channel, float limit){
  if (limit < 10.00 || limit > 50.0001) return;
  tentative_config.current_lim = limit;
  char command_buf[30];
  sprintf(command_buf, "^ALIM %u %u_", channel, (unsigned int)(limit*10.0));
  SubmitCommandBuf(command_buf, &config_modification_ack_handler, (void *)this);
}

void RoboteqADMController::SetOperatingMode(unsigned int channel, OperatingMode mode){
  if (mode < 0 || mode >= 6) return;
  tentative_config.mode = mode;
  char command_buf[30];
  sprintf(command_buf, "^MMOD %u %u_", channel, (unsigned int)mode);
  SubmitCommandBuf(command_buf, &config_modification_ack_handler, (void *)this);
}

void RoboteqADMController::SetCLEDMode(unsigned int channel, CLEDMode mode){
  if (mode < 0 || mode >= 4) return;
  tentative_config.closed_loop_error_detection = mode;
  char command_buf[30];
  sprintf(command_buf, "^CLERD %u %u_", channel, (unsigned int)mode);
  SubmitCommandBuf(command_buf, &config_modification_ack_handler, (void *)this);
}

void RoboteqADMController::SetCLFeedbackSensor(CLFeedbackSensor sensor){
  if (sensor < 0 || sensor > 1) return;
  tentative_config.closed_loop_feedback_sensor = sensor;
  char command_buf[30];
  sprintf(command_buf, "^BLFB %u_", (unsigned int)sensor);
  SubmitCommandBuf(command_buf, &config_modification_ack_handler, (void *)this);
}

void RoboteqADMController::SetPIDKp(unsigned int channel, float Kp){
  if (Kp < -0.00001) return;
  tentative_config.Kp = Kp;
  char command_buf[30];
  sprintf(command_buf, "^KP %u %u_", channel, (unsigned int)(Kp*10.0)); 
  SubmitCommandBuf(command_buf, &config_modification_ack_handler, (void *)this);
}

void RoboteqADMController::SetPIDKd(unsigned int channel, float Kd){
  if (Kd < -0.00001) return;
  tentative_config.Kd = Kd;
  char command_buf[30];
  sprintf(command_buf, "^KD %u %u_", channel, (unsigned int)(Kd*10.0)); 
  SubmitCommandBuf(command_buf, &config_modification_ack_handler, (void *)this);
}
void RoboteqADMController::SetPIDKi(unsigned int channel, float Ki){
  if (Ki < -0.00001) return;
  tentative_config.Ki = Ki;
  char command_buf[30];
  sprintf(command_buf, "^KI %u %u_", channel, (unsigned int)(Ki*10.0)); 
  SubmitCommandBuf(command_buf, &config_modification_ack_handler, (void *)this);
}
void RoboteqADMController::SetPIDIntegralCap(unsigned int channel, int integral_cap){
  if (integral_cap < 0 || integral_cap > 100) return;
  tentative_config.integral_cap = integral_cap;
  char command_buf[30];
  sprintf(command_buf, "^ICAP %u %u_", channel, (unsigned int)integral_cap);
  SubmitCommandBuf(command_buf, &config_modification_ack_handler, (void *)this);
}
void RoboteqADMController::SetMaxRPM(unsigned int channel, int max_rpm){
  if (max_rpm < 10 || max_rpm > 65000) return;
  tentative_config.max_rpm = max_rpm;
  char command_buf[30];
  sprintf(command_buf, "^MXRPM %u %u_", channel, (unsigned int)max_rpm);
  SubmitCommandBuf(command_buf, &config_modification_ack_handler, (void *)this);
}
void RoboteqADMController::SetMaxPowerForward(unsigned int channel, int max_power){
  if (max_power < 25 || max_power > 100) return;
  tentative_config.max_power_forward = max_power;
  char command_buf[30];
  sprintf(command_buf, "^MXPF %u %u_", channel, (unsigned int)max_power);
  SubmitCommandBuf(command_buf, &config_modification_ack_handler, (void *)this);
}
void RoboteqADMController::SetMaxPowerReverse(unsigned int channel, int max_power){
  if (max_power < 25 || max_power > 100) return;
  tentative_config.max_power_reverse = max_power;
  char command_buf[30];
  sprintf(command_buf, "^MXPR %u %u_", channel, (unsigned int)max_power);
  SubmitCommandBuf(command_buf, &config_modification_ack_handler, (void *)this);
}

void RoboteqADMController::SetTelemetryString(){
  char command_buf[30];
  sprintf(command_buf, "^TELS \"?A:?V:?T:?S:#200\"_");
  SubmitCommandBuf(command_buf, &telemetry_string_ack_handler, (void *)this);
}
void RoboteqADMController::HandleTelemetryStringAck(bool ack) { 
  telemetry_ack_received_ = ack;
}

// Brushless motor config
void RoboteqADMController::SetNumPoles(int n_poles){
  if (n_poles < 2 || n_poles > 99) return;
  tentative_config.n_poles = n_poles;
  char command_buf[30];
  sprintf(command_buf, "^BPOL %u_", (unsigned int)n_poles);
  SubmitCommandBuf(command_buf, &config_modification_ack_handler, (void *)this);
}

void RoboteqADMController::SetEncoderUsage(unsigned int channel, EncoderUsage usage){
  if (usage < 0 || usage > 2) return;
  tentative_config.encoder_usage = usage;
  char command_buf[30];
  // TODO(gizatt): hardcoded to use motor 1 on the given channel
  sprintf(command_buf, "^EMOD %u %u_", channel, 16+(unsigned int)usage);
  SubmitCommandBuf(command_buf, &config_modification_ack_handler, (void *)this);
}
void RoboteqADMController::SetSwitchingMode(unsigned int channel, SwitchingMode mode){
  if (mode < 0 || mode > 2) return;
  tentative_config.switching_mode = mode;
  char command_buf[30];
  sprintf(command_buf, "^BMOD %u %u_", channel, (unsigned int)mode);
  SubmitCommandBuf(command_buf, &config_modification_ack_handler, (void *)this);
}
void RoboteqADMController::SetSinusoidalMode(unsigned int channel, SinusoidalMode mode){
  if (mode < 0 || mode > 5) return;
  tentative_config.sinusoidal_mode = mode;
  char command_buf[30];
  sprintf(command_buf, "^BFBK %u %u_", channel, (unsigned int)mode);
  SubmitCommandBuf(command_buf, &config_modification_ack_handler, (void *)this);
}
void RoboteqADMController::SetEncoderPulsePerRev(unsigned int channel, int ppr){
  if (ppr < 1 || ppr > 5000) return;
  tentative_config.encoder_pulse_per_rev = ppr;
  char command_buf[30];
  sprintf(command_buf, "^EPPR %u %u_", channel, (unsigned int)ppr);
  SubmitCommandBuf(command_buf, &config_modification_ack_handler, (void *)this);
}
void RoboteqADMController::SetEncoderLowCountLimit(unsigned int channel, int count_lim){
  tentative_config.encoder_low_count_limit = count_lim;
  char command_buf[30];
  sprintf(command_buf, "^ELL %u %u_", channel, count_lim);
  SubmitCommandBuf(command_buf, &config_modification_ack_handler, (void *)this);
}
void RoboteqADMController::SetEncoderHighCountLimit(unsigned int channel, int count_lim){
  tentative_config.encoder_high_count_limit = count_lim;
  char command_buf[30];
  sprintf(command_buf, "^EHL %u %d_", channel, count_lim);
  SubmitCommandBuf(command_buf, &config_modification_ack_handler, (void *)this);
}


void RoboteqADMController::ReqFaultFlag(){
  char command_buf[30];
  sprintf(command_buf, "?FF_");
  SubmitCommandBuf(command_buf, NULL, NULL);
}
void RoboteqADMController::ReqRuntimeFlag(unsigned int channel){
  char command_buf[30];
  last_runtime_flag_channel_ = channel;
  sprintf(command_buf, "?FM %u_", channel);
  SubmitCommandBuf(command_buf, NULL, NULL);
}
void RoboteqADMController::ReqStatusFlag(){
  char command_buf[30];
  sprintf(command_buf, "?FS_");
  SubmitCommandBuf(command_buf, NULL, NULL);
}

void RoboteqADMController::ReqMotorCurrent(){
  char command_buf[30];
  sprintf(command_buf, "?A_");
  SubmitCommandBuf(command_buf, NULL, NULL);
}
void RoboteqADMController::ReqBatteryCurrent(){
  char command_buf[30];
  sprintf(command_buf, "?BA_");
  SubmitCommandBuf(command_buf, NULL, NULL);
}

void RoboteqADMController::ReqVoltage(){
  char command_buf[30];
  sprintf(command_buf, "?V_");
  SubmitCommandBuf(command_buf, NULL, NULL);
}

void RoboteqADMController::ReqTemp(){
  char command_buf[30];
  sprintf(command_buf, "?T_");
  SubmitCommandBuf(command_buf, NULL, NULL);
}

void RoboteqADMController::ReqSpeed(unsigned int channel){
  char command_buf[30];
  sprintf(command_buf, "?S %u_", channel);
  SubmitCommandBuf(command_buf, NULL, NULL);
}

void RoboteqADMController::SetESTOP(){
  tentative_config.estop = 1;
  SubmitCommandBuf("!EX_", &config_modification_ack_handler, (void *)this);
}
void RoboteqADMController::ReleaseESTOP(){
  tentative_config.estop = 0;
  SubmitCommandBuf("!MG_", &config_modification_ack_handler, (void *)this);
}

void RoboteqADMController::HandleConfig(const mithl_adm_config_t * msg){
  copy_known_elements(&latest_desired_config, msg);
  if (fabs(msg->command + 2.0) >= 0.01) {
    last_received_setpoint_ = get_current_time();
  }
}
void RoboteqADMController::HandleReflashAll(const mithl_trigger_t * msg){
  reset_config(&latest_config);
}
void RoboteqADMController::HandleEstop(const mithl_trigger_t * msg){
  latest_desired_config.estop = 1;
}
