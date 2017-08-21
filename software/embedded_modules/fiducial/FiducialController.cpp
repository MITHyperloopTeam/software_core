#include "FiducialController.h"
#include "arduino_time_manager.h"
#include "fiducial_utils.hpp"

// DB DB DB
#include "Arduino.h"
static int flash_now(int blinkingLEDState) {
  digitalWrite(13, blinkingLEDState);
  if (blinkingLEDState == HIGH)
    return LOW;
  return HIGH;
}
static void flash_x_times(int x) {
  int blinkingLEDState = LOW;
  for (int i=0; i<2*x + 1; i++) {
      blinkingLEDState = flash_now(blinkingLEDState);
      delay(150);
  }
}
// /DB /DB /DB

static char nib2chr[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

// #include <string>       // std
// #include <iomanip>      // std::setw
// #include <sstream>      // std::ostringstream

/* Annoying static stuff up here */
const mithl_fiducial_config_t FiducialController::the_empty_config = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
static void fiducial_config_handler(const zcm_recv_buf_t * rbuf, const char * channel,
              const mithl_fiducial_config_t * msg, void * user){
  ((FiducialController*)user)->HandleConfig(msg);
}
static void fiducial_teach_table_handler(const zcm_recv_buf_t * rbuf, const char * channel,
              const mithl_fiducial_teach_table_t * msg, void * user){
  ((FiducialController*)user)->HandleTeachTable(msg);
}

static void publish_string_to_lcm(zcm_t * zcm, const char * channel_name, char * c_str) {
  mithl_string_t msg;
  double now = get_current_time();
  msg.utime = now * 1000 * 1000;
  msg.data = c_str;
  mithl_string_t_publish(zcm, channel_name, &msg);
}

/* Class really begins here */

FiducialController::FiducialController(UARTClass * serial, zcm_t * zcm, mithl_fiducial_config_t * default_config,
            mithl_fiducial_teach_table_t * default_teach_table) :
        initialized_(false),
        serial_(serial),
        zcm_(zcm)
{
  Reset(default_config, default_teach_table);
  mithl_fiducial_config_t_subscribe(zcm_, "_FIDUCIAL_SET_CONFIG", &fiducial_config_handler, this);
  mithl_fiducial_teach_table_t_subscribe(zcm_, "_FIDUCIAL_SET_TEACH_TABLE", &fiducial_teach_table_handler, this);
}

void FiducialController::Reset(mithl_fiducial_config_t * default_config, mithl_fiducial_teach_table_t * default_teach_table) {
  // reset desired config & TT to the supplied default
  memcpy(&latest_desired_config, default_config, sizeof(mithl_fiducial_config_t));
  memcpy(&tentative_config, &latest_desired_config, sizeof(mithl_fiducial_config_t));

  memcpy(&latest_desired_teach_table, default_teach_table, sizeof(mithl_fiducial_teach_table_t));
  memcpy(&tentative_teach_table, &latest_desired_teach_table, sizeof(mithl_fiducial_teach_table_t));

  // known state
  strip_width = .05;
  strip_space_between = 1.0;

  // however, status and latest config is unknown
  latest_config_known_ = false;
  latest_config[0] = the_empty_config;
  latest_config[0].strip_width = strip_width;
  latest_config[0].strip_space_between = strip_space_between;
  latest_config[1] = the_empty_config;
  latest_config[1].strip_width = strip_width;
  latest_config[1].strip_space_between = strip_space_between;

  mithl_fiducial_teach_table_t empty_teach_table;
  empty_teach_table.utime = 0;
  // Clear all the rows of the empty teach table
  for (int i=0; i<31; i++) {
    mithl_fiducial_color_row_t row;

    row.utime =      0;
    row.color_x =    0;
    row.color_y =    0;
    row.cto =        0;
    row.color_int =  0;
    row.ito =        0;
    row.group =      0;
    row.hold =       0;

    empty_teach_table.rows[i] = row;
  }

  latest_teach_table_known_ = false;
  latest_teach_table[0] = empty_teach_table;
  latest_teach_table[1] = empty_teach_table;

  waiting_for_ack_ = false;
  waiting_for_ack_order_ = 0;
  waiting_for_ack_arg_ = 0;

  receive_buf_[FIDUCIAL_MAX_RECEIVE_LEN-1] = 0;
  frame_state_ = 0;
  max_chars_to_receive_ = FIDUCIAL_MAX_RECEIVE_LEN;

  receive_any_buf_[FIDUCIAL_MAX_RECEIVE_LEN-1] = 0;
  receive_any_index_ = 0;

  round_robin_start_ = 0;

  // init state
  last_report_time_ = get_current_time();
  last_color_report_time_ = get_current_time();
  command_send_time_ = get_current_time();

  // requires comms at this rate
  serial_->begin((uint32_t) FIDUCIAL_BAUD_RATE );

  // DB DB DB
  flash_x_times(5);
  delay(300);
  // /DB /DB /DB

  //serial_->write("%RESET 321654987_");

  double wait_start = get_current_time();
  while (get_current_time() - wait_start < 0.1){
    while (serial_->available()){
      Serial.write(serial_->read());
    }
    Serial.write("\n");
    delay(100);
  }

  // reset the controller as if it had been power cycled -- get back to reasonably known state
  initialized_ = true;
}


void FiducialController::Update() {
  // todo: performCommandWatchdogging
  HandleReceive();

  if (waiting_for_ack_ && get_current_time() - command_send_time_ > ACK_TIMEOUT){
    waiting_for_ack_ = false;

    // DB DB DB
    char * str;
    switch (waiting_for_ack_order_) {
      case 0:
        str = "0";
        break;
      case 1:
        str = "1";
        break;
      case 2:
        str = "2";
        break;
      case 8:
        str = "8";
        break;
      default:
        str = "???";
    }
    publish_string_to_lcm(zcm_, "_FIDUCIAL_ACK_TIMEOUT", str);
    // /DB /DB /DB

    // TODO: Report order number of missed ACK
  }

  if (!waiting_for_ack_){
    int offset = 0;
    // round robin and offset trickery is so that if one setter fails, the others
    // get opportuntities to be attempted anyway
    #define NUM_RR_CASES 10
    while (!waiting_for_ack_ && offset < NUM_RR_CASES){
      switch ((offset + round_robin_start_) % NUM_RR_CASES){
        case 0: // Send config set 0, if needed
        {
          unsigned int set_no = 0;
          if ((!latest_config_known_) || !(mithl_fid::equal_sensor_configs(&latest_desired_config, &(latest_config[set_no]))))
            // publish_string_to_lcm(zcm_, "_FIDUCIAL_WARN2_MESSAGE", "Invalid Config!");
            SetConfig(0, set_no, &latest_desired_config);
        }
        break;
        case 1: // Send config set 1, if needed
        {
          unsigned int set_no = 1;
          if ((!latest_config_known_) || !(mithl_fid::equal_sensor_configs(&latest_desired_config, &(latest_config[set_no]))))
            SetConfig(0, set_no, &latest_desired_config);
        }
        break;
        case 2: // Send teach table set 0, if needed
        {
          unsigned int set_no = 0;
          if ((!latest_teach_table_known_) || !(mithl_fid::equal_teach_tables(&latest_desired_teach_table, &(latest_teach_table[set_no]), latest_desired_config.maxcolno)))
            SetTeachTable(0, set_no, &latest_desired_teach_table);
        }
        break;
        case 3: // Send teach table set 1, if needed
        {
          unsigned int set_no = 1;
          //if ((!latest_teach_table_known_) || !(mithl_fid::equal_teach_tables(&latest_desired_teach_table, &(latest_teach_table[set_no]),  latest_desired_config.maxcolno)))
          //  SetTeachTable(0, set_no, &latest_desired_teach_table);
        }
        break;

        case 4: // Check config set 0, if needed
        {
          unsigned int set_no = 0;
          if ((!latest_config_known_) || !(mithl_fid::equal_sensor_configs(&latest_desired_config, &(latest_config[set_no]))))
            ReqConfig(0, set_no);
        }
        break;
        case 5: // Check config set 1, if needed
        {
          unsigned int set_no = 1;
          if ((!latest_config_known_) || !(mithl_fid::equal_sensor_configs(&latest_desired_config, &(latest_config[set_no]))))
            ReqConfig(0, set_no);
        }
        break;
        case 6: // Check teach table set 0, if needed
        {
          unsigned int set_no = 0;
          if ((!latest_teach_table_known_) || !(mithl_fid::equal_teach_tables(&latest_desired_teach_table, &(latest_teach_table[set_no]), latest_desired_config.maxcolno)))
            ReqTeachTable(0, set_no);
        }
        break;
        case 7: // Check teach table set 1, if needed
        {
          unsigned int set_no = 1;
          //if ((!latest_teach_table_known_) || !(mithl_fid::equal_teach_tables(&latest_desired_teach_table, &(latest_teach_table[set_no]), latest_desired_config.maxcolno)))
          //  ReqTeachTable(0, set_no);
        }
        break;
        case 8: // Request data reading from the sensor  // DB DB DB ; costs bandwidth; shouldn't have this in final pipeline
        {
          if (get_current_time() - last_color_report_time_ > COLOR_DATAVAL_READ_PERIOD) {
            last_color_report_time_ = get_current_time();
            ReqDataval(0);
          }
        }
        break;        
        case 9: // Publish some status  // costs bandwidth; slow this down for final
        {
          // publish_string_to_lcm(zcm_, "_FIDUCIAL_ARD_MESSAGE", "Controlling!");
        }
        break;
      }
      offset++;
    }
    round_robin_start_ = ((offset + round_robin_start_) % NUM_RR_CASES);
  }

  // periodically publish known state
  if (get_current_time() - last_report_time_ > REPORT_PERIOD){
    last_report_time_ = get_current_time();
    mithl_fiducial_config_t_publish(zcm_, "_FIDUCIAL_CUR_CONFIG", &(latest_config[0]));
    mithl_fiducial_teach_table_t_publish(zcm_, "_FIDUCIAL_CUR_TEACH_TABLE", &(latest_teach_table[0]));
    mithl_fiducial_config_t_publish(zcm_, "_FIDUCIAL_DESIRED_CONFIG", &(latest_desired_config));
    mithl_fiducial_teach_table_t_publish(zcm_, "_FIDUCIAL_DESIRED_TEACH_TABLE", &(latest_desired_teach_table));
  }
}

void FiducialController::HandleReceive() {
  while (serial_->available()){
    char c = serial_->read();

    {
      unsigned char my_c_str[1];
      my_c_str[0] = (unsigned char) c;
      //PublishBuffer((uint8_t*)my_c_str, 1, "RECV_BUFFER_SINGLE");
    } // DB DB DB DB

    if (receive_any_index_ > FIDUCIAL_MAX_COMMAND_LEN) {
      PublishBuffer((uint8_t*)receive_any_buf_, FIDUCIAL_MAX_COMMAND_LEN, "RECV_BUFFER_ANY");
      receive_any_index_ = 0;
    }
    receive_any_buf_[receive_any_index_] = c;
    receive_any_index_++;

    receive_buf_[frame_state_] = c;

    switch (frame_state_) { // State machine; looking for 0x55 to kick things off.
      case 0: // Case 0: Haven't seen 0x55 yet
        if (c == 0x55) {
          frame_state_++;
        } // otherwise it stays as frame_state_ = 0;
        break;
      default: // Case >0: Receiving data frame.

        if (frame_state_ == 7) { // Full header received. Can check it for consistency, then take what action needs taking
          uint8_t order = receive_buf_[1];

          uint16_t len_bottom = receive_buf_[4];
          uint16_t len_top = receive_buf_[5];
          uint16_t len = (len_top << 8) + len_bottom;

          max_chars_to_receive_ = FIDUCIAL_HEADER_LEN + len;

          // If header is invalid, give up on this read
          if (!mithl_fid::is_valid_receive_header(receive_buf_,FIDUCIAL_MAX_RECEIVE_LEN)) {
            // //publish_string_to_lcm(zcm_, "_FIDUCIAL_WARN_MESSAGE", "Warning: metadata structure was invalid.");
            // PublishBuffer((uint8_t*)receive_buf_, 8, "_FIDUCIAL_WARN_MESSAGE");

            frame_state_ = 0;
            max_chars_to_receive_ = FIDUCIAL_MAX_RECEIVE_LEN;
            break;
          }
        }

        frame_state_++;

        if (frame_state_ >= max_chars_to_receive_) { // Parse!
          ParseReceiveBuffer();
          frame_state_ = 0;
          max_chars_to_receive_ = FIDUCIAL_MAX_RECEIVE_LEN;
        }
    }
  }
}

void FiducialController::ParseReceiveBuffer(){
//  bool is_valid_buffer = mithl_fid::is_valid_receive_buffer(receive_buf_, FIDUCIAL_MAX_RECEIVE_LEN);

  // Check "ORDER" field of received buffer.
  uint8_t order = receive_buf_[1];
  uint16_t arg = (((uint16_t)(receive_buf_[3])) << 8) + receive_buf_[2];
  uint16_t len = (((uint16_t)(receive_buf_[5])) << 8) + receive_buf_[4];

  if (!mithl_fid::is_valid_receive_buffer(receive_buf_,FIDUCIAL_MAX_RECEIVE_LEN)) {
    if (len > FIDUCIAL_MAX_RECEIVE_LEN) {
      PublishBuffer((uint8_t*)receive_buf_, 8, "_FIDUCIAL_WARN2_MESSAGE");
    }
    publish_string_to_lcm(zcm_, "_FIDUCIAL_WARN_MESSAGE", "Warning: metadata (w/ buffer) structure was invalid.");
    // PublishBuffer((uint8_t*)receive_buf_, 8, "_FIDUCIAL_WARN_MESSAGE");
  }

  switch (order) {
    case 0: // Fiducial Module Indicating Bad Comm Received
    // order == 0 <--> (no examples)
    {
      // TODO: Publish error message buffer

      // Zero is the universal ACK; we treat it as an acknowledgement for any message
      waiting_for_ack_ = false;
      waiting_for_ack_order_ = 0;
      waiting_for_ack_arg_ = 0;
    }
    break;
    case 1: // Fiducial Module Indicating Receipt of Write
    // order == 1 <--> A 6.4.1	 Send Parameters to the Sensor RAM  Sensor->PC
    {
      // Not much to do here....
    }
    break;
    case 2: // Fiducial Module Reporting Current Param/Teach Table Values
    // order == 2 <--> A 6.4.2	 Read Parameters from the Sensor RAM  Sensor->PC// Publish ack message header
    {
      // Order=2. This could be reading config settings or color row depending on the value of ARG
      //
      // ARG = 0 parameter set 0 <- the one we use if IN0 = LO, it seems
      // ARG = 1 parameter set 1 <- the one we use if IN0 = HI
      // ARG = 2 teach vector set 0  <- the one we use if IN0 = LO
      // ARG = 3 teach vector set 1   <- the one we use if IN0 = HI

//      if (arg == 0) {
//        publish_string_to_lcm(zcm_, "RECV_ARG0", "");
//      } else if (arg == 1) {
//        publish_string_to_lcm(zcm_, "RECV_ARG1", "");
//      } else if (arg == 2) {
//        publish_string_to_lcm(zcm_, "RECV_ARG2", "");
//      } else if (arg == 3) {
//        publish_string_to_lcm(zcm_, "RECV_ARG3", "");
//      }

      if (len == 34) { // 34 bytes is exactly the length of the config frame
        mithl_fiducial_config_t incoming_config;
        double now = get_current_time();

        uint16_t set_no = waiting_for_ack_arg_ % 2;

        incoming_config.utime = now * 1000 * 1000;
        incoming_config.power = mithl_fid::read_word((uint16_t*)(&(receive_buf_[FIDUCIAL_HEADER_LEN + (2*0)])));
        incoming_config.powermode = mithl_fid::read_word((uint16_t*)(&(receive_buf_[FIDUCIAL_HEADER_LEN + (2*1)])));
        incoming_config.average = mithl_fid::read_word((uint16_t*)(&(receive_buf_[FIDUCIAL_HEADER_LEN + (2*2)])));
        incoming_config.evalmode = mithl_fid::read_word((uint16_t*)(&(receive_buf_[FIDUCIAL_HEADER_LEN + (2*3)])));
        incoming_config.hold = mithl_fid::read_word((uint16_t*)(&(receive_buf_[FIDUCIAL_HEADER_LEN + (2*4)])));
        incoming_config.intlim = mithl_fid::read_word((uint16_t*)(&(receive_buf_[FIDUCIAL_HEADER_LEN + (2*5)])));
        incoming_config.maxcolno = mithl_fid::read_word((uint16_t*)(&(receive_buf_[FIDUCIAL_HEADER_LEN + (2*6)])));
        incoming_config.outmode = mithl_fid::read_word((uint16_t*)(&(receive_buf_[FIDUCIAL_HEADER_LEN + (2*7)])));
        incoming_config.trigger = mithl_fid::read_word((uint16_t*)(&(receive_buf_[FIDUCIAL_HEADER_LEN + (2*8)])));
        incoming_config.exteach = mithl_fid::read_word((uint16_t*)(&(receive_buf_[FIDUCIAL_HEADER_LEN + (2*9)])));
        incoming_config.calcmode = mithl_fid::read_word((uint16_t*)(&(receive_buf_[FIDUCIAL_HEADER_LEN + (2*10)])));
        incoming_config.dyn_win_lo = mithl_fid::read_word((uint16_t*)(&(receive_buf_[FIDUCIAL_HEADER_LEN + (2*11)])));
        incoming_config.dyn_win_hi = mithl_fid::read_word((uint16_t*)(&(receive_buf_[FIDUCIAL_HEADER_LEN + (2*12)])));
        incoming_config.color_groups = mithl_fid::read_word((uint16_t*)(&(receive_buf_[FIDUCIAL_HEADER_LEN + (2*13)])));
        incoming_config.ledmode = mithl_fid::read_word((uint16_t*)(&(receive_buf_[FIDUCIAL_HEADER_LEN + (2*14)])));
        incoming_config.gain = mithl_fid::read_word((uint16_t*)(&(receive_buf_[FIDUCIAL_HEADER_LEN + (2*15)])));
        incoming_config.integral = mithl_fid::read_word((uint16_t*)(&(receive_buf_[FIDUCIAL_HEADER_LEN + (2*16)])));

        // for fields that are just  stored in class, not
        // stored in the fiducial sensor itself, just
        // populate them from knowledge.
        incoming_config.strip_width = strip_width;
        incoming_config.strip_space_between = strip_space_between;

        mithl_fiducial_config_t_publish(zcm_, "_FIDUCIAL_INCOMING_CONFIG", &incoming_config); // DB?

        memcpy(&(latest_config[set_no]), &incoming_config, sizeof(mithl_fiducial_config_t));  //latest_config = received_config;

        latest_config_known_ = true;
      } else if (len <= 8*31*2 && ((len % (8*2)) == 0)) { // Teach table has at most 31*8=248 words (and maybe can be multiple of 8 words less than that)

        if (len < 8*31*2) {
          publish_string_to_lcm(zcm_, "_FIDUCIAL_WARN2_MESSAGE", "Was less!");
        } else {
          publish_string_to_lcm(zcm_, "_FIDUCIAL_WARN2_MESSAGE", "Not less!");
        }
//        unsigned char three_chars[3];
//        PublishBuffer((uint8_t *)(&(three_chars[0])), 2, "_FIDUCIAL_WARN2_MESSAGE");
//        publish_string_to_lcm(zcm_, "_FIDUCIAL_WARN2_MESSAGE", three_chars);

        mithl_fiducial_teach_table_t incoming_table;
        double now = get_current_time();

        uint16_t set_no = waiting_for_ack_arg_ % 2;

        incoming_table.utime = now * 1000 * 1000;

        for (int i=0; i<FIDUCIAL_TEACH_TABLE_ROWS; i++) {
          mithl_fiducial_color_row_t incoming_row;

          // convenient python list: ['row_no','color_x','color_y','cto','color_int','ito','group']
          incoming_row.utime = now * 1000 * 1000;
          incoming_row.row_no = i;

          incoming_row.color_x =    mithl_fid::read_word((uint16_t*)(&(receive_buf_[FIDUCIAL_HEADER_LEN + (i*FIDUCIAL_COLOR_ROW_LEN) + (2*0)])));
          incoming_row.color_y =    mithl_fid::read_word((uint16_t*)(&(receive_buf_[FIDUCIAL_HEADER_LEN + (i*FIDUCIAL_COLOR_ROW_LEN) + (2*1)])));
          incoming_row.cto =        mithl_fid::read_word((uint16_t*)(&(receive_buf_[FIDUCIAL_HEADER_LEN + (i*FIDUCIAL_COLOR_ROW_LEN) + (2*2)])));
          incoming_row.color_int =  mithl_fid::read_word((uint16_t*)(&(receive_buf_[FIDUCIAL_HEADER_LEN + (i*FIDUCIAL_COLOR_ROW_LEN) + (2*3)])));
          incoming_row.ito =        mithl_fid::read_word((uint16_t*)(&(receive_buf_[FIDUCIAL_HEADER_LEN + (i*FIDUCIAL_COLOR_ROW_LEN) + (2*4)])));
          incoming_row.group =      mithl_fid::read_word((uint16_t*)(&(receive_buf_[FIDUCIAL_HEADER_LEN + (i*FIDUCIAL_COLOR_ROW_LEN) + (2*5)])));
          incoming_row.hold =       mithl_fid::read_word((uint16_t*)(&(receive_buf_[FIDUCIAL_HEADER_LEN + (i*FIDUCIAL_COLOR_ROW_LEN) + (2*6)])));
          // NOTE: there is one more double-byte at receive_buf_[FIDUCIAL_HEADER_LEN + (i*FIDUCIAL_COLOR_ROW_LEN) + (2*7)].
          //   This byte is a dummy value, always set to 0.

          incoming_table.rows[i] = incoming_row;
        }

        mithl_fiducial_teach_table_t_publish(zcm_, "_FIDUCIAL_INCOMING_TEACH_TABLE", &incoming_table); // Teach Table // DB?

        memcpy(&(latest_teach_table[set_no]), &incoming_table, sizeof(mithl_fiducial_teach_table_t));

        latest_teach_table_known_ = true;
      }
    }
    break;
    case 8:  // order == 8 <--> A 6.4.7 Data Frame  Sensor->PC
    {
      mithl_fiducial_color_dataval_t incoming_val;
      double now = get_current_time();
      // comvenient python list: ['color_r', 'color_g', 'color_b', 'color_x', 'color_y', 'color_int', 'color_delta_c', 'color_no', 'group', 'trig', 'temp', 'color_r_raw', 'color_g_raw', 'color_b_raw']
      incoming_val.utime = now * 1000 * 1000;
      incoming_val.color_r = mithl_fid::read_word((uint16_t*)(&(receive_buf_[FIDUCIAL_HEADER_LEN + (2*0)])));
      incoming_val.color_g = mithl_fid::read_word((uint16_t*)(&(receive_buf_[FIDUCIAL_HEADER_LEN + (2*1)])));
      incoming_val.color_b = mithl_fid::read_word((uint16_t*)(&(receive_buf_[FIDUCIAL_HEADER_LEN + (2*2)])));
      incoming_val.color_x = mithl_fid::read_word((uint16_t*)(&(receive_buf_[FIDUCIAL_HEADER_LEN + (2*3)])));
      incoming_val.color_y = mithl_fid::read_word((uint16_t*)(&(receive_buf_[FIDUCIAL_HEADER_LEN + (2*4)])));
      incoming_val.color_int = mithl_fid::read_word((uint16_t*)(&(receive_buf_[FIDUCIAL_HEADER_LEN + (2*5)])));
      incoming_val.color_delta_c = mithl_fid::read_word((uint16_t*)(&(receive_buf_[FIDUCIAL_HEADER_LEN + (2*6)])));
      incoming_val.color_no = mithl_fid::read_word((uint16_t*)(&(receive_buf_[FIDUCIAL_HEADER_LEN + (2*7)])));
      incoming_val.group = mithl_fid::read_word((uint16_t*)(&(receive_buf_[FIDUCIAL_HEADER_LEN + (2*8)])));
      incoming_val.trig = mithl_fid::read_word((uint16_t*)(&(receive_buf_[FIDUCIAL_HEADER_LEN + (2*9)])));
      incoming_val.temp = mithl_fid::read_word((uint16_t*)(&(receive_buf_[FIDUCIAL_HEADER_LEN + (2*10)])));
      incoming_val.color_r_raw = mithl_fid::read_word((uint16_t*)(&(receive_buf_[FIDUCIAL_HEADER_LEN + (2*11)])));
      incoming_val.color_g_raw = mithl_fid::read_word((uint16_t*)(&(receive_buf_[FIDUCIAL_HEADER_LEN + (2*12)])));
      incoming_val.color_b_raw = mithl_fid::read_word((uint16_t*)(&(receive_buf_[FIDUCIAL_HEADER_LEN + (2*13)])));

      mithl_fiducial_color_dataval_t_publish(zcm_, "_FIDUCIAL_INCOMING_DATAVAL", &incoming_val);
    }
    break;
    default:
    {
      // TODO: Offer cases for recognized messages that we choose to ignore
    }
  }

  // If we received expected ACK, update state to reflect
  if (waiting_for_ack_ && waiting_for_ack_order_==order) {
    waiting_for_ack_ = false;
    waiting_for_ack_order_ = 0;
    waiting_for_ack_arg_ = 0;
  }
}


/**
 * Submits command. Doesn't bother with ack callbacks
 * because ACK messages will always self-identify and
 * list the relevant sensor values.
 */
void FiducialController::SubmitCommandBuf(uint8_t * command_buf, int len, unsigned short ack_order, unsigned short ack_arg){
  // if another command is pending, fail us out immediately.
  // alternatively, timeout?
  if (!waiting_for_ack_){
    waiting_for_ack_ = true;
    waiting_for_ack_order_ = ack_order;
    waiting_for_ack_arg_ = ack_arg;
    // PublishBuffer(command_buf, len, "COMM_BUFFER_SUBMIT");
    //strncpy(command_buf_, command_buf, FIDUCIAL_MAX_COMMAND_LEN);
    serial_->write(command_buf, len);
    serial_->flush();
    command_send_time_ = get_current_time();
  }
}

void FiducialController::SetConfig(unsigned int channel, unsigned int set_no, mithl_fiducial_config_t * config){
  if (!mithl_fid::is_valid_config(config)) {
    publish_string_to_lcm(zcm_, "_FIDUCIAL_WARN2_MESSAGE", "Invalid Config!");
    return; // TODO: feel bad about how stupid this bug was
  }
  unsigned int buf_size = 8 + 34;
  uint8_t command_buf[buf_size];
  mithl_fid::write_config_command_buffer(config, command_buf, set_no); // populate buffer
  PublishBuffer((uint8_t *)command_buf, buf_size, "COMM_BUFFER_SUBMIT");
  SubmitCommandBuf((uint8_t *)command_buf, buf_size, 1, set_no);
}

void FiducialController::SetTeachTable(unsigned int channel, unsigned int set_no, mithl_fiducial_teach_table_t * teach_table) {
  // // TODO: implement validity test for teach table?
  unsigned int buf_size = 8 + (31*8*2);
  uint8_t command_buf[buf_size];
  mithl_fid::write_teach_table_command_buffer(teach_table, command_buf, set_no); // populate buffer
  SubmitCommandBuf((uint8_t *)command_buf, buf_size, 1, set_no);
}

void FiducialController::ReqConfig(unsigned int channel, unsigned int set_no){
  uint8_t command_buf[8];
  const uint8_t order = 2;
  const uint16_t arg = set_no;
  const uint16_t len = 0;
  mithl_fid::write_command_buffer_header(order, arg, len, 0xAA, command_buf);
  SubmitCommandBuf(command_buf, FIDUCIAL_HEADER_LEN, order, arg);
}

void FiducialController::ReqTeachTable(unsigned int channel, unsigned int set_no){
  uint8_t command_buf[8];
  const uint8_t order = 2;
  const uint16_t arg = 2+set_no;
  const uint16_t len = 0;
  mithl_fid::write_command_buffer_header(order, arg, len, 0xAA, command_buf);
  SubmitCommandBuf(command_buf, FIDUCIAL_HEADER_LEN, order, arg);
}

void FiducialController::ReqDataval(unsigned int channel){
  uint8_t command_buf[8];
  const uint8_t order = 8;
  const uint16_t arg = 0;
  const uint16_t len = 0;
  mithl_fid::write_command_buffer_header(order, arg, len, 0xAA, command_buf);
  SubmitCommandBuf(command_buf, FIDUCIAL_HEADER_LEN, order, arg);
}

void FiducialController::HandleConfig(const mithl_fiducial_config_t * msg){
  // store the values are to be set within this module's memory
  strip_width = msg->strip_width;
  strip_space_between = msg->strip_space_between;

  // aside from that, copy the desired config so we have it.
  //copy_known_elements(&latest_desired_config, msg);
  latest_desired_config = *msg; //TODO: Thread-safe for sim?
}

void FiducialController::HandleTeachTable(const mithl_fiducial_teach_table_t * msg){
  //copy_known_elements(&latest_desired_config, msg);
  latest_desired_teach_table = *msg; //TODO: Thread-safe for sim?
}

void FiducialController::PublishReceiveBuffer() {
  // Convert receive buf into ASCII hex string
  PublishBuffer((uint8_t*)receive_buf_, FIDUCIAL_MAX_COMMAND_LEN, "RECV_BUFFER_WHAT"); 
}

void FiducialController::PublishBuffer(uint8_t * buf, size_t length, const char * channel_name) {
  char my_c_str[3*length + 8]; // Strictly speaking, only need +1; +8 is an alignment/safety thing.
  for (int i=0; i<length; i++) {
    uint8_t whole_byte = (uint8_t)buf[i];
    uint8_t top_nib = (whole_byte >> 4);
    if (top_nib <= 0x0F) {
      my_c_str[3*i] = nib2chr[top_nib];
    } else {
      my_c_str[3*i] = '?';
    }
    uint8_t bottom_nib = (whole_byte & 0x0F);
    if (bottom_nib <= 0x0F) {
      my_c_str[3*i + 1] = nib2chr[bottom_nib];
    } else {
      my_c_str[3*i + 1] = '?';
    }
    my_c_str[3*i + 2] = ' '; // space, for readability
  }
  my_c_str[3*length] = '\0';

  // Publish hex string to LCM
  publish_string_to_lcm(zcm_, channel_name, my_c_str);
}

//
//static void config_modification_ack_handler(bool ack, void * extra){
//  ((FiducialController*)extra)->HandleConfigModificationAck(ack);
//}
//static void telemetry_string_ack_handler(bool ack, void * extra){
//  ((FiducialController*)extra)->HandleTelemetryStringAck(ack);
//}
//
//// maybe better with a macro but whatever, this saves some typing...
//template <typename type>
//static inline void copy_conditional_helper(type * dest, const type * source){
//  if (*source != -2) *dest = *source;
//}
//
//static void copy_known_elements(mithl_fiducial_config_t * dest,
//                                const mithl_fiducial_config_t * source){
//  // our command was committed, copy over the pending changes
//  copy_conditional_helper<int16_t>(&(dest->power), &(source->power));
//  copy_conditional_helper<int8_t>(&(dest->powermode), &(source->powermode));
//  copy_conditional_helper<int16_t>(&(dest->average), &(source->average));
//  copy_conditional_helper<int8_t>(&(dest->evalmode), &(source->evalmode));
//  copy_conditional_helper<int8_t>(&(dest->hold), &(source->hold));
//  copy_conditional_helper<int16_t>(&(dest->intlim), &(source->intlim));
//  copy_conditional_helper<int8_t>(&(dest->maxcolno), &(source->maxcolno));
//  copy_conditional_helper<int8_t>(&(dest->outmode), &(source->outmode));
//  copy_conditional_helper<int8_t>(&(dest->trigger), &(source->trigger));
//  copy_conditional_helper<int8_t>(&(dest->exteach), &(source->exteach));
//  copy_conditional_helper<int8_t>(&(dest->calcmode), &(source->calcmode));
//  copy_conditional_helper<int16_t>(&(dest->dyn_win_lo), &(source->dyn_win_lo));
//  copy_conditional_helper<int16_t>(&(dest->dyn_win_hi), &(source->dyn_win_hi));
//  copy_conditional_helper<int8_t>(&(dest->color_groups), &(source->color_groups));
//  copy_conditional_helper<int16_t>(&(dest->integral), &(source->integral));
//}

//void FiducialController::HandleConfigModificationAck(bool ack)/
//{
//  if (ack){
//    copy_known_elements(&latest_config, &tentative_config);
//  } else {
//    // our command was not commited, so hide the changes that didn't happen
//    memcpy(&tentative_config, &latest_config, sizeof(mithl_fiducial_config_t));
//  }
//}

//void FiducialController::SetSetpoint(unsigned int channel, float setpoint){
//  if (fabs(setpoint)>1.0001) return;
//  tentative_config.command = setpoint;
//  char command_buf[20];
//  sprintf(command_buf, "!G %1u %d_", channel, (int)(1000.0*setpoint));
//  SubmitCommandBuf(command_buf, &config_modification_ack_handler, (void *)this);
//}

//void FiducialController::ReqFaultFlag(){
//  char command_buf[30];
//  sprintf(command_buf, "?FF_");
//  SubmitCommandBuf(command_buf, NULL, NULL);
//}

//void FiducialController::SetESTOP(){
//  serial_->write("!EX_");
//}
//void FiducialController::ReleaseESTOP(){
//  serial_->write("!MG_");
//}

