#ifndef WARNING_SENDER_H
#define WARNING_SENDER_H

#include "arduino_simul_zcm_helpers.hpp"
#include "arduino_time_manager.h"
#include "zcmgen_c/mithl_string_t.h"

/* Utilities for sending generic warning messages from Arduinos in
   a readable way */


// send via LCM
static inline void send_warning_message_via_LCM(zcm_t * zcm, char * channel, char * string){
  mithl_string_t msg;
  msg.utime = get_current_time() * 1000 * 1000;
  msg.data = string;
  mithl_string_t_publish(zcm, channel, &msg);
}

#ifdef COMPILE_FOR_ARDUINO

// send via LCM
void send_warning_message(zcm_t * zcm, char * channel, char * string){
  send_warning_message_via_LCM(zcm, channel, string);
}

#else

// LCM and also printf it
void send_warning_message(zcm_t * zcm, char * channel, char * string){
  printf("%s >> %s\n", channel, string);
  send_warning_message_via_LCM(zcm, channel, string);
}

#endif

#endif