#ifndef ARDUINO_TIME_MANAGER_H
#define ARDUINO_TIME_MANAGER_H

#include "Arduino.h"
#include "arduino_simul_zcm_helpers.hpp"
#include "zcmgen_c/mithl_trigger_t.h"

/* Listens for ZCM broadcasts of the current global time,
   and learns to map the local clock to this global time:
    global_time = scaling * localtime + offset */


static void time_handler(const zcm_recv_buf_t * rbuf, const char * channel, 
  const mithl_trigger_t * msg, void * user);

void init_time_manager(zcm_t * zcm);

double get_current_time();

#endif