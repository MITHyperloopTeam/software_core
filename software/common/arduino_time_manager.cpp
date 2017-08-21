#include "arduino_time_manager.h"

/* Listens for ZCM broadcasts of the current global time,
   and learns to map the local clock to this global time:
    global_time = scaling * localtime + offset */

static double local_to_global_time_offset = 0.0;
static double local_to_global_time_scaling = 1.0;
static double last_local_t = 0.0, last_global_t = 0.0;
static bool initialized = false;

static uint32_t last_micros_time = 0;
static double collective_micros_wraparound_offset = 0.0;

static void time_handler(const zcm_recv_buf_t * rbuf, const char * channel, 
  const mithl_trigger_t * msg, void * user){
  double local_t = (double) micros() / 1000. / 1000.;
  double global_t = (double) msg->utime / 1000. / 1000.;

  bool ignore = false;
  if (initialized){
    if (local_t == last_local_t || global_t == last_global_t){
      // don't adjust anything, we either got a repeat mesage (same global t) or messages
      // too close in time (same micros). just wait for next message for life to improve
      ignore = true;
    } else if (last_local_t > local_t || last_global_t > global_t){
      // local time went backwards (rollover?), or global time went backwards (sim restart?)
      // keep scaling but recalculate the offset.
      local_to_global_time_offset = global_t - local_to_global_time_scaling * local_t;
    } else {
      local_to_global_time_scaling = (global_t - last_global_t) / (local_t - last_local_t);
      local_to_global_time_offset = global_t - local_to_global_time_scaling * local_t;
    }
  }

  if (!ignore){
    last_local_t = local_t;
    last_global_t = global_t;
    initialized = true;
  }
}

void init_time_manager(zcm_t * zcm){
    mithl_trigger_t_subscribe(zcm, "TIME", time_handler, NULL);
}

double get_current_time(){
  uint32_t new_micros = micros();
  if (last_micros_time > 1000*1000*1000 && (new_micros < last_micros_time - 1000*1000*1000)){
    collective_micros_wraparound_offset += 4294.967296; // 2^32 / 1000 / 1000. 
  }
  last_micros_time = new_micros;
  return local_to_global_time_scaling * (((double)new_micros) / 1000. / 1000. + collective_micros_wraparound_offset) + local_to_global_time_offset;
}