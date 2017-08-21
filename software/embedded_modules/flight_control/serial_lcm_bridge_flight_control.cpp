 /** 
**/

#include "linux_cobs_serial_transport.hpp"
#include "zcmgen_c/mithl_vectorXf_t.h"
#include "zcmgen_c/mithl_trigger_t.h"
#include "zcmgen_c/mithl_config_t.h"
#include "zcmgen_c/mithl_flight_control_high_rate_t.h"
#include "zcmgen_c/mithl_flight_control_low_rate_t.h"
#include "zcmgen_c/mithl_fiducial_t.h"
#include "zcmgen_c/mithl_state_estimator_particle.h"
#include "zcmgen_c/mithl_state_t.h"
#include "zcmgen_c/mithl_state_estimator_particle_set.h"
#include "zcmgen_c/mithl_state_estimator_state_t.h"
#include "zcmgen_c/mithl_state_estimator_config_t.h"
#include "zcmgen_c/mithl_net_diag_t.h"
#include "zcmgen_c/mithl_net_health_t.h"

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include "common_utils.h"

int myid = 53;

zcm_t * zcm_udp;
zcm_t * zcm_serial;

// updater for udp zcm
void *zcmUDPUpater(void *arg)
{
  while(1)
  {
    zcm_handle(zcm_udp);
  }
}

// up only
static void net_health_handler(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_net_health_t *msg, void *user)
{
  mithl_net_health_t_publish(zcm_udp, channel, msg);
}

static void net_diag_handler_up(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_net_diag_t *msg, void *user)
{
  mithl_net_diag_t_publish(zcm_udp, channel, msg);
}
static void net_diag_handler_down(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_net_diag_t *msg, void *user)
{
  // every arduino has an ID >= 50; don't send those down
  if (msg->origin_id < 50 || msg->dest_id == myid)
    mithl_net_diag_t_publish(zcm_serial, channel, msg);
}

// UP
static void bc_handler(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_vectorXf_t *msg, void *user)
{
  mithl_vectorXf_t_publish(zcm_udp, channel, msg);
}

// DOWN
static void sim_fb_handler(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_vectorXf_t *msg, void *user)
{
  mithl_vectorXf_t_publish(zcm_serial, channel, msg);
}
static void reset_handler(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_trigger_t *msg, void *user)
{
  mithl_trigger_t_publish(zcm_serial, channel, msg);
}
static void time_handler(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_trigger_t *msg, void *user)
{
  mithl_trigger_t_publish(zcm_serial, channel, msg);
}
static void tare_handler(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_trigger_t *msg, void *user)
{
  mithl_trigger_t_publish(zcm_serial, channel, msg);
}
static void config_request_handler(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_trigger_t *msg, void *user)
{
  mithl_trigger_t_publish(zcm_udp, channel, msg);
}
static void config_handler(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_config_t *msg, void *user)
{
  mithl_config_t_publish(zcm_serial, channel, msg);
}
static void imu_handler(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_vectorXf_t *msg, void *user)
{
  mithl_vectorXf_t_publish(zcm_udp, channel, msg);
}

static void mithl_flight_control_low_rate_t_handler_up(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_flight_control_low_rate_t *msg, void *user)
{
  mithl_flight_control_low_rate_t_publish(zcm_udp, channel, msg);
}
static void mithl_flight_control_high_rate_t_handler_up(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_flight_control_high_rate_t *msg, void *user)
{
  mithl_flight_control_high_rate_t_publish(zcm_udp, channel, msg);
}
static void mithl_state_estimator_particle_set_handler_up(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_state_estimator_particle_set *msg, void *user)
{
  mithl_state_estimator_particle_set_publish(zcm_udp, channel, msg);
}
static void mithl_fiducial_t_handler_down(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_fiducial_t *msg, void *user)
{
  mithl_fiducial_t_publish(zcm_serial, channel, msg);
}
static void mithl_state_t_handler_down(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_state_t *msg, void *user)
{
  mithl_state_t_publish(zcm_serial, channel, msg);
}
static void mithl_state_estimator_config_t_handler_down(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_state_estimator_config_t *msg, void *user)
{
  mithl_state_estimator_config_t_publish(zcm_serial, channel, msg);
}
static void mithl_state_estimator_config_t_handler_up(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_state_estimator_config_t *msg, void *user)
{
  mithl_state_estimator_config_t_publish(zcm_udp, channel, msg);
}
static void mithl_state_estimator_state_t_handler_up(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_state_estimator_state_t *msg, void *user)
{
  mithl_state_estimator_state_t_publish(zcm_udp, channel, msg);
}

int
main( int    argc,
      char** argv  )
{
  char * serial_port;
  if (argc == 2)
    serial_port = argv[1];
  else
    serial_port = "/dev/mithl_flight_control";

  zcm_trans_t * linux_cobs_serial_transport = linux_cobs_serial_transport_create(serial_port);
  zcm_serial = zcm_create_trans(linux_cobs_serial_transport);
  zcm_udp = zcm_create("udpm://239.255.76.67:7667?ttl=0");


  mithl_net_diag_t_subscribe(zcm_udp, "_NREP", &net_diag_handler_down, NULL);
  mithl_net_diag_t_subscribe(zcm_udp, "_NREQ", &net_diag_handler_down, NULL);
  mithl_net_diag_t_subscribe(zcm_serial, "_NREP", &net_diag_handler_up, NULL);
  mithl_net_diag_t_subscribe(zcm_serial, "_NREQ", &net_diag_handler_up, NULL);
  mithl_net_health_t_subscribe(zcm_serial, "_NSUM", &net_health_handler, NULL);
  
  mithl_vectorXf_t_subscribe(zcm_serial, "_BC", &bc_handler, NULL);
  //mithl_vectorXf_t_subscribe(zcm_udp, "SIM_FB", &sim_fb_handler, NULL);
  mithl_trigger_t_subscribe(zcm_udp, "RESET", &reset_handler, NULL);
  mithl_trigger_t_subscribe(zcm_udp, "TIME", &time_handler, NULL);
  mithl_trigger_t_subscribe(zcm_udp, "TARE", &tare_handler, NULL);
  mithl_trigger_t_subscribe(zcm_serial, "CNFG_RQST", &config_request_handler, NULL);
  mithl_config_t_subscribe(zcm_udp, "CNFG", &config_handler, NULL);
  mithl_vectorXf_t_subscribe(zcm_serial, "IMU", &imu_handler, NULL);

  mithl_flight_control_low_rate_t_subscribe(zcm_serial, "_FC_OL", &mithl_flight_control_low_rate_t_handler_up, NULL);
  mithl_flight_control_high_rate_t_subscribe(zcm_serial, "_FC_OH", &mithl_flight_control_high_rate_t_handler_up, NULL);
  mithl_state_estimator_particle_set_subscribe(zcm_serial, "_FC_SE", &mithl_state_estimator_particle_set_handler_up, NULL);

  mithl_state_t_subscribe(zcm_udp, "FSM_STATE", &mithl_state_t_handler_down, NULL);
  mithl_fiducial_t_subscribe(zcm_udp, "_FD_O", &mithl_fiducial_t_handler_down, NULL);
  mithl_state_estimator_state_t_subscribe(zcm_serial, "_FC_SE_STATE", &mithl_state_estimator_state_t_handler_up, NULL);
  mithl_state_estimator_config_t_subscribe(zcm_serial, "_FC_SE_CONFIG", &mithl_state_estimator_config_t_handler_up, NULL);
  mithl_state_estimator_config_t_subscribe(zcm_udp, "_FC_SE_CONFIG_SET", &mithl_state_estimator_config_t_handler_down, NULL);

  double now = getUnixTime();

  pthread_t pth;
  pthread_create(&pth,NULL,zcmUDPUpater, NULL);

  while (1) {
    zcm_handle_nonblock(zcm_serial);
    if (getUnixTime() - now > 1.0){
      now = getUnixTime();
      printf("Elapsed: %f\n", now);
    }
    usleep(1);
  }
}
