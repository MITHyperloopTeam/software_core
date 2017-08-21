/** 
**/

#include "linux_cobs_serial_transport.hpp"
#include "zcmgen_c/mithl_vectorXf_t.h"
#include "zcmgen_c/mithl_trigger_t.h"
#include "zcmgen_c/mithl_adm_config_t.h"
#include "zcmgen_c/mithl_adm_status_t.h"
#include "zcmgen_c/mithl_bac_config_t.h"
#include "zcmgen_c/mithl_bac_teleop_cmd_t.h"
#include "zcmgen_c/mithl_bac_auto_cmd_t.h"
#include "zcmgen_c/mithl_bac_mode_t.h"
#include "zcmgen_c/mithl_bac_state_high_rate_t.h"
#include "zcmgen_c/mithl_bac_state_low_rate_t.h"
#include "zcmgen_c/mithl_low_speed_low_rate_t.h"
#include "zcmgen_c/mithl_net_diag_t.h"
#include "zcmgen_c/mithl_net_health_t.h"
#include "zcmgen_c/mithl_trigger_t.h"
#include "zcmgen_c/mithl_state_t.h"
#include "zcmgen_c/mithl_vectorXf_t.h"

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include "common_utils.h"

int myid = 54;

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

static void trigger_handler_down(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_trigger_t *msg, void *user)
{
  mithl_trigger_t_publish(zcm_serial, channel, msg);
}

static void vector_handler(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_vectorXf_t *msg, void *user)
{
  mithl_vectorXf_t_publish(zcm_udp, channel, msg);
}

static void config_handler_up(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_adm_config_t *msg, void *user)
{ 
  mithl_adm_config_t_publish(zcm_udp, channel, msg);
}
static void config_handler_down(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_adm_config_t *msg, void *user)
{
  mithl_adm_config_t_publish(zcm_serial, channel, msg);
}

static void status_handler_up(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_adm_status_t *msg, void *user)
{ 
  mithl_adm_status_t_publish(zcm_udp, channel, msg);
}

static void bac_config_handler_up(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_bac_config_t *msg, void *user)
{ 
  mithl_bac_config_t_publish(zcm_udp, channel, msg);
}
static void bac_config_handler_down(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_bac_config_t *msg, void *user)
{ 
  mithl_bac_config_t_publish(zcm_serial, channel, msg);
}

static void bac_auto_cmd_handler_up(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_bac_auto_cmd_t *msg, void *user)
{ 
  mithl_bac_auto_cmd_t_publish(zcm_udp, channel, msg);
}
static void bac_auto_cmd_handler_down(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_bac_auto_cmd_t *msg, void *user)
{ 
  mithl_bac_auto_cmd_t_publish(zcm_serial, channel, msg);
}

static void bac_teleop_cmd_handler_up(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_bac_teleop_cmd_t *msg, void *user)
{ 
  mithl_bac_teleop_cmd_t_publish(zcm_udp, channel, msg);
}
static void bac_teleop_cmd_handler_down(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_bac_teleop_cmd_t *msg, void *user)
{ 
  mithl_bac_teleop_cmd_t_publish(zcm_serial, channel, msg);
}

static void bac_mode_handler_up(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_bac_mode_t *msg, void *user)
{ 
  mithl_bac_mode_t_publish(zcm_udp, channel, msg);
}
static void bac_mode_handler_down(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_bac_mode_t *msg, void *user)
{ 
  mithl_bac_mode_t_publish(zcm_serial, channel, msg);
}

static void bac_state_high_rate_t_handler_up(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_bac_state_high_rate_t *msg, void *user)
{ 
  mithl_bac_state_high_rate_t_publish(zcm_udp, channel, msg);
}

static void bac_state_low_rate_t_handler_up(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_bac_state_low_rate_t *msg, void *user)
{ 
  mithl_bac_state_low_rate_t_publish(zcm_udp, channel, msg);
}

static void low_speed_low_rate_t_handler_up(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_low_speed_low_rate_t *msg, void *user)
{ 
  mithl_low_speed_low_rate_t_publish(zcm_udp, channel, msg);
}

static void state_t_handler_down(const zcm_recv_buf_t *rbuf, const char *channel,
                       const mithl_state_t *msg, void *user)
{ 
  mithl_state_t_publish(zcm_serial, channel, msg);
}

int
main( int    argc,
      char** argv  )
{
  char * serial_port;
  if (argc == 2)
    serial_port = argv[1];
  else
    serial_port = "/dev/mithl_low_speed";

  zcm_trans_t * linux_cobs_serial_transport = linux_cobs_serial_transport_create(serial_port);
  zcm_serial = zcm_create_trans(linux_cobs_serial_transport);
  zcm_udp = zcm_create("udpm://239.255.76.67:7667?ttl=0");

  mithl_net_diag_t_subscribe(zcm_udp, "_NREP", &net_diag_handler_down, NULL);
  mithl_net_diag_t_subscribe(zcm_udp, "_NREQ", &net_diag_handler_down, NULL);
  mithl_net_diag_t_subscribe(zcm_serial, "_NREP", &net_diag_handler_up, NULL);
  mithl_net_diag_t_subscribe(zcm_serial, "_NREQ", &net_diag_handler_up, NULL);
  mithl_net_health_t_subscribe(zcm_serial, "_NSUM", &net_health_handler, NULL);

  mithl_trigger_t_subscribe(zcm_udp, "_ESTOP", &trigger_handler_down, NULL);
  mithl_trigger_t_subscribe(zcm_udp, "_RADM_CLAMP_REFLASH", &trigger_handler_down, NULL);
  mithl_trigger_t_subscribe(zcm_udp, "_RADM_WHEEL_REFLASH", &trigger_handler_down, NULL);

  mithl_vectorXf_t_subscribe(zcm_serial, "VEC", &vector_handler, NULL);
  mithl_vectorXf_t_subscribe(zcm_serial, "_BAC_ARM_COUNTDOWN", &vector_handler, NULL);
  mithl_adm_config_t_subscribe(zcm_serial, "_RADM_CLAMP_CURCONF", &config_handler_up, NULL);
  mithl_adm_status_t_subscribe(zcm_serial, "_RADM_CLAMP_STATUS", &status_handler_up, NULL);
  mithl_adm_config_t_subscribe(zcm_udp, "_RADM_CLAMP_SETCONF", &config_handler_down, NULL);
  
  mithl_adm_config_t_subscribe(zcm_serial, "_RADM_WHEEL_CURCONF", &config_handler_up, NULL);
  mithl_adm_status_t_subscribe(zcm_serial, "_RADM_WHEEL_STATUS", &status_handler_up, NULL);
  mithl_adm_config_t_subscribe(zcm_udp, "_RADM_WHEEL_SETCONF", &config_handler_down, NULL);

  mithl_bac_config_t_subscribe(zcm_serial, "_BAC_CONFIG_STATE", &bac_config_handler_up, NULL);
  mithl_bac_config_t_subscribe(zcm_udp, "_BAC_CONFIG_SET", &bac_config_handler_down, NULL);
  mithl_bac_auto_cmd_t_subscribe(zcm_serial, "_BAC_AUTO_CMD_STATE", &bac_auto_cmd_handler_up, NULL);
  mithl_bac_auto_cmd_t_subscribe(zcm_udp, "_BAC_AUTO_CMD_SET", &bac_auto_cmd_handler_down, NULL);
  mithl_bac_teleop_cmd_t_subscribe(zcm_serial, "_BAC_TELEOP_CMD_STATE", &bac_teleop_cmd_handler_up, NULL);
  mithl_bac_teleop_cmd_t_subscribe(zcm_udp, "_BAC_TELEOP_CMD_SET", &bac_teleop_cmd_handler_down, NULL);
  mithl_bac_mode_t_subscribe(zcm_serial, "_BAC_MODE_STATE", &bac_mode_handler_up, NULL);
  mithl_bac_mode_t_subscribe(zcm_udp, "_BAC_MODE_SET", &bac_mode_handler_down, NULL);
  mithl_bac_state_high_rate_t_subscribe(zcm_serial, "_BAC_STATE_H", &bac_state_high_rate_t_handler_up, NULL);
  mithl_bac_state_low_rate_t_subscribe(zcm_serial, "_BAC_STATE_L", &bac_state_low_rate_t_handler_up, NULL);

  mithl_low_speed_low_rate_t_subscribe(zcm_serial, "_LS_OL", &low_speed_low_rate_t_handler_up, NULL);

  mithl_state_t_subscribe(zcm_udp, "FSM_STATE", &state_t_handler_down, NULL);
  
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
